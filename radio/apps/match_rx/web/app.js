const state = { latest: null, busy: false };
let activePage = location.hash === "#radio" ? "radio" : "radar";
let radarEventSource = null;
let radarReconnectTimer = 0;
let radarStreamState = "closed";
let radarGeneration = 0;
let radarLatest = null;
let radarDirty = true;
let selectedRobot = null;
let radarMapOwnSide = "red";
const radarDisplayPositions = {};
const radarHitTargets = [];

// 每路保留"每个命令号最近一帧"，让信息波(0x0A01~05轮播)能同时稳定显示全部字段，
// 而不是只显示碰巧最后到达的那一个命令。key: role -> { cmd_hex -> {frame,timestamp} }
const latestByCmd = { rx1: {}, rx2: {} };
const frameArrivalHistory = { rx1: {}, rx2: {} };
const FRAME_RATE_WINDOW_SEC = 1;
let lastFrameSeen = { rx1: 0, rx2: 0 };

// 频谱通过 SSE latest-only 推送，rAF 做 60fps 插值；瀑布图逐 FFT 行滚动。
// target[role] = { freqs, power(目标), meta }; smooth[role].power = 当前显示值。
const spectrumTarget = { rx1: null, rx2: null };
const spectrumSmooth = { rx1: null, rx2: null };
const waterfallQueues = { rx1: [], rx2: [] };
let spectrumDirty = true;        // 首帧先画“等待数据”，之后仅在目标变化时重绘
let framesDirty = false;         // 帧文本需要重渲染
let streamRequestBusy = false;   // SSE 断线时的 HTTP fallback 不允许重叠
let spectrumEventSource = null;
let spectrumReconnectTimer = 0;
let spectrumStreamState = "closed";
let lastFrameSequence = 0;
let lastSpectrumRenderMs = 0;
let lastFrameRenderMs = 0;

const displayPerf = {
  intervalStartMs: performance.now(),
  renderFrames: 0,
  updates: { rx1: 0, rx2: 0 },
  latencySamples: [],
  transportSamples: [],
  droppedWaterfallRows: 0,
  latest: { render_fps: 0, rx1_hz: 0, rx2_hz: 0, latency_ms: 0, transport_ms: 0 },
};


const roles = {
  rx1: "RX1 信息波",
  rx2: "RX2 干扰波",
  referee: "裁判串口",
  recorder: "比赛内录",
  radar_integration: "雷达融合",
  vision_radar: "视觉雷达",
  rx1_sdr: "RX1 ANTSDR",
  rx2_sdr: "RX2 ANTSDR",
  rx1_frame_rate: "RX1 信息波帧率",
  rx2_frame_rate: "RX2 干扰波帧率",
  rx1_business_frames: "RX1 业务帧完整性",
  rx2_business_frames: "RX2 密钥帧完整性",
  rx1_tuning: "RX1 频点 / 增益一致性",
  rx2_tuning: "RX2 频点 / 等级一致性",
  rx1_signal: "RX1 ADC 信号质量",
  rx2_signal: "RX2 ADC 信号质量",
  rx1_decode_quality: "RX1 CRC / 丢包 / 时序",
  rx2_decode_quality: "RX2 CRC / 丢包 / 时序",
  rx1_spectrum: "RX1 频谱数据流",
  rx2_spectrum: "RX2 频谱数据流",
  referee_stability: "裁判串口稳定性",
  radar_output: "0x0305 坐标发送",
  coordinate_coverage: "融合坐标覆盖率",
  vision_performance: "视觉处理性能",
  display_performance: "浏览器显示性能",
};

const robotLabels = {
  opponent_hero: "英雄", opponent_engineer: "工程", opponent_infantry_3: "步兵3",
  opponent_infantry_4: "步兵4", opponent_aerial: "空中", opponent_sentry: "哨兵",
  hero: "英雄", engineer: "工程", standard_3: "步兵3", standard_4: "步兵4",
  aerial: "空中", sentry: "哨兵", reserved: "保留",
};

const mainStatusLabels = {
  alive: "存活",
  destroyed: "战亡",
  invincible_not_weakened: "无敌但不虚弱",
  invincible_weakened: "无敌且虚弱",
};

const sentryModeLabels = {
  offensive: "进攻姿态",
  defensive: "防御姿态",
  mobile: "移动姿态",
  enhanced_offensive: "强化进攻姿态",
  enhanced_defensive: "强化防御姿态",
  enhanced_mobile: "强化移动姿态",
};

const broadcastFrames = [
  ["0x0A01", "坐标"],
  ["0x0A02", "血量"],
  ["0x0A03", "发弹量"],
  ["0x0A04", "经济"],
  ["0x0A05", "增益状态"],
];

const $ = (id) => document.getElementById(id);

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;").replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;").replaceAll('"', "&quot;");
}

function fmtAge(timestamp) {
  if (!timestamp) return "—";
  const age = Math.max(0, Date.now() / 1000 - Number(timestamp));
  if (age < 1) return "刚刚";
  if (age < 60) return `${age.toFixed(0)} 秒前`;
  return `${(age / 60).toFixed(1)} 分钟前`;
}

function fmtRecentArrival(timestamp, now = Date.now() / 1000) {
  if (!timestamp) return "未到达";
  const age = Math.max(0, now - Number(timestamp));
  if (age < 0.05) return "刚刚";
  if (age < 10) return `${age.toFixed(1)} 秒前`;
  if (age < 60) return `${age.toFixed(0)} 秒前`;
  return `${(age / 60).toFixed(1)} 分钟前`;
}

function fmtHz(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  if (Math.abs(number) >= 1e6) return `${(number / 1e6).toFixed(3)} MHz`;
  if (Math.abs(number) >= 1e3) return `${(number / 1e3).toFixed(1)} kHz`;
  return `${number.toFixed(0)} Hz`;
}

function get(obj, path, fallback = undefined) {
  let cur = obj;
  for (const key of path) {
    if (!cur || typeof cur !== "object" || !(key in cur)) return fallback;
    cur = cur[key];
  }
  return cur;
}

async function apiGet(path) {
  const response = await fetch(path, { cache: "no-store" });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) throw new Error(payload.error || `${path} ${response.status}`);
  return payload;
}

async function apiPost(path, body = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) throw new Error(payload.error || `${path} ${response.status}`);
  return payload;
}

function stateClass(value) {
  return ["ok", "warn", "bad", "unknown"].includes(value) ? value : "unknown";
}

function stateLabel(value) {
  return { ok: "正常", warn: "WARN", bad: "ERROR", unknown: "待检测" }[stateClass(value)];
}

/* ---------- Spectrum: real frequency axis, adaptive dB, peak markers ---------- */
function drawSpectrum() {
  const canvas = $("spectrumCanvas");
  const ctx = canvas.getContext("2d");
  // 高 DPI 全尺寸 Canvas 是页面 GPU/合成开销的主要来源之一；频谱线无需
  // 使用 3x/4x 像素密度，1.5x 已足够清晰。
  const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
  const cssW = canvas.clientWidth || 640;
  const cssH = canvas.clientHeight || 260;
  const pxW = Math.round(cssW * dpr);
  const pxH = Math.round(cssH * dpr);
  if (canvas.width !== pxW || canvas.height !== pxH) { canvas.width = pxW; canvas.height = pxH; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);

  const padL = 46, padR = 14, padT = 16, padB = 30;
  const plotW = Math.max(10, cssW - padL - padR);
  const plotH = Math.max(10, cssH - padT - padB);

  const datasets = [["rx1", "#3a8fd8", "信息波"], ["rx2", "#2fbf8f", "干扰波"]];

  // 汇总频率范围与 dB 范围（两路合并，用真实 Hz 频率轴）；读取平滑后的显示缓冲。
  let fMin = Infinity, fMax = -Infinity, dbMin = Infinity, dbMax = -Infinity, latest = 0;
  const traces = [];
  for (const [role, color, label] of datasets) {
    const sm = spectrumSmooth[role];
    if (!sm || !sm.freqs || sm.freqs.length < 2) continue;
    latest = Math.max(latest, Number(sm.timestamp || 0));
    const freqs = sm.freqs;
    const powers = sm.power;
    for (const f of freqs) { if (f < fMin) fMin = f; if (f > fMax) fMax = f; }
    for (const p of powers) { const v = Number(p); if (Number.isFinite(v)) { if (v < dbMin) dbMin = v; if (v > dbMax) dbMax = v; } }
    traces.push({ role, color, label, freqs, powers, spec: sm.meta });
  }

  if (!traces.length) {
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = "13px Segoe UI, sans-serif";
    ctx.fillText("等待频谱数据…", padL + 8, padT + plotH / 2);
    $("spectrumAge").textContent = "无数据";
    return;
  }

  // dB 轴自适应留边并对齐到 10
  dbMax = Math.min(0, Math.ceil((dbMax + 5) / 10) * 10);
  dbMin = Math.floor((dbMin - 5) / 10) * 10;
  if (dbMax - dbMin < 30) dbMin = dbMax - 30;
  const xOf = (f) => padL + ((f - fMin) / Math.max(1, fMax - fMin)) * plotW;
  const yOf = (db) => padT + ((dbMax - Math.min(dbMax, Math.max(dbMin, db))) / (dbMax - dbMin)) * plotH;

  // 网格 + dB 标签
  ctx.strokeStyle = "rgba(255,255,255,0.08)";
  ctx.fillStyle = "rgba(255,255,255,0.45)";
  ctx.lineWidth = 1;
  ctx.font = "11px Segoe UI, sans-serif";
  ctx.textAlign = "left";
  const dbSteps = 4;
  for (let i = 0; i <= dbSteps; i += 1) {
    const db = dbMax - ((dbMax - dbMin) * i) / dbSteps;
    const y = yOf(db);
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + plotW, y); ctx.stroke();
    ctx.fillText(`${db}`, 6, y + 3);
  }
  // 频率轴标签（MHz）
  ctx.textAlign = "center";
  const fSteps = 5;
  for (let i = 0; i <= fSteps; i += 1) {
    const f = fMin + ((fMax - fMin) * i) / fSteps;
    const x = xOf(f);
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.fillText(`${(f / 1e6).toFixed(2)}`, x, cssH - 16);
  }
  ctx.fillStyle = "rgba(255,255,255,0.4)";
  ctx.fillText("MHz", padL + plotW / 2, cssH - 3);
  ctx.textAlign = "left";

  // 画每条谱线（填充 + 描边 + 峰值标记）
  for (const t of traces) {
    ctx.beginPath();
    t.freqs.forEach((f, i) => {
      const x = xOf(f), y = yOf(Number(t.powers[i]));
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    // 填充到底
    ctx.lineTo(xOf(t.freqs[t.freqs.length - 1]), padT + plotH);
    ctx.lineTo(xOf(t.freqs[0]), padT + plotH);
    ctx.closePath();
    ctx.fillStyle = t.color + "22";
    ctx.fill();
    // 描边
    ctx.beginPath();
    t.freqs.forEach((f, i) => {
      const x = xOf(f), y = yOf(Number(t.powers[i]));
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = t.color;
    ctx.lineWidth = 1.6;
    ctx.stroke();
    // 峰值竖线 + 读数
    const peakF = Number(t.spec.peak_frequency_hz);
    const peakDb = Number(t.spec.peak_dbfs);
    if (Number.isFinite(peakF) && peakF >= fMin && peakF <= fMax) {
      const px = xOf(peakF);
      ctx.strokeStyle = t.color;
      ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.moveTo(px, padT); ctx.lineTo(px, padT + plotH); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = t.color;
      ctx.font = "11px Segoe UI, sans-serif";
      const txt = `${t.label} ${(peakF / 1e6).toFixed(3)}MHz ${Number.isFinite(peakDb) ? peakDb.toFixed(0) : "?"}dB`;
      ctx.fillText(txt, Math.min(px + 4, padL + plotW - 130), padT + 12 + (t.role === "rx2" ? 14 : 0));
    }
  }

  $("spectrumAge").textContent = latest ? `更新 ${fmtAge(latest)}` : "无数据";
}

const waterfallLut = (() => {
  const stops = [
    [0.00, [2, 5, 12]],
    [0.18, [18, 31, 83]],
    [0.38, [18, 112, 155]],
    [0.58, [36, 190, 128]],
    [0.76, [235, 218, 72]],
    [0.90, [239, 92, 42]],
    [1.00, [255, 244, 225]],
  ];
  const lut = new Uint8ClampedArray(256 * 3);
  for (let i = 0; i < 256; i += 1) {
    const t = i / 255;
    let left = stops[0], right = stops[stops.length - 1];
    for (let j = 1; j < stops.length; j += 1) {
      if (t <= stops[j][0]) { left = stops[j - 1]; right = stops[j]; break; }
    }
    const u = Math.max(0, Math.min(1, (t - left[0]) / Math.max(1e-9, right[0] - left[0])));
    for (let c = 0; c < 3; c += 1) lut[i * 3 + c] = Math.round(left[1][c] + (right[1][c] - left[1][c]) * u);
  }
  return lut;
})();

// 绝对 dBFS 色标：两路接收机及 raw/filtered 观察点严格共用，颜色可直接横向比较。
const WATERFALL_DBFS_MIN = -120;
const WATERFALL_DBFS_MAX = 0;

function enqueueWaterfall(role, target) {
  const queue = waterfallQueues[role];
  queue.push({
    freqs: target.freqs.slice(),
    power: target.power.slice(),
    meta: target.meta,
  });
  while (queue.length > 8) {
    queue.shift();
    displayPerf.droppedWaterfallRows += 1;
  }
}

function drawWaterfallRow(role, row) {
  const canvas = $(`${role}Waterfall`);
  if (!canvas || !row.power.length) return;
  const width = Math.max(64, Math.round(canvas.clientWidth || 480));
  const height = Math.max(48, Math.round(canvas.clientHeight || 120));
  const ctx = canvas.getContext("2d", { alpha: false });
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    ctx.fillStyle = "#02060a";
    ctx.fillRect(0, 0, width, height);
  } else if (height > 1) {
    ctx.drawImage(canvas, 0, 1, width, height - 1, 0, 0, width, height - 1);
  }

  const image = ctx.createImageData(width, 1);
  for (let x = 0; x < width; x += 1) {
    const index = Math.min(row.power.length - 1, Math.floor((x / Math.max(1, width - 1)) * row.power.length));
    const value = Number(row.power[index]);
    const normalized = Number.isFinite(value)
      ? Math.max(0, Math.min(1, (value - WATERFALL_DBFS_MIN) / (WATERFALL_DBFS_MAX - WATERFALL_DBFS_MIN)))
      : 0;
    const color = Math.round(normalized * 255);
    image.data[x * 4] = waterfallLut[color * 3];
    image.data[x * 4 + 1] = waterfallLut[color * 3 + 1];
    image.data[x * 4 + 2] = waterfallLut[color * 3 + 2];
    image.data[x * 4 + 3] = 255;
  }
  ctx.putImageData(image, 0, height - 1);

  const first = row.freqs[0], last = row.freqs[row.freqs.length - 1];
  $(`${role}WaterfallRange`).textContent = `${(first / 1e6).toFixed(3)} — ${(last / 1e6).toFixed(3)} MHz · ${WATERFALL_DBFS_MIN}…${WATERFALL_DBFS_MAX} dBFS`;
}

function flushWaterfalls() {
  for (const role of ["rx1", "rx2"]) {
    const queue = waterfallQueues[role];
    while (queue.length) drawWaterfallRow(role, queue.shift());
  }
}

/* ---------- Signal quality tiles ---------- */
function qualityCard(role, data) {
  const status = get(data, ["statuses", role], {});
  const payload = status.data || {};
  const iq = get(payload, ["decoder", "iq_diagnostics"], {});
  const filteredIq = get(payload, ["decoder", "filtered_iq_diagnostics"], {});
  const stats = get(payload, ["decoder", "stats"], {});
  const spec = spectrumTarget[role]?.meta || get(data, ["spectra", role, "data"], {});
  const label = role === "rx1" ? "信息波" : "干扰波";
  const rms = Number.isFinite(iq.rms_dbfs) ? `${iq.rms_dbfs.toFixed(1)} dBFS` : "—";
  const filteredRms = Number.isFinite(filteredIq.rms_dbfs) ? `${filteredIq.rms_dbfs.toFixed(1)} dBFS` : "—";
  const headroom = Number.isFinite(iq.headroom_db) ? `${iq.headroom_db.toFixed(1)} dB` : "—";
  const clip = Number.isFinite(iq.clip_fraction) ? `${(iq.clip_fraction * 100).toFixed(3)}%` : "—";
  const peak = Number.isFinite(spec.peak_dbfs) ? `${Number(spec.peak_dbfs).toFixed(1)} dBFS` : "—";
  const peakOff = Number.isFinite(spec.peak_offset_hz) ? fmtHz(spec.peak_offset_hz) : "—";
  const frames = stats.frames ?? 0;
  const gain = data.rx1_gain || {};
  const gainMin = Number(gain.min_db ?? -1);
  const gainMax = Number(gain.max_db ?? 73);
  const gainText = role === "rx1"
    ? `增益 ${Number.isFinite(gain.actual_db) ? Number(gain.actual_db).toFixed(1) : Number(gain.configured_db ?? 20).toFixed(1)} / 范围 ${gainMin.toFixed(0)}…${gainMax.toFixed(0)} dB`
    : "";
  const signalState = ["warning", "clipped"].includes(iq.signal_state) ? iq.signal_state : "";
  const signalLabel = iq.signal_state === "clipped"
    ? "ERROR · ADC 削顶"
    : iq.signal_state === "warning"
      ? "WARN · 接近削顶"
      : iq.sample_count ? "正常" : "待检测";
  return `
    <article class="q-card ${role} ${signalState}">
      <div class="q-label">${label} · ADC 原始 RMS · ${signalLabel}</div>
      <div class="q-value">${rms}</div>
      <div class="q-sub">滤波后 RMS ${filteredRms}</div>
      <div class="q-sub">余量 ${headroom} · clipping ${clip}</div>
      <div class="q-sub">当前观察点峰值 ${peak} @ ${peakOff}</div>
      ${gainText ? `<div class="q-sub">${gainText}</div>` : ""}
      <div class="q-sub">累计裁判帧 ${frames}</div>
    </article>`;
}

function renderQuality(data) {
  $("qualityGrid").innerHTML = qualityCard("rx1", data) + qualityCard("rx2", data);
}

/* ---------- Decoded business-field rendering per cmd ---------- */
function cell(label, value) {
  return `<div class="decode-cell"><div class="dc-label">${escapeHtml(label)}</div><div class="dc-value">${escapeHtml(value)}</div></div>`;
}

function robotGrid(map, fmt = (v) => v) {
  if (!map || typeof map !== "object") return "";
  return `<div class="decode-grid">${Object.entries(map)
    .map(([key, value]) => cell(robotLabels[key] || key, fmt(value)))
    .join("")}</div>`;
}

// 把单帧 parsed 渲染成对应命令的业务区块（不含原始 hex，交给外层统一折叠）。
function sectionForFrame(cmd, entry) {
  const frame = entry.frame || {};
  const parsed = frame.parsed || {};
  const age = fmtAge(entry.timestamp);
  const head = (title) => `<h3>${title} <span class="sec-age">· ${age}</span></h3>`;

  // 0x0A06 密钥（干扰波）
  const password = get(parsed, ["radar_command", "password"], get(parsed, ["radar_decision_sync", "password"], parsed.password));
  if (password) {
    return `<div class="decode-section"><div class="password-hero"><span class="ph-label">0x0A06 密钥 · ${age}</span><span class="ph-value">${escapeHtml(password)}</span></div></div>`;
  }
  const positions = parsed.positions_cm || parsed.official_positions_cm;
  if (positions) {
    return `<div class="decode-section">${head("坐标 (cm)")}${robotGrid(positions, (p) => `X ${p?.x ?? "—"} / Y ${p?.y ?? "—"}`)}</div>`;
  }
  const hp = parsed.hp || parsed.official_hp;
  if (hp) return `<div class="decode-section">${head("血量")}${robotGrid(hp)}</div>`;
  const bullets = parsed.bullet_allowance || parsed.official_bullet_allowance;
  if (bullets) return `<div class="decode-section">${head("发弹余量")}${robotGrid(bullets)}</div>`;
  if (parsed.remaining_coins != null || parsed.total_coins != null) {
    return `<div class="decode-section">${head("经济")}<div class="decode-grid">${
      cell("剩余金币", parsed.remaining_coins ?? "—") +
      cell("总金币", parsed.total_coins ?? "—") +
      cell("占领位图", parsed.occupation_bits != null ? `0x${Number(parsed.occupation_bits).toString(16)}` : "—")
    }</div></div>`;
  }
  const buffs = parsed.buff_status;
  if (buffs && typeof buffs === "object") {
    const rows = Object.entries(buffs)
      .map(([key, b]) => cell(robotLabels[key] || key, `回血${b.hp_recovery_percent ?? 0}% 攻${b.attack_percent ?? 0}% 防${b.defense_percent ?? 0}%`))
      .join("");
    const sentryName = sentryModeLabels[parsed.sentry_mode_name] || parsed.sentry_mode_name || "未知";
    const sentry = parsed.sentry_mode != null ? cell("哨兵姿态", `${parsed.sentry_mode} · ${sentryName}`) : "";
    const mainStatuses = parsed.robot_main_status || {};
    const mainStatusNames = parsed.robot_main_status_names || {};
    const statusRows = Object.entries(mainStatuses)
      .map(([key, value]) => {
        const statusName = mainStatusLabels[mainStatusNames[key]] || mainStatusNames[key] || "未知";
        return cell(`${robotLabels[key] || key}主要状态`, `${value} · ${statusName}`);
      })
      .join("");
    return `<div class="decode-section">${head("增益与主要状态")}<div class="decode-grid">${rows}${sentry}${statusRows}</div></div>`;
  }
  // 未识别命令：显示命令号 + 原始数据
  return `<div class="decode-section">${head(cmd)}<div class="decode-grid">${cell("数据", frame.data_hex || "—")}</div></div>`;
}

// 渲染一路的所有命令帧（每命令最近一帧），按命令号排序，稳定同时展示。
function renderDecodedRole(container, byCmd, emptyText) {
  const cmds = Object.keys(byCmd).sort();
  if (!cmds.length) {
    container.innerHTML = `<p class="empty">${escapeHtml(emptyText)}</p>`;
    return;
  }
  // 密钥优先置顶
  cmds.sort((a, b) => (b === "0x0A06" ? 1 : 0) - (a === "0x0A06" ? 1 : 0));
  const sections = cmds.map((cmd) => sectionForFrame(cmd, byCmd[cmd]));
  const lastEntry = cmds.map((c) => byCmd[c]).sort((a, b) => b.timestamp - a.timestamp)[0];
  const rawHex = lastEntry?.frame?.raw_hex || lastEntry?.frame?.data_hex;
  const rawBlock = rawHex
    ? `<details class="raw-frame"><summary>最近原始帧字节</summary><code>${escapeHtml(rawHex)}</code></details>`
    : "";
  container.innerHTML = sections.join("") + rawBlock;
}

// 默认只保留五类轮播帧的最近到达时刻；完整业务字段放在折叠区内按需查看。
function renderBroadcastArrivals(container, byCmd) {
  const now = Date.now() / 1000;
  container.innerHTML = broadcastFrames.map(([cmd, label]) => {
    const timestamp = Number(byCmd[cmd]?.timestamp || 0);
    const age = timestamp ? Math.max(0, now - timestamp) : Infinity;
    const freshness = !timestamp ? "missing" : age < 3 ? "fresh" : "stale";
    const history = frameArrivalHistory.rx1[cmd] || [];
    while (history.length && history[0] < now - FRAME_RATE_WINDOW_SEC) history.shift();
    const frameRate = history.length / FRAME_RATE_WINDOW_SEC;
    return `<div class="arrival-cell ${freshness}">
      <span class="arrival-label">${cmd} · ${escapeHtml(label)}</span>
      <div class="arrival-metrics">
        <span><em>最近到达</em><strong>${fmtRecentArrival(timestamp, now)}</strong></span>
        <span><em>1s 平均帧率</em><strong>${frameRate.toFixed(1)} 帧/s</strong></span>
      </div>
    </div>`;
  }).join("");
}

function recordFrameArrival(role, cmd, timestamp) {
  if (!Number.isFinite(timestamp) || timestamp <= 0) return;
  const history = frameArrivalHistory[role][cmd] || (frameArrivalHistory[role][cmd] = []);
  // SSE/HTTP 重连可能重送最近帧；同一命令只记录严格更新的到达时刻。
  if (!history.length || timestamp > history[history.length - 1]) history.push(timestamp);
  const cutoff = Date.now() / 1000 - FRAME_RATE_WINDOW_SEC;
  while (history.length && history[0] < cutoff) history.shift();
}

function frameRateForRole(role, now = Date.now() / 1000) {
  const cutoff = now - FRAME_RATE_WINDOW_SEC;
  let count = 0;
  for (const history of Object.values(frameArrivalHistory[role] || {})) {
    while (history.length && history[0] < cutoff) history.shift();
    count += history.length;
  }
  return count / FRAME_RATE_WINDOW_SEC;
}

// 把一批帧并入"每命令最近一帧"累加器；返回各路是否有新帧。
function ingestFrames(frames) {
  const touched = { rx1: false, rx2: false };
  // 服务端返回 newest-first；按时间正序记录，保证 1 秒滑动窗口不漏掉同类帧。
  const orderedFrames = [...frames].sort((a, b) => Number(a.timestamp) - Number(b.timestamp));
  for (const item of orderedFrames) {
    const role = item.role;
    if (role !== "rx1" && role !== "rx2") continue;
    const frame = item.frame || {};
    const cmd = frame.cmd_hex || (frame.cmd_id != null ? `0x${Number(frame.cmd_id).toString(16).padStart(4, "0").toUpperCase()}` : "?");
    const timestamp = Number(item.timestamp);
    const prev = latestByCmd[role][cmd];
    const isNewArrival = !prev || timestamp > Number(prev.timestamp);
    if (!prev || timestamp >= Number(prev.timestamp)) {
      latestByCmd[role][cmd] = { frame, timestamp };
    }
    if (isNewArrival) recordFrameArrival(role, cmd, timestamp);
    if (timestamp > lastFrameSeen[role]) { lastFrameSeen[role] = timestamp; touched[role] = true; }
  }
  return touched;
}

function renderReception(healthModules, updateDecoded = false) {
  const health = healthModules || {};
  for (const role of ["rx1", "rx2"]) {
    const byCmd = latestByCmd[role];
    const cmds = role === "rx1"
      ? broadcastFrames.map(([cmd]) => cmd).filter((cmd) => byCmd[cmd])
      : Object.keys(byCmd);
    const newest = cmds.length ? Math.max(...cmds.map((c) => Number(byCmd[c].timestamp))) : 0;
    const modHealth = stateClass(get(health, [role, "state"], "unknown"));
    const businessHealth = stateClass(get(health, [`${role}_business_frames`, "state"], "unknown"));
    const receptionState = modHealth === "bad" || businessHealth === "bad"
      ? "bad"
      : modHealth === "warn" || businessHealth === "warn"
        ? "warn"
        : modHealth === "unknown" || businessHealth === "unknown" ? "unknown" : "ok";
    const receptionMessage = get(health, [`${role}_business_frames`, "message"], get(health, [role, "message"], "等待检测"));

    if (role === "rx1") {
      $(`${role}Cmd`).textContent = cmds.length ? `已到达 ${cmds.length} / ${broadcastFrames.length} 类帧` : "等待帧…";
      renderBroadcastArrivals($("rx1Arrivals"), byCmd);
      if (updateDecoded) renderDecodedRole($("rx1Decoded"), byCmd, "尚未解析到信息波裁判帧。");
    } else {
      const now = Date.now() / 1000;
      $(`${role}Age`).textContent = newest ? fmtRecentArrival(newest, now) : "—";
      $("rx2FrameRate").textContent = `${frameRateForRole("rx2", now).toFixed(1)} 帧/s`;
      // 干扰波仍显示当前活跃命令号与完整密钥内容。
      $(`${role}Cmd`).textContent = cmds.length ? cmds.sort().join(" · ") : "等待帧…";
    }

    const badge = $(`${role}Status`);
    const fresh = newest && (Date.now() / 1000 - newest) < 3;
    badge.title = receptionMessage;
    if (!cmds.length) {
      badge.className = `recv-badge ${receptionState}`;
      badge.textContent = receptionState === "bad" ? "关键帧缺失" : receptionState === "warn" ? "等待业务帧" : "待检测";
    } else if (!fresh) {
      badge.className = `recv-badge ${receptionState === "bad" ? "bad" : "warn"}`;
      badge.textContent = receptionState === "bad" ? "业务帧中断" : "接收已停止";
    } else {
      badge.className = `recv-badge ${receptionState}`;
      badge.textContent = receptionState === "bad" ? "接收有错误" : receptionState === "warn" ? "接收有告警" : receptionState === "unknown" ? "待检测" : "接收中";
    }

    if (role === "rx2" && updateDecoded) {
      renderDecodedRole($("rx2Decoded"), byCmd, "尚未解析到干扰波密钥帧。");
    }
  }
}

function renderLossDiagnostics(data) {
  const pipeline = get(data, ["statuses", "rx1", "data", "reception_pipeline"], null);
  const badge = $("lossBadge");
  const container = $("lossDiagnostics");
  if (!pipeline) {
    badge.className = "badge unknown";
    badge.textContent = "样本不足";
    container.innerHTML = `<p class="empty">等待信息波帧诊断数据。</p>`;
    return;
  }
  const pct = (value) => value == null ? "—" : `${(Number(value) * 100).toFixed(1)}%`;
  const available = pipeline.estimate_available === true;
  const lossRate = pipeline.end_to_end_loss_rate;
  const badgeState = !available ? "unknown" : Number(lossRate || 0) > 0.2 ? "bad" : Number(lossRate || 0) > 0.05 ? "warn" : "ok";
  badge.className = `badge ${badgeState}`;
  badge.textContent = available ? `总丢包 ${pct(lossRate)}` : "样本不足";
  const counts = pipeline.counts || pipeline.debug || {};
  const preCrcRate = pipeline.pre_crc_loss_rate ?? pipeline.pre_crc_missing_rate;
  const rejectedFrames = pipeline.crc_rejected_frames ?? pipeline.rejected_frames ?? 0;
  const rejectionRate = pipeline.crc_rejection_rate ?? pipeline.rejection_rate;
  container.innerHTML = `
    <div class="loss-cell"><span>估计原始发送</span><strong>${pipeline.estimated_original_frames ?? 0}</strong></div>
    <div class="loss-cell"><span>本机发现候选</span><strong>${pipeline.suspected_frames ?? 0}</strong></div>
    <div class="loss-cell"><span>CRC 有效帧</span><strong>${pipeline.crc_valid_frames ?? 0}</strong></div>
    <div class="loss-cell"><span>前级估计丢失</span><strong>${pipeline.pre_crc_missing_frames ?? 0} · ${pct(preCrcRate)}</strong></div>
    <div class="loss-cell"><span>本机拒绝</span><strong>${rejectedFrames} · ${pct(rejectionRate)}</strong></div>
    <div class="loss-cell"><span>端到端总丢包</span><strong>${available ? pct(lossRate) : "—"}</strong></div>
    <div class="loss-debug">仅统计，不修复坏帧。CRC8 ${counts.crc8_failures ?? 0} · 长度 ${counts.length_failures ?? 0} · CRC16 ${counts.crc16_failures ?? 0} · 命令长度 ${counts.command_length_failures ?? 0}</div>`;
}

/* ---------- Referee ---------- */
function renderReferee(data) {
  const status = get(data, ["statuses", "referee", "data"], {});
  const health = get(data, ["health", "modules", "referee"], {});
  $("serialBadge").className = `badge ${stateClass(health.state)}`;
  $("serialBadge").textContent = stateLabel(health.state);
  const radar = status.last_radar_decision_sync || {};
  const robot = status.last_robot_status || {};
  const game = status.last_game_status || {};
  const lastTx = status.last_tx || {};
  $("refereeState").innerHTML = `
    <span>串口</span><strong>${status.serial_open ? "打开" : "关闭"}</strong>
    <span>端口</span><strong>${escapeHtml(status.port || get(data, ["referee", "port"], "-"))}</strong>
    <span>机器人</span><strong>${escapeHtml(robot.robot_id ?? "-")} / ${escapeHtml(robot.radio_side || "-")}</strong>
    <span>加密等级</span><strong>${escapeHtml(radar.own_encryption_level ?? "-")}</strong>
    <span>可改密钥</span><strong>${radar.can_change_password === true ? "是" : radar.can_change_password === false ? "否" : "-"}</strong>
    <span>比赛阶段</span><strong>${escapeHtml(game.game_progress ?? "-")}</strong>
    <span>最近发送</span><strong>${escapeHtml(lastTx.data_cmd_hex || lastTx.cmd_hex || "-")}</strong>
    <span>错误</span><strong>${escapeHtml(status.last_error || health.message || "无")}</strong>`;
}

/* ---------- Config drawer ---------- */
function applyModeUI(intelligent) {
  $("intelligentToggle").checked = intelligent;
  // 智能模式：红蓝方/等级只读（显示裁判系统当前值）；手动模式可编辑热切换。
  $("radioSide").disabled = intelligent;
  $("interferenceLevel").disabled = intelligent;
  $("radioSide").classList.toggle("readonly", intelligent);
  $("interferenceLevel").classList.toggle("readonly", intelligent);
}

function fillConfig(data) {
  const intelligent = data.intelligent_mode !== false;
  const configured = data.configured || {
    radio_side: data.radio_side,
    interference_level: data.interference_level,
  };
  const effective = data.effective || configured;
  const displayed = intelligent ? effective : configured;
  applyModeUI(intelligent);
  // 智能模式显示裁判当前值；人工基线仍保存在 configured 中。
  if (intelligent || document.activeElement !== $("radioSide")) {
    $("radioSide").value = displayed.radio_side || "red";
  }
  if (intelligent || document.activeElement !== $("interferenceLevel")) {
    $("interferenceLevel").value = String(displayed.interference_level || 1);
  }
  $("rx1Uri").value = get(data, ["sdr_uris", "rx1"], "");
  $("rx2Uri").value = get(data, ["sdr_uris", "rx2"], "");
  $("refereePort").value = get(data, ["referee", "port"], "");
  $("refereeBaudrate").value = get(data, ["referee", "baudrate"], 115200);
  const gain = data.rx1_gain || {};
  $("rx1Gain").min = String(gain.min_db ?? -1);
  $("rx1Gain").max = String(gain.max_db ?? 73);
  if (document.activeElement !== $("rx1Gain")) {
    $("rx1Gain").value = String(gain.configured_db ?? 20);
  }
  $("rx1GainValue").textContent = `${Number($("rx1Gain").value).toFixed(0)} dB`;
  const plan = data.frequency_plan || {};
  const interference = plan.interference || {};
  const recorder = get(data, ["statuses", "recorder", "data"], {});
  const recorderState = {
    armed: "待比赛开始",
    recording: "录制中",
    post_roll: "收尾录制",
    finalizing: "正在保存",
    disabled: "已关闭",
    error: "异常",
  }[recorder.state] || "等待状态";
  $("frequencyPlan").innerHTML = `
    <span>信息波频点</span><strong>${fmtHz(plan.broadcast_hz)}</strong>
    <span>干扰波频点</span><strong>${fmtHz(interference.center_f)}</strong>
    <span>干扰带宽</span><strong>${fmtHz(interference.BW_ganrao)}</strong>
    <span>比赛内录</span><strong>${escapeHtml(recorderState)}</strong>
    <span>事件/清单目录</span><strong>${escapeHtml(recorder.session_dir || get(data, ["runtime", "recording_root"], "-"))}</strong>
    <span>大尺寸 IQ 目录</span><strong>${escapeHtml(recorder.iq_session_dir || recorder.iq_record_root || "-")}</strong>
    <span>日志根目录</span><strong>${escapeHtml(get(data, ["runtime", "log_root"], "-"))}</strong>`;
}

function nodeRuntimeLabel(role, status, payload, health) {
  const age = Number(health.last_seen_age_sec);
  const fresh = Boolean(status.timestamp) && (!Number.isFinite(age) || age <= 10);
  if (role === "vision_radar") {
    return payload.schema === "shark.radar.telemetry.v1" && fresh ? "运行中" : "未运行";
  }
  if (role === "radar_integration") {
    return payload.schema === "shark.radar.fusion.v1" && fresh ? "运行中" : "未运行";
  }
  return payload.started ? "运行中" : "未运行";
}

function nodeCard(role, data) {
  const status = get(data, ["statuses", role], {});
  const payload = status.data || {};
  const stats = get(payload, ["decoder", "stats"], {});
  const health = get(data, ["health", "modules", role], {});
  return `
    <article class="mini-card">
      <div class="card-head">
        <strong>${roles[role]}</strong>
        <span class="pill ${stateClass(health.state)}">${stateLabel(health.state)}</span>
      </div>
      <div class="kv">
        <span>启动</span><strong>${nodeRuntimeLabel(role, status, payload, health)}</strong>
        <span>Profile</span><strong>${escapeHtml(payload.rx_profile || "-")}</strong>
        <span>Frames</span><strong>${stats.frames ?? "-"}</strong>
        <span>错误</span><strong>${escapeHtml(payload.last_error || health.message || "-")}</strong>
        <span>更新</span><strong>${fmtAge(status.timestamp)}</strong>
      </div>
    </article>`;
}

function renderNodes(data) {
  $("nodeGrid").innerHTML = ["rx1", "rx2", "vision_radar", "radar_integration"]
    .map((role) => nodeCard(role, data)).join("");
}

function renderProcesses(data) {
  const processes = data.processes || {};
  $("processGrid").innerHTML = ["rx", "referee"].map((name) => {
    const proc = processes[name] || {};
    const running = proc.running === true;
    const externallyManaged = proc.externally_managed === true;
    const controlHint = externallyManaged ? "由一键启动器托管，请在启动终端统一启停" : "";
    return `
      <article class="mini-card">
        <div class="card-head">
          <strong>${name === "rx" ? "双 RX" : "裁判串口"}</strong>
          <span class="pill ${running ? "ok" : proc.returncode ? "bad" : "unknown"}">${running ? "running" : "stopped"}</span>
        </div>
        <div class="kv">
          <span>PID</span><strong>${proc.pid || "-"}</strong>
          <span>托管</span><strong>${externallyManaged ? "一键启动器" : "Dashboard"}</strong>
          <span>退出码</span><strong>${proc.returncode ?? "-"}</strong>
          <span>日志</span><strong>${escapeHtml(proc.log_path || "-")}</strong>
        </div>
        <div class="actions">
          <button type="button" data-start="${name}" ${externallyManaged ? "disabled" : ""} title="${controlHint}">启动</button>
          <button type="button" data-stop="${name}" class="danger" ${externallyManaged ? "disabled" : ""} title="${controlHint}">停止</button>
        </div>
      </article>`;
  }).join("");
}

function renderEvents(data) {
  const events = (data.events || []).slice(0, 12);
  const processLogs = Object.values(data.processes || {}).map((proc) => proc.log_path).filter(Boolean);
  const logHtml = processLogs.length
    ? `<div class="log-paths">${processLogs.map((item) => `<code>${escapeHtml(item)}</code>`).join("")}</div>`
    : "";
  $("eventList").innerHTML = logHtml + (events.length
    ? events.map((event) => `
      <article class="event-row">
        <strong>${escapeHtml(event.kind || "event")}</strong>
        <span>${fmtAge(event.timestamp)}</span>
        <p>${escapeHtml(event.message || "")}</p>
      </article>`).join("")
    : `<p class="empty">暂无故障事件</p>`);
}

/* ---------- Main render (slow: full state @1Hz) ---------- */
function render(data) {
  state.latest = data;
  const overall = get(data, ["health", "state"], "unknown");
  $("overallBadge").className = `badge ${stateClass(overall)}`;
  $("overallBadge").textContent = `总体 ${stateLabel(overall)}`;
  const effective = data.effective || {
    radio_side: data.radio_side,
    interference_level: data.interference_level,
  };
  $("sideBadge").textContent = `${effective.radio_side || "-"} · L${effective.interference_level || "-"}`;
  $("summary").textContent = `${roles.rx1} / ${roles.rx2} / 裁判 · 模式 ${data.mode || "match_rx"}`;
  renderQuality(data);
  renderLossDiagnostics(data);
  renderReception(get(data, ["health", "modules"], {}));
  renderReferee(data);
  if (activePage === "radar") {
    radarLatest = { radar: data.radar, vision_radar: data.vision_radar };
    renderRadarData(radarLatest);
    radarDirty = true;
  }
  fillConfig(data);
  if (!$("drawer").hidden) {
    renderNodes(data);
    renderProcesses(data);
    renderEvents(data);
  }
}

/* ---------- Data loop: poll spectrum+frames, update targets only (no draw) ---------- */
function updateSpectrumTargets(payload) {
  const spectra = (payload && payload.spectra) || {};
  let changed = false;
  for (const role of ["rx1", "rx2"]) {
    const entry = spectra[role] || {};
    const spec = entry.data || {};
    const powers = Array.isArray(spec.power_dbfs) ? spec.power_dbfs : null;
    if (!powers || powers.length < 2) continue;
    const timestamp = Number(entry.timestamp || spec.timestamp || 0);
    if (spectrumTarget[role] && timestamp <= Number(spectrumTarget[role].timestamp || 0)) continue;
    const center = Number(spec.center_frequency_hz);
    const offs = Array.isArray(spec.offset_hz) && spec.offset_hz.length === powers.length ? spec.offset_hz : null;
    const freqs = powers.map((_, i) => {
      const off = offs ? Number(offs[i]) : 0;
      return Number.isFinite(center) ? center + off : off;
    });
    const target = {
      freqs,
      power: powers.map((p) => (Number.isFinite(Number(p)) ? Number(p) : -240)),
      timestamp,
      sourceTimestamp: Number(spec.timestamp || timestamp),
      meta: {
        peak_frequency_hz: spec.peak_frequency_hz,
        peak_dbfs: spec.peak_dbfs,
        peak_offset_hz: spec.peak_offset_hz,
        noise_floor_dbfs: spec.noise_floor_dbfs,
      },
    };
    spectrumTarget[role] = target;
    enqueueWaterfall(role, target);
    displayPerf.updates[role] += 1;
    const nowSeconds = Date.now() / 1000;
    if (target.sourceTimestamp > 0) displayPerf.latencySamples.push(Math.max(0, (nowSeconds - target.sourceTimestamp) * 1000));
    if (timestamp > 0) displayPerf.transportSamples.push(Math.max(0, (nowSeconds - timestamp) * 1000));
    if (displayPerf.latencySamples.length > 200) displayPerf.latencySamples.splice(0, displayPerf.latencySamples.length - 200);
    if (displayPerf.transportSamples.length > 200) displayPerf.transportSamples.splice(0, displayPerf.transportSamples.length - 200);
    changed = true;
    // Keep the slow status snapshot's signal-quality cards current without
    // forcing a full /api/state request for every FFT update.
    if (state.latest) {
      state.latest.spectra = state.latest.spectra || {};
      state.latest.spectra[role] = entry;
    }
  }
  if (changed) spectrumDirty = true;
  return changed;
}

function ingestStreamPayload(payload) {
  const serverSequence = Number(payload.frame_sequence || 0);
  if (serverSequence < lastFrameSequence) {
    lastFrameSequence = 0;
    frameArrivalHistory.rx1 = {};
    frameArrivalHistory.rx2 = {};
  }
  updateSpectrumTargets(payload);
  if (ingestFrames(payload.frames || [])) framesDirty = true;
  if (serverSequence >= lastFrameSequence) lastFrameSequence = serverSequence;
}

async function dataTick() {
  if (streamRequestBusy || document.hidden) return;
  streamRequestBusy = true;
  try {
    const query = new URLSearchParams({
      after_seq: String(lastFrameSequence),
      rx1_after: String(Number(spectrumTarget.rx1?.timestamp || 0)),
      rx2_after: String(Number(spectrumTarget.rx2?.timestamp || 0)),
      tap: $("spectrumTap").value,
    });
    const payload = await apiGet(`/api/spectrum?${query.toString()}`);
    ingestStreamPayload(payload);
  } catch (_error) {
    // 静默：数据循环失败不打扰主状态行
  } finally {
    streamRequestBusy = false;
  }
}

function stopSpectrumStream() {
  if (spectrumReconnectTimer) {
    clearTimeout(spectrumReconnectTimer);
    spectrumReconnectTimer = 0;
  }
  if (spectrumEventSource) {
    spectrumEventSource.close();
    spectrumEventSource = null;
  }
  spectrumStreamState = "closed";
}

function startSpectrumStream() {
  if (document.hidden || typeof EventSource === "undefined") return;
  stopSpectrumStream();
  const query = new URLSearchParams({
    after_seq: String(lastFrameSequence),
    rx1_after: String(Number(spectrumTarget.rx1?.timestamp || 0)),
    rx2_after: String(Number(spectrumTarget.rx2?.timestamp || 0)),
    tap: $("spectrumTap").value,
  });
  spectrumStreamState = "connecting";
  const source = new EventSource(`/api/spectrum-stream?${query.toString()}`);
  spectrumEventSource = source;
  source.onopen = () => {
    spectrumStreamState = "open";
    $("waterfallStatus").textContent = "SSE 实时推送";
  };
  source.onmessage = (event) => {
    try {
      ingestStreamPayload(JSON.parse(event.data));
    } catch (_error) {
      displayPerf.droppedWaterfallRows += 1;
    }
  };
  source.onerror = () => {
    if (spectrumEventSource !== source) return;
    source.close();
    spectrumEventSource = null;
    spectrumStreamState = "reconnecting";
    $("waterfallStatus").textContent = "SSE 重连中（HTTP 兜底）";
    spectrumReconnectTimer = window.setTimeout(startSpectrumStream, 1000);
  };
}

function mean(values) {
  if (!values.length) return 0;
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

async function updateDisplayPerformance() {
  const now = performance.now();
  const elapsed = Math.max(0.001, (now - displayPerf.intervalStartMs) / 1000);
  const metrics = {
    render_fps: displayPerf.renderFrames / elapsed,
    rx1_hz: displayPerf.updates.rx1 / elapsed,
    rx2_hz: displayPerf.updates.rx2 / elapsed,
    latency_ms: mean(displayPerf.latencySamples),
    transport_ms: mean(displayPerf.transportSamples),
    dropped_waterfall_rows: displayPerf.droppedWaterfallRows,
    stream_state: activePage === "radar" ? radarStreamState : spectrumStreamState,
    active_page: activePage,
  };
  displayPerf.latest = metrics;
  displayPerf.intervalStartMs = now;
  displayPerf.renderFrames = 0;
  displayPerf.updates.rx1 = 0;
  displayPerf.updates.rx2 = 0;
  displayPerf.latencySamples = [];
  displayPerf.transportSamples = [];

  if (activePage === "radio") {
    const badge = $("spectrumPerf");
    badge.textContent = `RX1 ${metrics.rx1_hz.toFixed(1)}Hz · RX2 ${metrics.rx2_hz.toFixed(1)}Hz · ${metrics.render_fps.toFixed(0)}FPS · ${metrics.latency_ms.toFixed(0)}ms`;
    const activeRates = [metrics.rx1_hz, metrics.rx2_hz].filter((value) => value > 0.1);
    const minRate = activeRates.length ? Math.min(...activeRates) : 0;
    badge.className = `perf-badge ${metrics.render_fps >= 45 && minRate >= 12 ? "ok" : minRate >= 4 ? "warn" : "bad"}`;
  }
  try {
    await apiPost("/api/client-metrics", metrics);
  } catch (_error) {
    // Performance telemetry must never interrupt spectrum rendering.
  }
}

/* ---------- Radar map: latest-only SSE, point/table linkage ---------- */
const radarRoleIds = ["1", "2", "3", "4", "6", "7"];

function normalizeRadarSide(value) {
  return String(value || "").toLowerCase().startsWith("b") ? "blue" : "red";
}

function radarRobotOrderForSide(side) {
  const allyPrefix = normalizeRadarSide(side) === "red" ? "R" : "B";
  const opponentPrefix = allyPrefix === "R" ? "B" : "R";
  return [opponentPrefix, allyPrefix].flatMap((prefix) => radarRoleIds.map((role) => `${prefix}${role}`));
}

function resolveRadarOwnSide(fusion = {}, serial = {}) {
  return normalizeRadarSide(
    serial.effective_radio_side || fusion.detected_side || fusion.side ||
    get(state.latest, ["effective", "radio_side"], "") || $("radioSide")?.value
  );
}

function updateRadarMapOrientation(side) {
  const cleanSide = normalizeRadarSide(side);
  radarMapOwnSide = cleanSide;
  const label = cleanSide === "red" ? "红方" : "蓝方";
  const image = $("radarMapImage");
  const expectedSource = cleanSide === "red" ? "/image_2026_red_bottom.jpg" : "/image_2026_blue_bottom.jpg";
  if (image && image.getAttribute("src") !== expectedSource) image.setAttribute("src", expectedSource);
  if (image) image.alt = `RoboMaster 2026 赛场地图，${label}在下`;
  if ($("radarMapOrientation")) $("radarMapOrientation").textContent = "赛场坐标";
  fitRadarMapStage();
  return cleanSide;
}

function radarCanvasPoint(position) {
  if (!position) return null;
  return radarMapOwnSide === "red"
    ? { x: 1500 - position.y, y: 2800 - position.x }
    : { x: position.y, y: position.x };
}

function radarPosition(entry) {
  const position = entry && entry.position_cm;
  const x = Number(position && position.x);
  const y = Number(position && position.y);
  return Number.isFinite(x) && Number.isFinite(y) && (x !== 0 || y !== 0) ? { x, y } : null;
}

function coordText(position) {
  return position ? `(${position.x.toFixed(0)}, ${position.y.toFixed(0)})` : "—";
}

function ageText(age) {
  const value = Number(age);
  return Number.isFinite(value) ? `${value.toFixed(value < 1 ? 2 : 1)}s` : "—";
}

function strategyPayload(strategy, name) {
  const entry = strategy && strategy[name];
  return (entry && entry.payload) || {};
}

const gameProgressLabels = {
  0: "未开始", 1: "准备阶段", 2: "15 秒自检", 3: "5 秒倒计时", 4: "比赛进行中", 5: "比赛结算",
};

const dartSelectedTargetLabels = {
  0: "未选定 / 前哨站", 1: "基地固定目标", 2: "基地随机固定目标",
  3: "基地随机移动目标", 4: "基地末端移动目标",
};

const dartHitTargetLabels = {
  0: "尚未命中", 1: "前哨站", 2: "基地固定目标", 3: "基地随机固定目标",
  4: "基地随机移动目标", 5: "基地末端移动目标",
};

function enumLabel(labels, value) {
  if (value == null) return "—";
  return labels[Number(value)] || `未知值 ${value}`;
}

function renderRadarHealth(payload, fusion, vision, radio) {
  const healthSnapshot = payload?.health || get(state.latest, ["health"], {});
  const modules = healthSnapshot.modules || {};
  const issues = Object.entries(modules).map(([name, item]) => ({
    name: roles[name] || name,
    state: item?.state || "unknown",
    message: item?.message || "未收到状态说明",
    age: item?.last_seen_age_sec,
  }));

  if (fusion.schema === "shark.radar.fusion.v1") {
    if (fusion.side_mismatch) {
      issues.push({ name: "阵营一致性", state: "bad", message: "配置阵营与裁判识别结果不一致", age: null });
    }
    if (radio.fresh === false) {
      issues.push({
        name: "无线电坐标源",
        state: "warn",
        message: radio.expired ? "0x0A01 坐标年龄超过 1.0 秒，已停止采用，仅保留过期影子" : "尚未收到有效 0x0A01 坐标",
        age: radio.age_sec,
      });
    }
    if (fusion.vision && fusion.vision.online === false) {
      issues.push({ name: "视觉坐标源", state: "warn", message: "视觉遥测已超时，当前不参与坐标发送", age: fusion.vision.age_sec });
    }
  }

  const counts = { ok: 0, warn: 0, bad: 0, unknown: 0 };
  for (const issue of issues) {
    const key = Object.prototype.hasOwnProperty.call(counts, issue.state) ? issue.state : "unknown";
    counts[key] += 1;
  }
  $("radarHealthCounts").innerHTML = `
    <div class="health-count error"><span>ERROR</span><strong>${counts.bad}</strong></div>
    <div class="health-count warn"><span>WARN</span><strong>${counts.warn}</strong></div>
    <div class="health-count"><span>正常</span><strong>${counts.ok}</strong></div>
    <div class="health-count unknown"><span>待检测</span><strong>${counts.unknown}</strong></div>`;

  const rates = payload?.reception_rates || get(state.latest, ["reception_rates"], {});
  const cameraFpsValue = vision.camera_fps;
  const cameraFpsText = cameraFpsValue === null || cameraFpsValue === undefined
    ? "--"
    : Number(cameraFpsValue).toFixed(1);
  const processingFps = Number(vision.processing_fps ?? vision.fps ?? 0);
  $("radarPerformanceGrid").innerHTML = `
    <div class="radar-title-rate" title="RX1 信息波 1 秒平均帧率"><span>RX1</span><strong>${Number(rates.rx1_hz || 0).toFixed(1)} Hz</strong></div>
    <div class="radar-title-rate" title="RX2 干扰波 1 秒平均帧率"><span>RX2</span><strong>${Number(rates.rx2_hz || 0).toFixed(1)} Hz</strong></div>
    <div class="radar-title-rate" title="相机实际交付的新帧率"><span>相机采集</span><strong>${cameraFpsText} FPS</strong></div>
    <div class="radar-title-rate" title="视觉算法处理循环帧率"><span>视觉处理</span><strong>${processingFps.toFixed(1)} FPS</strong></div>`;

  const rank = { bad: 0, warn: 1, unknown: 2 };
  const visibleIssues = issues
    .filter((issue) => issue.state !== "ok")
    .sort((a, b) => (rank[a.state] ?? 3) - (rank[b.state] ?? 3));
  $("radarIssueList").innerHTML = visibleIssues.length ? visibleIssues.map((issue) => {
    const kind = issue.state === "bad" ? "error" : issue.state === "warn" ? "warn" : "unknown";
    const label = stateLabel(issue.state);
    const updated = Number.isFinite(Number(issue.age)) ? `状态年龄 ${ageText(issue.age)}` : "尚无更新时间";
    return `<article class="radar-issue-card ${kind}">
      <header><strong>${escapeHtml(issue.name)}</strong><span>${label}</span></header>
      <p>${escapeHtml(issue.message)}</p><small>${updated}</small>
    </article>`;
  }).join("") : `<div class="radar-issue-empty">暂无 ERROR / WARN，所有项目均已检测</div>`;
}

function renderRadarData(payload) {
  const integrationEntry = payload?.radar || get(state.latest, ["radar"], {});
  const visionEntry = payload?.vision_radar || get(state.latest, ["vision_radar"], {});
  const fusion = integrationEntry?.data || {};
  const visionTelemetry = visionEntry?.data || {};
  const vision = fusion.vision || visionTelemetry.vision || {};
  const radio = fusion.radio || {};
  const tx = fusion.tx_0305 || {};
  const serial = fusion.referee_serial || get(state.latest, ["statuses", "referee", "data"], {});
  const ownSide = updateRadarMapOrientation(resolveRadarOwnSide(fusion, serial));
  const radarRobotOrder = radarRobotOrderForSide(ownSide);
  const allyPrefix = ownSide === "red" ? "R" : "B";
  const healthSnapshot = payload?.health || get(state.latest, ["health"], {});
  const health = healthSnapshot.modules || {};
  const integrationHealth = health.radar_integration || {};
  const overallState = fusion.side_mismatch
    ? "bad"
    : healthSnapshot.state === "bad" || fusion.health === "bad"
      ? "bad"
      : healthSnapshot.state === "warn" || fusion.health === "warn"
        ? "warn"
        : fusion.health || integrationHealth.state || healthSnapshot.state || "unknown";
  $("radarOverallBadge").className = `badge ${stateClass(overallState)}`;
  $("radarOverallBadge").textContent = fusion.side_mismatch ? "阵营冲突" : stateLabel(overallState);
  renderRadarHealth(payload, fusion, vision, radio);

  const robots = fusion.robots || {};
  $("radarRobotRows").innerHTML = radarRobotOrder.map((name) => {
    const item = robots[name] || {};
    const visual = item.visual || {};
    const radioItem = item.radio || {};
    const finalItem = item.final || {};
    const visualPos = radarPosition(visual);
    const radioPos = radarPosition(radioItem);
    const finalPos = radarPosition(finalItem);
    const radioAge = Number(radioItem.age_sec);
    const rowClass = radioItem.expired ? "radio-expired" : Number.isFinite(radioAge) && radioAge >= 0.8 ? "radio-near" : "";
    const radioState = radioItem.expired
      ? "已过期，仅作影子"
      : radioItem.fresh ? "有效" : "missing";
    const fallback = radioItem.expired
      ? finalItem.source === "vision" ? " · 最终已回退视觉" : " · 最终 missing"
      : "";
    return `<tr data-robot="${name}" class="${rowClass}${selectedRobot === name ? " selected" : ""}">
      <td><strong>${name}</strong><span class="team-tag ${name.startsWith(allyPrefix) ? "ally" : "opponent"}">${name.startsWith(allyPrefix) ? "己方" : "对方"}</span></td>
      <td><div class="coord-line">${coordText(visualPos)}</div><div class="state-line">${escapeHtml(visual.state || "missing")} ${visual.confidence == null ? "" : Number(visual.confidence).toFixed(2)}</div></td>
      <td><div class="coord-line">${coordText(radioPos)}</div><div class="state-line">${radioState} · ${ageText(radioItem.age_sec)}${fallback}</div></td>
      <td><div class="coord-line">${coordText(finalPos)}</div><div class="state-line">${escapeHtml(finalItem.source || "missing")}</div></td>
    </tr>`;
  }).join("");

  const strategy = fusion.strategy || {};
  const marks = strategyPayload(strategy, "RadarMarkProgress").enemy || {};
  const decision = strategyPayload(strategy, "RadarDecisionSync");
  const dart = strategyPayload(strategy, "DartStatus");
  const game = strategyPayload(strategy, "GameStatus");
  const passwordMessage = strategyPayload(strategy, "RadarCommand0121");
  const lastAck = serial.last_algorithm_ack || tx.last_ack || {};
  const pending = serial.last_algorithm_request && (!lastAck.request_id || lastAck.request_id !== serial.last_algorithm_request.request_id)
    ? serial.last_algorithm_request : null;
  const verifyRemaining = Math.max(0, Number(serial.password_verify_cooldown_remaining_sec || 0));
  const verifyReady = serial.password_verify_ready !== false && verifyRemaining <= 0;
  const markNames = [
    ["英雄", "opponent_hero"], ["工程", "opponent_engineer"],
    ["步兵3", "opponent_infantry_3"], ["步兵4", "opponent_infantry_4"],
    ["空中", "opponent_aerial"], ["哨兵", "opponent_sentry"],
  ];
  const hasStrategy = [marks, decision, dart, game].some((item) => Object.keys(item).length);
  $("strategyBadge").className = `badge ${hasStrategy ? "ok" : "unknown"}`;
  $("strategyBadge").textContent = hasStrategy ? "裁判已同步" : "等待裁判";
  $("radarStrategy").innerHTML = `
    <div class="strategy-cell wide"><span>0x020C · 对方易伤门槛（裁判仅下发二态）</span><div class="mark-switch-list">${markNames.map(([label, key]) => {
      const active = marks[key] === true;
      return `<div class="mark-switch-row ${active ? "on" : "off"}"><span>${label}</span><i aria-hidden="true"><b></b></i><strong>${active ? "已达到（≥100）" : "未达到（<100）"}</strong></div>`;
    }).join("")}</div></div>
    <div class="strategy-cell"><span>0x0001 · 比赛阶段</span><strong>${enumLabel(gameProgressLabels, game.game_progress)} · 剩余 ${game.stage_remain_time ?? "—"}s</strong></div>
    <div class="strategy-cell"><span>0x020E · 双倍易伤</span><strong>剩余 ${decision.double_vulnerability_count ?? "—"} 次 · ${decision.is_double_vulnerability == null ? "状态未知" : decision.is_double_vulnerability ? "正在生效" : "未生效"}</strong></div>
    <div class="strategy-cell"><span>0x0105 · 飞镖选定目标</span><strong>${enumLabel(dartSelectedTargetLabels, dart.selected_target)} · 发射剩余 ${dart.dart_remaining_time ?? "—"}s</strong></div>
    <div class="strategy-cell"><span>0x0105 · 最近命中</span><strong>${enumLabel(dartHitTargetLabels, dart.recent_hit_target)} · 累计 ${dart.accumulated_hit_count ?? "—"} 次</strong></div>
    <div class="strategy-cell"><span>0x020E · 己方加密状态</span><strong>${decision.own_encryption_level == null ? "等级未知" : `L${decision.own_encryption_level}`} · ${decision.can_change_password == null ? "改密状态未知" : decision.can_change_password ? "允许更新密钥" : "暂不允许更新"}</strong></div>
    <div class="strategy-cell"><span>0x0121 · 当前破解密钥</span><strong>${escapeHtml(passwordMessage.password || "—")} · ${passwordMessage.password ? "等待或已提交验证" : "尚未接收"}</strong></div>
    <div class="strategy-cell"><span>0x0121 · 最近实际提交验证</span><strong>${escapeHtml(serial.last_password_verify || "—")} · ${serial.last_password_verify ? (verifyReady ? "现在可再次验证" : `${verifyRemaining.toFixed(1)}s 后可再次验证`) : "尚未提交"}</strong></div>
    <div class="strategy-cell"><span>双倍易伤请求</span><strong>${pending ? `等待 ACK · ${escapeHtml(pending.request_id || "—")}` : lastAck.request_id ? `已确认 · ${escapeHtml(lastAck.request_id)}` : "尚无请求"}</strong></div>
    <div class="strategy-cell"><span>双倍易伤串口结果</span><strong>${lastAck.written === true ? `写入成功 · radar_cmd ${lastAck.radar_cmd}` : lastAck.request_id ? `写入失败 · ${escapeHtml(lastAck.error || "未写入")}` : "—"}</strong></div>
    <div class="strategy-cell"><span>0x0305 · 坐标发送</span><strong>${Number(tx.actual_rate_hz || 0).toFixed(2)} Hz · 成功 ${tx.success_count ?? 0} / 请求 ${tx.request_count ?? 0}</strong></div>
    <div class="strategy-cell"><span>0x0305 · 当前发送条件</span><strong>${fusion.send_allowed ? "允许发送" : `已暂停 · ${escapeHtml(fusion.send_block_reason || "无有效坐标")}`}</strong></div>`;

  const serialHealth = health.referee || {};
  $("radarSerialBadge").className = `badge ${stateClass(serialHealth.state)}`;
  $("radarSerialBadge").textContent = stateLabel(serialHealth.state);
  $("radarSerialSummary").innerHTML = `
    <span>在线 / 端口</span><strong>${serial.serial_open ? "在线" : serial.dry_run ? "dry-run" : "离线"} · ${escapeHtml(serial.port || "—")}</strong>
    <span>机器人 ID / 阵营</span><strong>${serial.detected_robot_id ?? "—"} / ${escapeHtml(serial.effective_radio_side || "—")}</strong>
    <span>最近接收</span><strong>${fmtAge(get(serial, ["last_rx", "timestamp"], 0))}</strong>
    <span>最近发送</span><strong>${fmtAge(get(serial, ["last_tx", "timestamp"], 0))}</strong>
    <span>最后错误</span><strong>${escapeHtml(serial.last_error || "—")}</strong>`;
  $("radarSerialDiagnostics").innerHTML = `
    <span>收 / 发 / dry-run</span><strong>${serial.rx_count ?? 0} / ${serial.tx_count ?? 0} / ${serial.dry_run_count ?? 0}</strong>
    <span>radar_cmd</span><strong>${serial.last_radar_cmd_value ?? 0}</strong>
    <span>0x0305 限频拦截</span><strong>${serial.blocked_0305_rate_count ?? 0}</strong>
    <span>密码冷却拦截</span><strong>${serial.password_verify_suppressed_count ?? 0}</strong>
    <span>干扰等级 / 自动跟随</span><strong>L${serial.rx_interference_level ?? "—"} / ${serial.auto_rx_interference_level ? "开启" : "关闭"}</strong>
    <span>串口重连</span><strong>${Math.max(0, Number(serial.serial_open_count || 0) - 1)}</strong>`;
}

function drawRadarMap() {
  if (activePage !== "radar" || !radarLatest) return false;
  const fusion = radarLatest.radar?.data || get(state.latest, ["radar", "data"], {});
  const serial = fusion.referee_serial || get(state.latest, ["statuses", "referee", "data"], {});
  const ownSide = updateRadarMapOrientation(resolveRadarOwnSide(fusion, serial));
  const radarRobotOrder = radarRobotOrderForSide(ownSide);
  const robots = fusion.robots || {};
  const canvas = $("radarMapCanvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  radarHitTargets.length = 0;
  let moving = false;
  const colorFor = (name) => name.startsWith("R") ? "#d93d4a" : "#2c7fd1";

  for (const name of radarRobotOrder) {
    const item = robots[name] || {};
    const visual = radarPosition(item.visual);
    const radio = radarPosition(item.radio);
    const finalTarget = radarPosition(item.final);
    let final = null;
    if (finalTarget) {
      const shown = radarDisplayPositions[name] || { ...finalTarget };
      const dx = finalTarget.x - shown.x, dy = finalTarget.y - shown.y;
      if (Math.abs(dx) + Math.abs(dy) > 0.3) {
        shown.x += dx * 0.38; shown.y += dy * 0.38; moving = true;
      } else { shown.x = finalTarget.x; shown.y = finalTarget.y; }
      radarDisplayPositions[name] = shown;
      final = shown;
    } else {
      delete radarDisplayPositions[name];
    }
    const color = colorFor(name);
    if ($("showVision").checked && visual) {
      const p = radarCanvasPoint(visual);
      ctx.fillStyle = color + "38"; ctx.strokeStyle = color + "aa"; ctx.lineWidth = 5;
      ctx.fillRect(p.x - 17, p.y - 17, 34, 34); ctx.strokeRect(p.x - 17, p.y - 17, 34, 34);
      radarHitTargets.push({ name, layer: "vision", x: p.x, y: p.y });
    }
    if ($("showRadio").checked && radio && (item.radio?.fresh || item.radio?.ghost_visible)) {
      const p = radarCanvasPoint(radio);
      const expired = item.radio?.expired;
      ctx.beginPath(); ctx.moveTo(p.x, p.y - 22); ctx.lineTo(p.x + 21, p.y + 18); ctx.lineTo(p.x - 21, p.y + 18); ctx.closePath();
      ctx.fillStyle = expired ? "rgba(133,91,91,.58)" : color + "70";
      ctx.strokeStyle = expired ? "#b34b4b" : color; ctx.lineWidth = 5; ctx.fill(); ctx.stroke();
      radarHitTargets.push({ name, layer: expired ? "radio-expired" : "radio", x: p.x, y: p.y });
    }
    if ($("showFinal").checked && final) {
      const p = radarCanvasPoint(final);
      if (selectedRobot === name) { ctx.beginPath(); ctx.arc(p.x, p.y, 34, 0, Math.PI * 2); ctx.strokeStyle = "#ffd54f"; ctx.lineWidth = 8; ctx.stroke(); }
      ctx.beginPath(); ctx.arc(p.x, p.y, 21, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 5; ctx.stroke();
      radarHitTargets.unshift({ name, layer: "final", x: p.x, y: p.y });
    }
    const labelPosition = final || visual || radio;
    if ($("showLabels").checked && labelPosition) {
      const p = radarCanvasPoint(labelPosition);
      ctx.font = "bold 48px Segoe UI, Microsoft YaHei, sans-serif";
      ctx.lineWidth = 10; ctx.strokeStyle = "rgba(255,255,255,.94)"; ctx.strokeText(name, p.x + 34, p.y - 10);
      ctx.fillStyle = color; ctx.fillText(name, p.x + 34, p.y - 10);
      ctx.font = "32px Segoe UI, Microsoft YaHei, sans-serif";
      ctx.lineWidth = 7; ctx.strokeStyle = "rgba(255,255,255,.9)";
      const coordinates = `${labelPosition.x.toFixed(0)},${labelPosition.y.toFixed(0)}`;
      ctx.strokeText(coordinates, p.x + 34, p.y + 29);
      ctx.fillStyle = "#17212b"; ctx.fillText(coordinates, p.x + 34, p.y + 29);
    }
  }
  return moving;
}

const RADAR_MAP_ASPECT = 1500 / 2800;
function fitRadarMapStage() {
  const viewport = $("radarMapViewport");
  const stage = $("radarMapStage");
  if (!viewport || !stage) return;
  const availableWidth = viewport.clientWidth;
  const availableHeight = viewport.clientHeight;
  if (availableWidth <= 0 || availableHeight <= 0) return;
  const width = Math.min(availableWidth, availableHeight * RADAR_MAP_ASPECT);
  const height = width / RADAR_MAP_ASPECT;
  stage.style.width = `${width}px`;
  stage.style.height = `${height}px`;
}

function stopRadarStream() {
  if (radarReconnectTimer) { clearTimeout(radarReconnectTimer); radarReconnectTimer = 0; }
  if (radarEventSource) { radarEventSource.close(); radarEventSource = null; }
  radarStreamState = "closed";
  if ($("radarStreamStatus")) $("radarStreamStatus").textContent = "SSE 已暂停";
}

function startRadarStream() {
  if (activePage !== "radar" || document.hidden || typeof EventSource === "undefined") return;
  stopRadarStream();
  radarStreamState = "connecting";
  const source = new EventSource(`/api/radar-stream?generation=${radarGeneration}`);
  radarEventSource = source;
  $("radarStreamStatus").textContent = "SSE 连接中";
  source.onopen = () => {
    radarStreamState = "open";
    $("radarStreamStatus").textContent = "SSE latest-only 实时推送";
  };
  source.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      const serverNow = Number(payload.now || 0);
      if (serverNow > 0) displayPerf.latencySamples.push(Math.max(0, Date.now() - serverNow * 1000));
      radarGeneration = Math.max(radarGeneration, Number(payload.generation || 0));
      radarLatest = payload;
      radarDirty = true;
      renderRadarData(payload);
    } catch (_error) {
      radarStreamState = "parse_error";
      $("radarStreamStatus").textContent = "雷达数据解析失败";
    }
  };
  source.onerror = () => {
    if (radarEventSource !== source) return;
    source.close(); radarEventSource = null;
    radarStreamState = "reconnecting";
    $("radarStreamStatus").textContent = "SSE 重连中";
    radarReconnectTimer = window.setTimeout(startRadarStream, 1000);
  };
}

function setActivePage(page) {
  activePage = page === "radar" ? "radar" : "radio";
  history.replaceState(null, "", activePage === "radio" ? "#radio" : location.pathname + location.search);
  $("radioPage").hidden = activePage !== "radio";
  $("radarPage").hidden = activePage !== "radar";
  $("radioPage").classList.toggle("active", activePage === "radio");
  $("radarPage").classList.toggle("active", activePage === "radar");
  $("radioTab").classList.toggle("active", activePage === "radio");
  $("radarTab").classList.toggle("active", activePage === "radar");
  if (activePage === "radar") {
    stopSpectrumStream();
    fitRadarMapStage();
    radarDirty = true;
    if (state.latest) renderRadarData({ radar: state.latest.radar, vision_radar: state.latest.vision_radar });
    startRadarStream();
  } else {
    stopRadarStream();
    startSpectrumStream();
    spectrumDirty = true;
  }
}

function selectRadarRobot(name, scrollTable = false) {
  selectedRobot = name;
  radarDirty = true;
  document.querySelectorAll("#radarRobotRows tr").forEach((row) => row.classList.toggle("selected", row.dataset.robot === name));
  if (scrollTable) document.querySelector(`#radarRobotRows tr[data-robot="${name}"]`)?.scrollIntoView({ block: "nearest" });
}

/* ---------- Render loop: 60fps interpolation, pushed waterfall, 10fps DOM ---------- */
function stepSmoothing() {
  const EASE = 0.55;
  let moving = false;
  for (const role of ["rx1", "rx2"]) {
    const tgt = spectrumTarget[role];
    if (!tgt) continue;
    let sm = spectrumSmooth[role];
    // 频点数组变化（等级切换/首帧）时直接对齐，避免插值错位
    if (!sm || sm.power.length !== tgt.power.length) {
      sm = { freqs: tgt.freqs.slice(), power: tgt.power.slice(), timestamp: tgt.timestamp, meta: tgt.meta };
      spectrumSmooth[role] = sm;
      moving = true;
      continue;
    }
    sm.freqs = tgt.freqs;
    sm.timestamp = tgt.timestamp;
    sm.meta = tgt.meta;
    for (let i = 0; i < sm.power.length; i += 1) {
      const d = tgt.power[i] - sm.power[i];
      if (Math.abs(d) > 0.05) { sm.power[i] += d * EASE; moving = true; }
      else sm.power[i] = tgt.power[i];
    }
  }
  return moving;
}

function renderLoop(nowMs) {
  if (activePage === "radio") {
    flushWaterfalls();
    if (spectrumDirty && nowMs - lastSpectrumRenderMs >= 16) {
      const moving = stepSmoothing();
      drawSpectrum();
      spectrumDirty = moving;
      lastSpectrumRenderMs = nowMs;
      displayPerf.renderFrames += 1;
    }
    const refreshArrivalMetrics = nowMs - lastFrameRenderMs >= 250;
    if ((framesDirty || refreshArrivalMetrics) && nowMs - lastFrameRenderMs >= 100) {
      renderReception(get(state.latest, ["health", "modules"], {}), framesDirty);
      framesDirty = false;
      lastFrameRenderMs = nowMs;
    }
  } else if (radarDirty) {
    radarDirty = drawRadarMap();
    displayPerf.renderFrames += 1;
  }
  requestAnimationFrame(renderLoop);
}



function collectConfig() {
  const intelligent = $("intelligentToggle").checked;
  const configured = get(state.latest || {}, ["configured"], {});
  return {
    mode: "match_rx",
    intelligent_mode: intelligent,
    // 智能模式下下拉框显示有效值，但保存时不得覆盖人工基线。
    radio_side: intelligent ? (configured.radio_side || "red") : $("radioSide").value,
    interference_level: intelligent
      ? Number(configured.interference_level || 1)
      : Number($("interferenceLevel").value || 1),
    broadcast_gain: Number($("rx1Gain").value || 20),
    sdr_uris: { rx1: $("rx1Uri").value.trim(), rx2: $("rx2Uri").value.trim() },
    referee: {
      port: $("refereePort").value.trim(),
      baudrate: Number($("refereeBaudrate").value || 115200),
    },
  };
}

// 实时控制栏：切模式 / 手动改红蓝方或等级时，立即热下发（不需点保存配置）。
async function pushLiveControl() {
  try {
    render(await apiPost("/api/config", collectConfig()));
  } catch (error) {
    $("summary").textContent = `切换失败：${error.message}`;
  }
}

async function refresh() {
  if (state.busy) return;
  state.busy = true;
  try {
    render(await apiGet("/api/state"));
  } catch (error) {
    $("summary").textContent = `Panel API 异常：${error.message}`;
  } finally {
    state.busy = false;
  }
}

async function saveConfig() {
  render(await apiPost("/api/config", { ...collectConfig(), persist: true }));
}
async function checkSdr() { await apiGet("/api/check_sdr"); await refresh(); }
async function clearState() {
  await apiPost("/api/clear", {});
  latestByCmd.rx1 = {};
  latestByCmd.rx2 = {};
  frameArrivalHistory.rx1 = {};
  frameArrivalHistory.rx2 = {};
  lastFrameSeen = { rx1: 0, rx2: 0 };
  spectrumTarget.rx1 = null;
  spectrumTarget.rx2 = null;
  spectrumSmooth.rx1 = null;
  spectrumSmooth.rx2 = null;
  waterfallQueues.rx1.length = 0;
  waterfallQueues.rx2.length = 0;
  for (const id of ["rx1Waterfall", "rx2Waterfall"]) {
    const canvas = $(id);
    if (canvas) { canvas.width = 0; canvas.height = 0; }
  }
  spectrumDirty = true;
  framesDirty = true;
  await refresh();
}

async function processAction(name, action) {
  if (action === "start") await apiPost("/api/start", { ...collectConfig(), name });
  else await apiPost("/api/stop", { name });
  await refresh();
}

/* ---------- Drawer ---------- */
function openDrawer() {
  $("drawer").hidden = false;
  if (state.latest) { renderNodes(state.latest); renderProcesses(state.latest); renderEvents(state.latest); }
}
function closeDrawer() { $("drawer").hidden = true; }

document.addEventListener("click", async (event) => {
  const target = event.target;
  if (!(target instanceof HTMLElement)) return;
  try {
    if (target.id === "refreshBtn") await refresh();
    if (target.id === "checkBtn") await checkSdr();
    if (target.id === "clearBtn") await clearState();
    if (target.id === "saveConfigBtn") await saveConfig();
    if (target.id === "advBtn") openDrawer();
    if (target.dataset.page) setActivePage(target.dataset.page);
    const robotRow = target.closest("#radarRobotRows tr[data-robot]");
    if (robotRow) selectRadarRobot(robotRow.dataset.robot, false);
    if (target.id === "drawerClose" || target === $("drawer")) closeDrawer();
    if (target.dataset.start) await processAction(target.dataset.start, "start");
    if (target.dataset.stop) await processAction(target.dataset.stop, "stop");
    if (target.classList.contains("dtab")) {
      document.querySelectorAll(".dtab").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".dtab-page").forEach((p) => p.classList.remove("active"));
      target.classList.add("active");
      $(`dtab-${target.dataset.dtab}`).classList.add("active");
    }
  } catch (error) {
    $("summary").textContent = `操作失败：${error.message}`;
  }
});

window.addEventListener("resize", () => {
  fitRadarMapStage();
  spectrumDirty = true;
  for (const id of ["rx1Waterfall", "rx2Waterfall"]) {
    const canvas = $(id);
    if (canvas) { canvas.width = 0; canvas.height = 0; }
  }
});
if (typeof ResizeObserver !== "undefined") {
  const radarMapResizeObserver = new ResizeObserver(fitRadarMapStage);
  radarMapResizeObserver.observe($("radarMapViewport"));
}
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { stopSpectrumStream(); stopRadarStream(); }
  else if (activePage === "radar") startRadarStream();
  else startSpectrumStream();
});
window.addEventListener("beforeunload", () => { stopSpectrumStream(); stopRadarStream(); });

for (const id of ["showFinal", "showVision", "showRadio", "showLabels"]) {
  $(id).addEventListener("change", () => { radarDirty = true; });
}

$("radarMapCanvas").addEventListener("mousemove", (event) => {
  if (!radarLatest) return;
  const canvas = $("radarMapCanvas");
  const rect = canvas.getBoundingClientRect();
  const x = ((event.clientX - rect.left) / Math.max(rect.width, 1)) * canvas.width;
  const y = ((event.clientY - rect.top) / Math.max(rect.height, 1)) * canvas.height;
  let closest = null;
  let distance = Infinity;
  for (const hit of radarHitTargets) {
    const current = Math.hypot(hit.x - x, hit.y - y);
    if (current < 42 && current < distance) { closest = hit; distance = current; }
  }
  const tooltip = $("radarTooltip");
  if (!closest) {
    tooltip.hidden = true;
    canvas.dataset.hoverRobot = "";
    return;
  }
  const fusion = radarLatest.radar?.data || get(state.latest, ["radar", "data"], {});
  const item = get(fusion, ["robots", closest.name], {});
  const allyPrefix = radarMapOwnSide === "red" ? "R" : "B";
  tooltip.innerHTML = `<strong>${closest.name} · ${closest.name.startsWith(allyPrefix) ? "己方" : "对方"} · ${escapeHtml(closest.layer)}</strong><br>
    最终 ${coordText(radarPosition(item.final))} · ${escapeHtml(item.final?.source || "missing")}<br>
    视觉 ${coordText(radarPosition(item.visual))} · ${escapeHtml(item.visual?.state || "missing")} · 置信度 ${item.visual?.confidence == null ? "—" : Number(item.visual.confidence).toFixed(2)}<br>
    无线电 ${coordText(radarPosition(item.radio))} · 年龄 ${ageText(item.radio?.age_sec)}`;
  tooltip.style.left = `${Math.min(rect.width - 300, Math.max(8, event.clientX - rect.left + 14))}px`;
  tooltip.style.top = `${Math.min(rect.height - 105, Math.max(8, event.clientY - rect.top + 14))}px`;
  tooltip.hidden = false;
  canvas.dataset.hoverRobot = closest.name;
});
$("radarMapCanvas").addEventListener("mouseleave", () => {
  $("radarTooltip").hidden = true;
  $("radarMapCanvas").dataset.hoverRobot = "";
});
$("radarMapCanvas").addEventListener("click", () => {
  const name = $("radarMapCanvas").dataset.hoverRobot;
  if (name) selectRadarRobot(name, true);
});

// 实时控制栏：change 即热下发
$("intelligentToggle").addEventListener("change", pushLiveControl);
$("radioSide").addEventListener("change", pushLiveControl);
$("interferenceLevel").addEventListener("change", pushLiveControl);
$("rx1Gain").addEventListener("input", () => {
  $("rx1GainValue").textContent = `${Number($("rx1Gain").value).toFixed(0)} dB`;
});
$("rx1Gain").addEventListener("change", pushLiveControl);
$("spectrumTap").addEventListener("change", () => {
  spectrumTarget.rx1 = null;
  spectrumTarget.rx2 = null;
  spectrumSmooth.rx1 = null;
  spectrumSmooth.rx2 = null;
  waterfallQueues.rx1.length = 0;
  waterfallQueues.rx2.length = 0;
  for (const id of ["rx1Waterfall", "rx2Waterfall"]) {
    const canvas = $(id);
    if (canvas) { canvas.width = 0; canvas.height = 0; }
  }
  spectrumDirty = true;
  startSpectrumStream();
});

refresh();
if (activePage === "radio") dataTick();
setActivePage(activePage);
requestAnimationFrame(renderLoop);
setInterval(refresh, 2000);          // 全量状态仅承载配置/健康摘要
setInterval(() => { if (activePage === "radio" && spectrumStreamState !== "open") dataTick(); }, 1000); // SSE 断线兜底
setInterval(updateDisplayPerformance, 1000);
