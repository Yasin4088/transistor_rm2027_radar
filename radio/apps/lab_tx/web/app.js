const robotLabels = {
  opponent_hero: "英雄", opponent_engineer: "工程", opponent_infantry_3: "步兵3",
  opponent_infantry_4: "步兵4", opponent_aerial: "空中", opponent_sentry: "哨兵", reserved: "保留",
};
const robotKeys = ["opponent_hero", "opponent_engineer", "opponent_infantry_3", "opponent_infantry_4", "opponent_aerial", "opponent_sentry"];
const hpKeys = ["opponent_hero", "opponent_engineer", "opponent_infantry_3", "opponent_infantry_4", "reserved", "opponent_sentry"];
const bulletKeys = ["opponent_hero", "opponent_infantry_3", "opponent_infantry_4", "opponent_aerial", "opponent_sentry"];
const buffKeys = ["opponent_hero", "opponent_engineer", "opponent_infantry_3", "opponent_infantry_4", "opponent_sentry"];
const hpMax = {
  opponent_hero: 600, opponent_engineer: 250, opponent_infantry_3: 400,
  opponent_infantry_4: 400, reserved: 0, opponent_sentry: 400,
};
const bulletMax = {
  opponent_hero: 100, opponent_infantry_3: 1500, opponent_infantry_4: 1500,
  opponent_aerial: 750, opponent_sentry: 1900,
};
const buffMax = {
  hp_recovery_percent: Object.fromEntries(buffKeys.map((key) => [key, 25])),
  shooting_heat_cooling: {
    opponent_hero: 200, opponent_engineer: 0, opponent_infantry_3: 120,
    opponent_infantry_4: 120, opponent_sentry: 120,
  },
  defense_percent: Object.fromEntries(buffKeys.map((key) => [key, 99])),
  negative_defense_percent: Object.fromEntries(buffKeys.map((key) => [key, 100])),
  attack_percent: Object.fromEntries(buffKeys.map((key) => [key, 300])),
};
const cmdList = [
  ["0x0A01", "坐标"], ["0x0A02", "血量"], ["0x0A03", "发弹量"], ["0x0A04", "经济"], ["0x0A05", "增益状态"],
];

let serverState = null;
let broadcastData = null;      // current broadcast payload config
let waveStatus = { broadcast: { running: false }, interference: { running: false } };
let formDefaultsLoaded = false;
let broadcastAutoChangeBusy = false;

const defaultTxAttenuation = { broadcast: 61, interference: 11 };
const fourSdrRoles = [
  ["broadcast_tx", "信息波 TX", "broadcastTxUri", "broadcastTxStatus"],
  ["interference_tx", "干扰波 TX", "interferenceTxUri", "interferenceTxStatus"],
];

const $ = (id) => document.getElementById(id);

function numberValue(id, fallback = 0) {
  const value = Number($(id).value);
  return Number.isFinite(value) ? value : fallback;
}
function intValue(id, fallback = 0) {
  const raw = $(id).value.trim();
  const parsed = raw.startsWith("0x") || raw.startsWith("0X") ? parseInt(raw, 16) : parseInt(raw, 10);
  return Number.isFinite(parsed) ? parsed : fallback;
}
function clampInt(value, low, high, fallback = 0) {
  const parsed = Number.isFinite(Number(value)) ? Math.trunc(Number(value)) : fallback;
  return Math.min(Math.max(parsed, low), high);
}
function normalizeOccupationBits(value) {
  const bits = clampInt(value, 0, 0xFFFFFFFF, 0) & 0xFFFF;
  const central = Math.min((bits >>> 1) & 0x03, 2);
  const fortress = (bits >>> 4) & 0x03;
  const outpost = Math.min((bits >>> 6) & 0x03, 2);
  return (bits & ((1 << 0) | (1 << 3) | 0xFF00)) | (central << 1) | (fortress << 4) | (outpost << 6);
}

async function api(path, body = null) {
  const options = body
    ? { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }
    : {};
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `${path} ${response.status}`);
  return payload;
}

let toastTimer = null;
function toast(message, kind = "") {
  const el = $("toast");
  el.textContent = message;
  el.className = `toast ${kind}`;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 3200);
}

function boolText(value) { return value ? "是" : "否"; }

/* ---------- TX-only link state ---------- */
function collectState() {
  const txSampleRate = intValue("fourTxSampleRate", 1000000);
  const broadcast = { ...(broadcastData || {}) };
  broadcast.auto_change_data = $("broadcastAutoChange").checked;
  return {
    side: $("side").value,
    level: Number($("level").value),
    password: $("password").value.trim(),
    mode: "actual_tx",
    tx: { lab_confirm: $("labConfirm").checked },
    four_sdr: {
      broadcast_tx: {
        uri: $("broadcastTxUri").value.trim(),
        rf_port: "A",
        attenuation_db: numberValue("broadcastTxAttenuation", defaultTxAttenuation.broadcast),
        sample_rate: txSampleRate,
      },
      interference_tx: {
        uri: $("interferenceTxUri").value.trim(),
        rf_port: "A",
        attenuation_db: numberValue("interferenceTxAttenuation", defaultTxAttenuation.interference),
        sample_rate: txSampleRate,
      },
    },
    broadcast,
  };
}

function applyServerDefaults(state) {
  if (formDefaultsLoaded || !state) return;
  const four = state.four_sdr || {};
  const broadcast = four.broadcast_tx || {};
  const interference = four.interference_tx || {};
  if (state.side) $("side").value = state.side;
  if (state.level) $("level").value = String(state.level);
  if (state.password) $("password").value = state.password;
  $("broadcastAutoChange").checked = state.broadcast?.auto_change_data === true;
  $("broadcastTxUri").value = broadcast.uri || "ip:192.168.2.1";
  $("interferenceTxUri").value = interference.uri || "ip:192.168.3.1";
  $("broadcastTxAttenuation").value = String(broadcast.attenuation_db ?? defaultTxAttenuation.broadcast);
  $("interferenceTxAttenuation").value = String(interference.attenuation_db ?? defaultTxAttenuation.interference);
  $("fourTxSampleRate").value = String(
    interference.sample_rate ?? broadcast.sample_rate ?? 1000000
  );
  formDefaultsLoaded = true;
}

/* ---------- Frame-data editor (drawer) ---------- */
function buildInputs() {
  const cmdRoot = $("enabledCmds");
  cmdRoot.innerHTML = "";
  for (const [cmd, label] of cmdList) {
    const item = document.createElement("label");
    item.innerHTML = `<input class="cmd-enabled" data-cmd="${cmd}" type="checkbox" checked />${cmd} ${label}`;
    cmdRoot.appendChild(item);
  }
  const pos = $("positionsTable");
  pos.innerHTML = `<div class="head">机器人</div><div class="head">X cm</div><div class="head">Y cm</div>`;
  for (const key of robotKeys) {
    pos.insertAdjacentHTML("beforeend",
      `<div class="row-label">${robotLabels[key]}</div>
       <input data-pos-x="${key}" type="number" min="0" max="2800" step="1" />
       <input data-pos-y="${key}" type="number" min="0" max="1500" step="1" />`);
  }
  const hp = $("hpTable");
  hp.innerHTML = `<div class="head">对象</div><div class="head">HP</div>`;
  for (const key of hpKeys) {
    hp.insertAdjacentHTML("beforeend",
      `<div class="row-label">${robotLabels[key]}</div><input data-hp="${key}" type="number" min="0" max="${hpMax[key]}" step="1" />`);
  }
  const bullets = $("bulletsTable");
  bullets.innerHTML = `<div class="head">对象</div><div class="head">剩余量</div>`;
  for (const key of bulletKeys) {
    bullets.insertAdjacentHTML("beforeend",
      `<div class="row-label">${robotLabels[key]}</div><input data-bullets="${key}" type="number" min="0" max="${bulletMax[key]}" step="1" />`);
  }
  const buffs = $("buffTable");
  buffs.innerHTML =
    `<div class="head">对象</div><div class="head">回血%</div><div class="head">冷却</div>` +
    `<div class="head">防御%</div><div class="head">负防%</div><div class="head">攻击%</div>` +
    `<div class="head">主要状态</div>`;
  for (const key of buffKeys) {
    buffs.insertAdjacentHTML("beforeend",
      `<div class="row-label">${robotLabels[key]}</div>
       <input data-buff="${key}" data-field="hp_recovery_percent" type="number" min="0" max="${buffMax.hp_recovery_percent[key]}" step="1" />
       <input data-buff="${key}" data-field="shooting_heat_cooling" type="number" min="0" max="${buffMax.shooting_heat_cooling[key]}" step="1" />
       <input data-buff="${key}" data-field="defense_percent" type="number" min="0" max="${buffMax.defense_percent[key]}" step="1" />
       <input data-buff="${key}" data-field="negative_defense_percent" type="number" min="0" max="${buffMax.negative_defense_percent[key]}" step="1" />
       <input data-buff="${key}" data-field="attack_percent" type="number" min="0" max="${buffMax.attack_percent[key]}" step="1" />
       <input data-main-status="${key}" type="number" min="0" max="3" step="1" />`);
  }
}

function fillEditor(broadcast) {
  broadcast = broadcast || {};
  const enabled = new Set(broadcast.enabled_cmds || cmdList.map((i) => i[0]));
  document.querySelectorAll(".cmd-enabled").forEach((input) => { input.checked = enabled.has(input.dataset.cmd); });
  for (const key of robotKeys) {
    const item = broadcast.robots?.[key] || {};
    document.querySelector(`[data-pos-x="${key}"]`).value = item.x ?? 0;
    document.querySelector(`[data-pos-y="${key}"]`).value = item.y ?? 0;
  }
  for (const key of hpKeys) document.querySelector(`[data-hp="${key}"]`).value = broadcast.hp?.[key] ?? 0;
  for (const key of bulletKeys) document.querySelector(`[data-bullets="${key}"]`).value = broadcast.bullets?.[key] ?? 0;
  $("remainingCoins").value = broadcast.macro?.remaining_coins ?? 0;
  $("totalCoins").value = broadcast.macro?.total_coins ?? 0;
  $("occupationBits").value = `0x${Number(broadcast.macro?.occupation_bits ?? 0).toString(16)}`;
  for (const key of buffKeys) {
    const item = broadcast.buffs?.[key] || {};
    document.querySelectorAll(`[data-buff="${key}"]`).forEach((input) => { input.value = item[input.dataset.field] ?? 0; });
    document.querySelector(`[data-main-status="${key}"]`).value = broadcast.robot_main_status?.[key] ?? 0;
  }
  $("sentryMode").value = broadcast.sentry_mode ?? 2;
}

function collectEditor() {
  const enabled = Array.from(document.querySelectorAll(".cmd-enabled")).filter((i) => i.checked).map((i) => i.dataset.cmd);
  const robots = {};
  for (const key of robotKeys) robots[key] = {
    x: clampInt(document.querySelector(`[data-pos-x="${key}"]`).value, 0, 2800),
    y: clampInt(document.querySelector(`[data-pos-y="${key}"]`).value, 0, 1500),
  };
  const hp = {}; for (const key of hpKeys) hp[key] = clampInt(document.querySelector(`[data-hp="${key}"]`).value, 0, hpMax[key]);
  const bullets = {}; for (const key of bulletKeys) bullets[key] = clampInt(document.querySelector(`[data-bullets="${key}"]`).value, 0, bulletMax[key]);
  const buffs = {};
  const robotMainStatus = {};
  for (const key of buffKeys) {
    buffs[key] = {};
    document.querySelectorAll(`[data-buff="${key}"]`).forEach((input) => {
      buffs[key][input.dataset.field] = clampInt(input.value, 0, buffMax[input.dataset.field][key]);
    });
    robotMainStatus[key] = clampInt(document.querySelector(`[data-main-status="${key}"]`).value, 0, 3);
  }
  const totalCoins = clampInt(intValue("totalCoins", 0), 0, 65535);
  return {
    enabled_cmds: enabled, robots, hp, bullets,
    macro: {
      remaining_coins: clampInt(intValue("remainingCoins", 0), 0, totalCoins),
      total_coins: totalCoins,
      occupation_bits: normalizeOccupationBits(intValue("occupationBits", 0)),
    },
    buffs,
    sentry_mode: clampInt(intValue("sentryMode", 2), 1, 6, 2),
    robot_main_status: robotMainStatus,
  };
}

/* ---------- Rendering wave state ---------- */
function renderWave(wave) {
  const st = waveStatus[wave] || {};
  const diag = st.diagnostics || {};
  const running = Boolean(st.running);
  const waitingForDevice = wave === "interference" && !$("interferenceTxUri").value.trim();
  const card = $(`${wave}Card`);
  const state = $(`${wave}State`);
  const toggle = $(`${wave}Toggle`);
  card.classList.toggle("live", running);
  state.className = `tx-state ${running ? "live" : "off"}`;
  state.textContent = running ? (st.external ? "外部发射中" : "发射中") : waitingForDevice ? "等待连接" : "未发射";
  toggle.className = `toggle-btn ${running ? "live" : "off"}`;
  toggle.textContent = running ? "停止发射" : waitingForDevice ? "等待识别" : "开始发射";
  const confirmed = Boolean(serverState?.lab_tx_effective_confirm);
  toggle.disabled = (!confirmed || waitingForDevice) && !running;
  if (wave === "interference") $("interferenceApply").disabled = !running || waitingForDevice;

  // metrics
  const metrics = $(`${wave}Metrics`);
  const underflowAge = Math.max(0, Number(diag.underflow_last_change_age_sec || 0));
  const underflowState = String(diag.underflow_state || "");
  let underflow = running ? "等待日志" : "—";
  let underflowKind = "";
  if (diag.underflow_available) {
    if (underflowState === "clean") {
      underflow = "连续 · 0 U";
      underflowKind = "ok";
    } else if (underflowState === "active") {
      underflow = `${diag.underflow_count} 次 U · 刚增加`;
      underflowKind = "bad";
    } else if (underflowState === "stable") {
      underflow = `${diag.underflow_count} 次 U · 已稳定 ${Math.floor(underflowAge)}s`;
      underflowKind = "warn";
    } else if (underflowState === "historical") {
      underflow = `上次 ${diag.underflow_count} 次 U`;
    } else {
      underflow = `${diag.underflow_count || 0} 次 U`;
      underflowKind = diag.underflow_detected ? "bad" : "ok";
    }
  }
  if (wave === "broadcast") {
    const autoChange = $("broadcastAutoChange").checked;
    $("broadcastAutoChangeLabel").classList.toggle("active", autoChange);
    metrics.innerHTML = metric("频点", `${$("side").value === "blue" ? "433.92" : "433.20"} MHz`)
      + metric("PID", running ? st.pid ?? "—" : "—")
      + metric("命令", (broadcastData?.enabled_cmds || cmdList.map(c=>c[0])).length + (autoChange ? " 项 · 自动变化" : " 项"))
      + metric("供样 / Underflow", underflow, underflowKind);
  } else {
    metrics.innerHTML = metric("等级", `L${$("level").value}`)
      + metric("PID", running ? st.pid ?? "—" : "—")
      + metric("密钥", $("password").value.trim() || "—")
      + metric("供样 / Underflow", underflow, underflowKind);
  }
}
function metric(label, value, kind = "") {
  return `<div class="m"><div class="ml">${label}</div><div class="mv ${kind}">${value}</div></div>`;
}

function renderSdrStatus(result) {
  const root = $("sdrStatus");
  root.innerHTML = "";
  const add = (label, item) => {
    const ok = item?.available === true;
    const unknown = item?.available == null;
    const waiting = unknown && !item?.uri && String(item?.message || "").includes("等待识别");
    const row = document.createElement("div");
    row.className = `status-item ${ok ? "ok" : unknown ? "warn" : "bad"}`;
    row.textContent = `${label} ${item?.uri || ""}：${ok ? "可达" : waiting ? "待连接" : unknown ? "未知" : "不可达"} - ${item?.message || ""}`;
    root.appendChild(row);
  };
  for (const [role, label] of fourSdrRoles) {
    if (result?.[role]) {
      add(label, result[role]);
      updateDeviceCard(role, result[role]);
    }
  }
  if (result?.topology?.errors?.length) result.topology.errors.forEach((t) => add("拓扑", { available: false, message: t }));
  if (result?.topology?.ok) {
    const message = result.topology.mode === "single_broadcast"
      ? "信息波 TX 可用；干扰波 TX 等待单独接入识别"
      : "双台 TX 可用且 URI 未冲突";
    add("拓扑", { available: true, message });
  }
}

function updateDeviceCard(role, item) {
  const meta = fourSdrRoles.find(([k]) => k === role);
  if (!meta) return;
  const [, label, inputId, statusId] = meta;
  const el = $(statusId);
  if (!el) return;
  const ok = item?.available === true;
  const unknown = item?.available == null;
  const waiting = unknown && !item?.uri && String(item?.message || "").includes("等待识别");
  el.className = `device-status ${ok ? "ok" : unknown ? "unknown" : "bad"}`;
  const stateText = ok ? "可达" : waiting ? "待连接" : unknown ? "检测中" : "不可达";
  el.textContent = `${stateText}：${item?.uri || $(inputId).value.trim() || label}${item?.message ? ` | ${item.message}` : ""}`;
}

function resetDeviceCards(text = "未检测") {
  for (const [role, , inputId] of fourSdrRoles) updateDeviceCard(role, { uri: $(inputId)?.value.trim() || "", available: null, message: text });
}

/* ---------- Wave control actions ---------- */
async function toggleWave(wave) {
  const running = Boolean(waveStatus[wave]?.running);
  try {
    if (running) {
      await api("/api/tx-wave-stop", { wave });
      toast(`${wave === "broadcast" ? "信息波" : "干扰波"}已停止发射`, "ok");
    } else {
      await api("/api/tx-wave-start", { wave, state: collectState() });
      toast(`${wave === "broadcast" ? "信息波" : "干扰波"}已开始发射`, "ok");
    }
    await refresh();
  } catch (error) {
    toast(error.message, "bad");
  }
}

async function reconfigureInterference() {
  try {
    await api("/api/tx-wave-reconfigure", { wave: "interference", state: collectState(), changes: {} });
    toast(`干扰波已热切换到 L${$("level").value} / 密钥 ${$("password").value.trim()}`, "ok");
    await refresh();
  } catch (error) {
    toast(error.message, "bad");
  }
}

async function randomizeBroadcast() {
  try {
    const result = await api("/api/randomize-broadcast", { state: collectState() });
    broadcastData = result.broadcast;
    fillEditor(broadcastData);
    renderWave("broadcast");
    if (waveStatus.broadcast?.running) {
      await api("/api/tx-wave-reconfigure", { wave: "broadcast", state: collectState(), changes: {} });
      toast("信息波数据已随机并热切换", "ok");
    } else {
      toast("信息波数据已随机（下次发射生效）", "ok");
    }
  } catch (error) {
    toast(error.message, "bad");
  }
}

async function toggleBroadcastAutoChange() {
  const input = $("broadcastAutoChange");
  const enabled = input.checked;
  renderWave("broadcast");
  if (!waveStatus.broadcast?.running) {
    toast(enabled ? "自动变化已选中，将在信息波发射时生效" : "信息波将使用固定数据", "ok");
    return;
  }
  broadcastAutoChangeBusy = true;
  input.disabled = true;
  try {
    await api("/api/tx-wave-reconfigure", { wave: "broadcast", state: collectState(), changes: {} });
    toast(enabled ? "信息波已切换为每 100 ms 自动变化数据" : "信息波已切换为固定数据", "ok");
    await refresh();
  } catch (error) {
    input.checked = !enabled;
    renderWave("broadcast");
    toast(error.message, "bad");
  } finally {
    broadcastAutoChangeBusy = false;
    input.disabled = false;
  }
}

async function randomizePassword() {
  try {
    const result = await api("/api/randomize-password", { state: collectState() });
    $("password").value = result.password;
    renderWave("interference");
    if (waveStatus.interference?.running) {
      await api("/api/tx-wave-reconfigure", { wave: "interference", state: collectState(), changes: {} });
      toast(`干扰密钥已随机为 ${result.password} 并热切换`, "ok");
    } else {
      toast(`干扰密钥已随机为 ${result.password}`, "ok");
    }
  } catch (error) {
    toast(error.message, "bad");
  }
}

/* ---------- Drawer ---------- */
function openDrawer() {
  fillEditor(broadcastData);
  $("drawer").hidden = false;
}
function closeDrawer() { $("drawer").hidden = true; }

async function applyEditor() {
  broadcastData = collectEditor();
  closeDrawer();
  renderWave("broadcast");
  if (waveStatus.broadcast?.running) {
    try {
      await api("/api/tx-wave-reconfigure", { wave: "broadcast", state: collectState(), changes: {} });
      toast("信息波数据已保存并热切换", "ok");
    } catch (error) { toast(error.message, "bad"); }
  } else {
    toast("信息波数据已保存（下次发射生效）", "ok");
  }
}

/* ---------- Lab confirm ---------- */
async function onLabConfirm() {
  try {
    const result = await api("/api/lab-confirm", { confirmed: $("labConfirm").checked });
    serverState = { ...(serverState || {}), ...result };
    renderWave("broadcast");
    renderWave("interference");
  } catch (error) {
    toast(error.message, "bad");
  }
}

async function checkSdr() {
  resetDeviceCards("检测中...");
  try {
    renderSdrStatus(await api("/api/check-four-sdr", { state: collectState() }));
  } catch (error) {
    resetDeviceCards("检测失败");
    toast(error.message, "bad");
  }
}

/* ---------- Refresh loop ---------- */
async function refresh() {
  try {
    const result = await api("/api/state");
    serverState = result.server;
    waveStatus = result.server.waves || waveStatus;
    applyServerDefaults(result.state);
    if (
      waveStatus.broadcast?.running
      && typeof waveStatus.broadcast.auto_change_data === "boolean"
      && !broadcastAutoChangeBusy
    ) {
      $("broadcastAutoChange").checked = waveStatus.broadcast.auto_change_data;
    }
    $("labConfirm").checked = Boolean(result.server.lab_tx_effective_confirm);
    $("serverLine").textContent =
      `工作目录：${result.server.workspace} | 安全确认：${boolText(result.server.lab_tx_effective_confirm)} | ` +
      `信息波：${waveStatus.broadcast?.running ? "发射中" : "停"} | 干扰波：${waveStatus.interference?.running ? "发射中" : "停"}`;
    if (broadcastData === null) broadcastData = result.state?.broadcast || null;
    renderWave("broadcast");
    renderWave("interference");
  } catch (error) {
    $("serverLine").textContent = error.message;
  }
}

/* ---------- Wiring ---------- */
function wire() {
  $("labConfirm").addEventListener("change", onLabConfirm);
  $("broadcastToggle").addEventListener("click", () => toggleWave("broadcast"));
  $("broadcastAutoChange").addEventListener("change", toggleBroadcastAutoChange);
  $("interferenceToggle").addEventListener("click", () => toggleWave("interference"));
  $("broadcastRandom").addEventListener("click", randomizeBroadcast);
  $("interferenceRandom").addEventListener("click", randomizePassword);
  $("interferenceApply").addEventListener("click", reconfigureInterference);
  $("broadcastEdit").addEventListener("click", openDrawer);
  $("drawerClose").addEventListener("click", closeDrawer);
  $("drawerApply").addEventListener("click", applyEditor);
  $("drawerRandom").addEventListener("click", async () => {
    await randomizeBroadcast();
    fillEditor(broadcastData);
  });
  $("checkFourSdrBtn").addEventListener("click", checkSdr);
  $("level").addEventListener("change", () => renderWave("interference"));
  $("password").addEventListener("input", () => renderWave("interference"));
  $("interferenceTxUri").addEventListener("input", () => renderWave("interference"));
  $("side").addEventListener("change", () => renderWave("broadcast"));
  document.querySelectorAll(".dtab").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".dtab").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".dtab-page").forEach((p) => p.classList.remove("active"));
      btn.classList.add("active");
      $(`dtab-${btn.dataset.dtab}`).classList.add("active");
    });
  });
}

async function init() {
  buildInputs();
  wire();
  await refresh();
  await checkSdr();
  setInterval(refresh, 3000);
}

init();
