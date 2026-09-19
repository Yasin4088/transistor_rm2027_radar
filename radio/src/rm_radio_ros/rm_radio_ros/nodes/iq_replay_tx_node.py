from __future__ import annotations

import json
import gc
import hashlib
import shutil
import threading
import time
import traceback
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from gnuradio import blocks, gr, iio
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ..core.iq_replay import IqReplayItem, resolve_iq_replay_items


class IqReplayTopBlock(gr.top_block):
    def __init__(
        self,
        item: IqReplayItem,
        tx_uri: str,
        attenuation_db: float,
        amplitude_scale: float = 1.0,
        repeat_source: bool = False,
        tx_buffer_size: int = 1_048_576,
    ):
        gr.top_block.__init__(self, "RM_IQ_REPLAY_TX", catch_exceptions=True)
        self.item = item
        self.file_source = blocks.file_source(gr.sizeof_gr_complex, str(item.path), repeat_source, 0, 0)
        self.amplifier = blocks.multiply_const_cc(float(amplitude_scale))
        self.tx_sink = iio.fmcomms2_sink_fc32(tx_uri, [True, True, False, False], int(tx_buffer_size), False)
        self.tx_sink.set_len_tag_key("")
        self.tx_sink.set_bandwidth(int(item.bandwidth_hz))
        self.tx_sink.set_frequency(int(item.center_frequency))
        self.tx_sink.set_samplerate(int(item.sample_rate))
        self.tx_sink.set_attenuation(0, float(attenuation_db))
        self.tx_sink.set_filter_params("Auto", "", 0, 0)
        self.connect((self.file_source, 0), (self.amplifier, 0))
        self.connect((self.amplifier, 0), (self.tx_sink, 0))


class IqReplayTxNode(Node):
    def __init__(self):
        super().__init__("rm_iq_replay_tx_node")
        self.declare_parameter("tx_uri", "ip:192.168.1.10")
        self.declare_parameter("iq_paths", "")
        self.declare_parameter("radio_side", "red")
        self.declare_parameter("tx_type", "broadcast")
        self.declare_parameter("interference_level", 1)
        self.declare_parameter("attenuation_db", 60.0)
        self.declare_parameter("iq_amplitude_scale", 1.0)
        self.declare_parameter("sample_rate", 2_000_000)
        self.declare_parameter("repeat", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("tx_buffer_size", 1_048_576)
        self.declare_parameter("cache_iq_to_tmp", True)
        self.declare_parameter("iq_cache_dir", "/tmp/rm_radio_iq_replay")
        self.declare_parameter("status_period_sec", 1.0)

        self.items: list[IqReplayItem] = []
        self.source_items: list[IqReplayItem] = []
        self.current_index = -1
        self.current_item: Optional[IqReplayItem] = None
        self.started = False
        self.loaded = False
        self.last_error = ""
        self.replay_count = 0
        self.started_at = 0.0
        self.item_started_at = 0.0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._top_block: Optional[IqReplayTopBlock] = None
        self._lock = threading.RLock()

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_timer(max(float(self.get_parameter("status_period_sec").value), 0.1), self._publish_status)

        if self._load_items():
            self.start_replay()

    def _workspace_root(self) -> Path:
        return Path(__file__).resolve().parents[3]

    def _load_items(self) -> bool:
        try:
            self.items = resolve_iq_replay_items(
                str(self.get_parameter("iq_paths").value),
                self._workspace_root(),
                default_radio_side=str(self.get_parameter("radio_side").value).strip().lower(),
                default_tx_type=str(self.get_parameter("tx_type").value).strip().lower(),
                default_interference_level=int(self.get_parameter("interference_level").value),
                sample_rate=int(self.get_parameter("sample_rate").value),
            )
            self.source_items = list(self.items)
            self.items = self._prepare_runtime_items(self.items)
            self.loaded = True
            self.last_error = ""
            return True
        except Exception as exc:  # noqa: BLE001 - report via status
            self.loaded = False
            self.last_error = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.get_logger().error(self.last_error)
            return False

    def _prepare_runtime_items(self, items: list[IqReplayItem]) -> list[IqReplayItem]:
        if bool(self.get_parameter("dry_run").value) or not bool(self.get_parameter("cache_iq_to_tmp").value):
            return items
        cache_dir = Path(str(self.get_parameter("iq_cache_dir").value)).expanduser()
        cache_dir.mkdir(parents=True, exist_ok=True)
        runtime_items: list[IqReplayItem] = []
        for item in items:
            runtime_items.append(self._cache_item_to_tmp(item, cache_dir))
        return runtime_items

    def _cache_item_to_tmp(self, item: IqReplayItem, cache_dir: Path) -> IqReplayItem:
        source = item.path
        digest = hashlib.sha256(str(source).encode("utf-8")).hexdigest()[:16]
        token = f"{digest}_{source.name or 'iq'}.fc32"
        target = cache_dir / token
        if not target.exists() or target.stat().st_size != item.size_bytes:
            tmp_target = target.with_suffix(target.suffix + ".tmp")
            if tmp_target.exists():
                tmp_target.unlink()
            self.get_logger().info(f"caching IQ replay file {source} -> {target}")
            shutil.copy2(source, tmp_target)
            tmp_target.replace(target)
        return IqReplayItem(
            path=target,
            profile=item.profile,
            radio_side=item.radio_side,
            center_frequency=item.center_frequency,
            bandwidth_hz=item.bandwidth_hz,
            sample_rate=item.sample_rate,
            size_bytes=item.size_bytes,
            duration_seconds=item.duration_seconds,
        )

    def start_replay(self) -> bool:
        if not self.loaded and not self._load_items():
            return False
        if bool(self.get_parameter("dry_run").value):
            with self._lock:
                self.started = True
                self.started_at = time.time()
                self.current_index = 0
                self.current_item = self.items[0] if self.items else None
            self.get_logger().info("dry_run enabled; marked IQ replay TX as started")
            return True
        if self.started:
            return True
        self._stop_event.clear()
        self.started = True
        self.started_at = time.time()
        self._thread = threading.Thread(target=self._run_replay_loop, name="rm-iq-replay-tx", daemon=True)
        self._thread.start()
        self.get_logger().info(f"started IQ replay TX with {len(self.items)} file(s)")
        return True

    def stop_replay(self) -> None:
        self._stop_event.set()
        self._stop_current_top_block()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        with self._lock:
            self.started = False

    def _run_replay_loop(self) -> None:
        repeat = bool(self.get_parameter("repeat").value)
        tx_uri = str(self.get_parameter("tx_uri").value).strip()
        attenuation_db = float(self.get_parameter("attenuation_db").value)
        amplitude_scale = max(0.0, float(self.get_parameter("iq_amplitude_scale").value))
        tx_buffer_size = max(16_384, int(self.get_parameter("tx_buffer_size").value))
        try:
            if repeat and len(self.items) == 1:
                item = self.items[0]
                with self._lock:
                    self.current_index = 0
                    self.current_item = item
                    self.item_started_at = time.time()
                self.get_logger().info(
                    f"replaying IQ {item.path} at {item.center_frequency} Hz continuously "
                    f"with amplitude scale {amplitude_scale:g}"
                )
                top_block = IqReplayTopBlock(
                    item,
                    tx_uri,
                    attenuation_db,
                    amplitude_scale=amplitude_scale,
                    repeat_source=True,
                    tx_buffer_size=tx_buffer_size,
                )
                with self._lock:
                    self._top_block = top_block
                top_block.start()
                while not self._stop_event.is_set():
                    time.sleep(0.1)
                top_block.stop()
                top_block.wait()
                with self._lock:
                    self._top_block = None
                    self.replay_count += 1
                return

            while not self._stop_event.is_set():
                for index, item in enumerate(self.items):
                    if self._stop_event.is_set():
                        break
                    with self._lock:
                        self.current_index = index
                        self.current_item = item
                        self.item_started_at = time.time()
                    self.get_logger().info(
                        f"replaying IQ {item.path} at {item.center_frequency} Hz "
                        f"for {item.duration_seconds:.3f}s"
                    )
                    top_block = IqReplayTopBlock(
                        item,
                        tx_uri,
                        attenuation_db,
                        amplitude_scale=amplitude_scale,
                        tx_buffer_size=tx_buffer_size,
                    )
                    with self._lock:
                        self._top_block = top_block
                    top_block.start()
                    deadline = time.time() + item.duration_seconds + 0.25
                    while time.time() < deadline and not self._stop_event.is_set():
                        time.sleep(0.05)
                    top_block.stop()
                    top_block.wait()
                    with self._lock:
                        self._top_block = None
                        self.replay_count += 1
                    top_block = None
                    gc.collect()
                    time.sleep(0.1)
                if not repeat:
                    break
        except Exception as exc:  # noqa: BLE001 - keep status visible
            self.last_error = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.get_logger().error(self.last_error)
        finally:
            self._stop_current_top_block()
            with self._lock:
                self.started = False

    def _stop_current_top_block(self) -> None:
        with self._lock:
            top_block = self._top_block
            self._top_block = None
        if top_block is None:
            return
        try:
            top_block.stop()
            top_block.wait()
        except Exception as exc:  # noqa: BLE001
            self.last_error = "".join(traceback.format_exception_only(type(exc), exc)).strip()

    def _status_payload(self) -> str:
        with self._lock:
            elapsed = max(0.0, time.time() - self.item_started_at) if self.item_started_at else 0.0
            current_item = self.current_item
            payload = {
                "node": self.get_name(),
                "flowgraph_name": "iq_replay_tx",
                "loaded": self.loaded,
                "started": self.started,
                "dry_run": bool(self.get_parameter("dry_run").value),
                "tx_uri": str(self.get_parameter("tx_uri").value),
                "sample_rate": int(self.get_parameter("sample_rate").value),
                "iq_amplitude_scale": float(self.get_parameter("iq_amplitude_scale").value),
                "repeat": bool(self.get_parameter("repeat").value),
                "tx_buffer_size": int(self.get_parameter("tx_buffer_size").value),
                "cache_iq_to_tmp": bool(self.get_parameter("cache_iq_to_tmp").value),
                "iq_cache_dir": str(self.get_parameter("iq_cache_dir").value),
                "replay_count": self.replay_count,
                "current_index": self.current_index,
                "current_elapsed_seconds": elapsed,
                "current_item": None if current_item is None else current_item.to_dict(),
                "source_items": [item.to_dict() for item in self.source_items],
                "items": [item.to_dict() for item in self.items],
                "last_error": self.last_error,
            }
        return json.dumps(payload, ensure_ascii=False)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._status_payload()
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.stop_replay()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[IqReplayTxNode] = None
    try:
        node = IqReplayTxNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
