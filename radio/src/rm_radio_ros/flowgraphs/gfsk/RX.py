"""Pure receive-only GNU Radio flowgraph for SharkRadio match operation.

This module intentionally contains no packet generator, PDU transmit path or
RF sink. It selects exactly one demodulator at construction time:

* ``mm`` for the continuous Gaussian + Mueller & Müller information receiver;
* ``legacy`` for the interference receiver's proven ``digital.gfsk_demod``.

The ROS wrapper supplies configuration through environment variables and the
setter methods below. Raw IQ is also tapped for spectrum telemetry and, when
enabled by the wrapper, the RX2 IQ-assisted decoder.
"""

from __future__ import annotations

import os
import threading

import numpy as np
from PyQt5 import Qt
from gnuradio import blocks, digital, filter, gr, iio, qtgui
from gnuradio.fft import window
from gnuradio.filter import firdes
import sip

from continuous_gfsk_demod import ContinuousGfskDemod


_BROADCAST_FREQS = {
    "red": 433_200_000,
    "blue": 433_920_000,
}

BROADCAST_BPF = {
    "red": {"low_cut": 5_000, "high_cut": 260_000, "transition": 50_000},
    "blue": {"low_cut": 5_000, "high_cut": 260_000, "transition": 50_000},
}

_INTERFERENCE_RX_SETTINGS = {
    "red": {
        1: {"cen_f": 432_200_000, "bw_re": 940_000, "LowPass": 500_000},
        2: {"cen_f": 432_500_000, "bw_re": 860_000, "LowPass": 500_000},
        3: {"cen_f": 432_800_000, "bw_re": 250_000, "LowPass": 160_000},
    },
    "blue": {
        1: {"cen_f": 434_920_000, "bw_re": 940_000, "LowPass": 500_000},
        2: {"cen_f": 434_620_000, "bw_re": 860_000, "LowPass": 500_000},
        3: {"cen_f": 434_320_000, "bw_re": 250_000, "LowPass": 160_000},
    },
}

_INTERFERENCE_SENSITIVITY = {
    1: 2.8194 / 2.0,
    2: 2.5681 / 2.0,
    3: 0.6517 / 2.0,
}

_RX_GAIN_MODES = ("manual", "slow_attack", "fast_attack", "hybrid")
_RX_GAIN_MODE_ALIASES = {
    "slowattack": "slow_attack",
    "slow-attack": "slow_attack",
    "fastattack": "fast_attack",
    "fast-attack": "fast_attack",
}
_BROADCAST_RX_GAIN_DEFAULT = 20.0
_BROADCAST_RX_GAIN_HARD_MIN = -1.0
_BROADCAST_RX_GAIN_HARD_MAX = 73.0


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, "1" if default else "0")
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _env_side() -> str:
    side = os.environ.get("RM_RADIO_SIDE", "red").strip().lower()
    return side if side in _BROADCAST_FREQS else "red"


def _env_interference_level() -> int:
    try:
        level = int(os.environ.get("RM_RADIO_INTERFERENCE_LEVEL", "1"))
    except ValueError:
        level = 1
    return level if level in (1, 2, 3) else 1


def _clean_rx_gain_mode(value, default: str) -> str:
    clean = str(value or default).strip().lower()
    clean = _RX_GAIN_MODE_ALIASES.get(clean, clean)
    return clean if clean in _RX_GAIN_MODES else default


def _env_rx_gain_mode(profile: str) -> str:
    env_profile = "INTERFERENCE" if profile == "interference" else "BROADCAST"
    default = "fast_attack" if profile == "interference" else "manual"
    raw = (
        os.environ.get(f"{env_profile}_RX_GAIN_MODE")
        or os.environ.get(f"RM_RADIO_{env_profile}_RX_GAIN_MODE")
        or os.environ.get("RX_GAIN_MODE")
        or os.environ.get("RM_RADIO_RX_GAIN_MODE")
    )
    return _clean_rx_gain_mode(raw, default)


def _env_rx_gain(profile: str) -> float:
    env_profile = "INTERFERENCE" if profile == "interference" else "BROADCAST"
    raw = (
        os.environ.get(f"{env_profile}_RX_GAIN")
        or os.environ.get(f"RM_RADIO_{env_profile}_RX_GAIN")
        or os.environ.get("RX_GAIN")
        or os.environ.get("RM_RADIO_RX_GAIN")
    )
    try:
        value = float(raw) if raw is not None and str(raw).strip() else _BROADCAST_RX_GAIN_DEFAULT
    except ValueError:
        value = 20.0
    if profile == "broadcast":
        return min(max(value, _BROADCAST_RX_GAIN_HARD_MIN), _env_broadcast_gain_max())
    return value


def _env_broadcast_gain_max() -> float:
    try:
        value = float(os.environ.get("BROADCAST_RX_GAIN_MAX", _BROADCAST_RX_GAIN_HARD_MAX))
    except (TypeError, ValueError):
        value = _BROADCAST_RX_GAIN_HARD_MAX
    return min(max(value, _BROADCAST_RX_GAIN_HARD_MIN), _BROADCAST_RX_GAIN_HARD_MAX)


def _env_demod_mode(profile: str) -> str:
    default = "mm" if profile == "broadcast" else "legacy"
    raw = (
        os.environ.get(f"RM_RADIO_{profile.upper()}_DEMOD_MODE")
        or os.environ.get("RM_RADIO_DEMOD_MODE")
        or default
    )
    clean = str(raw).strip().lower().replace("-", "_")
    aliases = {
        "continuous": "mm",
        "gaussian_mm": "mm",
        "m_and_m": "mm",
        "gfsk_demod": "legacy",
    }
    clean = aliases.get(clean, clean)
    return clean if clean in ("mm", "legacy") else default


def _env_input_sps() -> float:
    try:
        value = float(os.environ.get("RM_RADIO_INPUT_SPS", "94"))
    except (TypeError, ValueError):
        value = 94.0
    if value <= 1.0:
        raise ValueError(f"RM_RADIO_INPUT_SPS must be greater than 1, got {value!r}")
    return value


def _set_iio_rx_bandwidth(source, bandwidth: float) -> None:
    setter = getattr(source, "set_bandwidth", None)
    if setter is not None:
        setter(int(bandwidth))
        return
    filter_setter = getattr(source, "set_filter_params", None)
    if filter_setter is not None:
        filter_setter("Auto", "", 0, 0)


class _DemodBitCallbackSink(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name="demod_bit_callback_sink",
            in_sig=[np.uint8],
            out_sig=None,
        )
        self._callback = None
        self._lock = threading.RLock()

    def set_callback(self, callback) -> None:
        with self._lock:
            self._callback = callback

    def work(self, input_items, _output_items):
        data = bytes((int(value) & 0x01) for value in input_items[0])
        with self._lock:
            callback = self._callback
        if callback is not None and data:
            callback(data)
        return len(input_items[0])


class _ComplexCallbackSink(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name="complex_callback_sink",
            in_sig=[np.complex64],
            out_sig=None,
        )
        self._callback = None
        self._lock = threading.RLock()

    def set_callback(self, callback) -> None:
        with self._lock:
            self._callback = callback

    def work(self, input_items, _output_items):
        samples = np.asarray(input_items[0], dtype=np.complex64)
        with self._lock:
            callback = self._callback
        if callback is not None and samples.size:
            callback(samples.copy())
        return len(input_items[0])


class RadioRx(gr.top_block, Qt.QWidget):
    """Receive-only top block shared by RX1 and RX2."""

    def __init__(self):
        gr.top_block.__init__(self, "SharkRadio RX", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("SharkRadio RX")
        self.flowgraph_started = threading.Event()

        self.rx_profile = os.environ.get("RM_RADIO_RX_PROFILE", "broadcast").strip().lower()
        if self.rx_profile not in ("broadcast", "interference"):
            self.rx_profile = "broadcast"
        self.frontend_profile = os.environ.get("RM_RADIO_FRONTEND_PROFILE", "auto").strip().lower()
        if self.frontend_profile == "auto":
            self.frontend_profile = self.rx_profile
        if self.frontend_profile not in ("broadcast", "interference"):
            raise ValueError(
                "RM_RADIO_FRONTEND_PROFILE must be auto, broadcast, or interference, "
                f"got {self.frontend_profile!r}"
            )
        self.side = _env_side()
        self.interference_level = _env_interference_level()
        if self.rx_profile == "interference":
            defaults = _INTERFERENCE_RX_SETTINGS[self.side][self.interference_level]
            sensitivity = _INTERFERENCE_SENSITIVITY[self.interference_level]
        else:
            broadcast_bpf = BROADCAST_BPF[self.side]
            defaults = {
                "cen_f": _BROADCAST_FREQS[self.side],
                "bw_re": 540_000,
                "LowPass": broadcast_bpf["high_cut"],
            }
            sensitivity = 1.5628 / 2.0

        self.sample_rate = 2_000_000.0
        # Legacy gfsk_demod cannot change symbol_sync SPS after construction,
        # so this deployment parameter must be applied before building it.
        self.sps = _env_input_sps()
        self.BT = 0.35
        self.BW = 540_000.0
        self.center_F = float(_BROADCAST_FREQS[self.side])
        self.cen_f = float(defaults["cen_f"])
        self.bw_re = float(defaults["bw_re"])
        self.LowPass = float(defaults["LowPass"])
        self.sen_re = float(sensitivity)
        self.sncy = "sncy_gr" if self.rx_profile == "interference" else "sncy_gb"
        self.broadcast_gain_max = _env_broadcast_gain_max()
        self.GainMode = _env_rx_gain_mode(self.rx_profile)
        self.Gain = _env_rx_gain(self.rx_profile)
        # Keep quadrature, RF-DC and BB-DC tracking enabled on both RX paths.
        # Disabling these on the interference profile leaves the LO/DC centre
        # spike uncompensated and makes the two receivers behave differently.
        self.rx_tracking = True
        self.demod_mode = _env_demod_mode(self.rx_profile)
        self.disable_gui_sinks = _env_bool("RM_RADIO_DISABLE_GUI_SINKS", True)

        self._layout = Qt.QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)

        self.rx_uri = os.environ.get("RM_RADIO_RX_URI", "ip:192.168.2.1").strip() or iio.get_pluto_uri()
        self.iio_source = iio.fmcomms2_source_fc32(self.rx_uri, [True, True], 32768)
        self.iio_source.set_len_tag_key("")
        self.iio_source.set_frequency(int(self.cen_f))
        self.iio_source.set_samplerate(int(self.sample_rate))
        _set_iio_rx_bandwidth(self.iio_source, self.bw_re)
        self.iio_source.set_gain_mode(0, self.GainMode)
        if self.GainMode == "manual":
            self.iio_source.set_gain(0, self.Gain)
        self.iio_source.set_quadrature(self.rx_tracking)
        self.iio_source.set_rfdc(self.rx_tracking)
        self.iio_source.set_bbdc(self.rx_tracking)
        self.iio_source.set_filter_params("Auto", "", 0, 0)

        if self.frontend_profile == "broadcast":
            # A real-tap band-pass keeps both positive and negative IQ
            # frequencies while rejecting the carrier/DC region. RX2 retains
            # its proven two-stage low-pass path below.
            self.broadcast_band_pass = filter.fir_filter_ccf(
                1,
                self._broadcast_band_pass_taps(),
            )
            self.filtered_iq_source = self.broadcast_band_pass
        else:
            self.low_pass_1 = filter.fir_filter_ccf(1, self._low_pass_taps())
            self.low_pass_2 = filter.fir_filter_ccf(1, self._low_pass_taps())
            self.filtered_iq_source = self.low_pass_2

        if self.demod_mode == "mm":
            self.demodulator = ContinuousGfskDemod(
                input_sps=self.sps,
                input_sensitivity=self.sen_re,
                bt=self.BT,
                decimation=2,
            )
        else:
            self.demodulator = digital.gfsk_demod(
                samples_per_symbol=self.sps,
                sensitivity=self.sen_re,
                gain_mu=0.175,
                mu=0.5,
                omega_relative_limit=0.005,
                freq_error=0.0,
                verbose=False,
                log=False,
            )

        self.demod_bit_callback_sink = _DemodBitCallbackSink()
        self.iq_callback_sink = _ComplexCallbackSink()
        self.filtered_iq_callback_sink = _ComplexCallbackSink()

        if self.frontend_profile == "broadcast":
            self.connect(self.iio_source, self.broadcast_band_pass, self.demodulator)
        else:
            self.connect(self.iio_source, self.low_pass_1, self.low_pass_2, self.demodulator)
        self.connect(self.demodulator, self.demod_bit_callback_sink)
        self.connect(self.iio_source, self.iq_callback_sink)
        self.connect(self.filtered_iq_source, self.filtered_iq_callback_sink)

        self.frequency_sink = None
        self.waterfall_sink = None
        if not self.disable_gui_sinks:
            self._create_gui_sinks()

    def _low_pass_taps(self):
        cutoff = min(max(1_000.0, float(self.LowPass)), self.sample_rate * 0.49)
        return firdes.low_pass(1, self.sample_rate, cutoff, 10_000, window.WIN_HAMMING, 6.76)

    def _broadcast_band_pass_taps(self):
        settings = BROADCAST_BPF[self.side]
        high_cut = min(
            max(float(settings["low_cut"]) + 1.0, float(self.LowPass)),
            self.sample_rate * 0.49,
        )
        return firdes.band_pass(
            1,
            self.sample_rate,
            float(settings["low_cut"]),
            high_cut,
            float(settings["transition"]),
            window.WIN_HAMMING,
            6.76,
        )

    def _create_gui_sinks(self) -> None:
        self.frequency_sink = qtgui.freq_sink_c(
            8192,
            window.WIN_BLACKMAN_hARRIS,
            self.cen_f,
            self.sample_rate,
            "接收频谱",
            1,
            None,
        )
        self.frequency_sink.set_update_time(0.10)
        self.frequency_sink.set_y_axis(-140, 10)
        self.frequency_sink.enable_grid(True)
        frequency_widget = sip.wrapinstance(self.frequency_sink.qwidget(), Qt.QWidget)
        self._layout.addWidget(frequency_widget)

        self.waterfall_sink = qtgui.waterfall_sink_c(
            4096,
            window.WIN_BLACKMAN_hARRIS,
            self.cen_f,
            self.sample_rate,
            "接收瀑布图",
            1,
            None,
        )
        self.waterfall_sink.set_update_time(0.10)
        self.waterfall_sink.set_intensity_range(-140, 10)
        waterfall_widget = sip.wrapinstance(self.waterfall_sink.qwidget(), Qt.QWidget)
        self._layout.addWidget(waterfall_widget)

        self.connect(self.filtered_iq_source, self.frequency_sink)
        self.connect(self.filtered_iq_source, self.waterfall_sink)

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, value) -> None:
        self.sample_rate = float(value)
        self.iio_source.set_samplerate(int(self.sample_rate))
        if self.frontend_profile == "broadcast":
            self.broadcast_band_pass.set_taps(self._broadcast_band_pass_taps())
        else:
            taps = self._low_pass_taps()
            self.low_pass_1.set_taps(taps)
            self.low_pass_2.set_taps(taps)
        if self.frequency_sink is not None:
            self.frequency_sink.set_frequency_range(self.cen_f, self.sample_rate)
        if self.waterfall_sink is not None:
            self.waterfall_sink.set_frequency_range(self.cen_f, self.sample_rate)

    def get_sps(self):
        return self.sps

    def set_sps(self, value) -> None:
        self.sps = float(value)
        setter = getattr(self.demodulator, "set_input_sps", None)
        if setter is not None:
            setter(self.sps)

    def get_cen_f(self):
        return self.cen_f

    def set_cen_f(self, value) -> None:
        self.cen_f = float(value)
        self.iio_source.set_frequency(int(self.cen_f))
        if self.frequency_sink is not None:
            self.frequency_sink.set_frequency_range(self.cen_f, self.sample_rate)
        if self.waterfall_sink is not None:
            self.waterfall_sink.set_frequency_range(self.cen_f, self.sample_rate)

    def set_center_F(self, value) -> None:
        self.center_F = float(value)

    def set_BW(self, value) -> None:
        self.BW = float(value)

    def set_bw_re(self, value) -> None:
        self.bw_re = float(value)
        _set_iio_rx_bandwidth(self.iio_source, self.bw_re)

    def set_LowPass(self, value) -> None:
        self.LowPass = float(value)
        if self.frontend_profile == "broadcast":
            self.broadcast_band_pass.set_taps(self._broadcast_band_pass_taps())
        else:
            taps = self._low_pass_taps()
            self.low_pass_1.set_taps(taps)
            self.low_pass_2.set_taps(taps)

    def set_sen_re(self, value) -> None:
        self.sen_re = float(value)
        setter = getattr(self.demodulator, "set_input_sensitivity", None)
        if setter is not None:
            setter(self.sen_re)
        elif hasattr(self.demodulator, "fmdemod"):
            self.demodulator.fmdemod.set_gain(1.0 / self.sen_re)

    def set_sncy(self, value) -> None:
        self.sncy = value

    def set_GainMode(self, value) -> None:
        self.GainMode = _clean_rx_gain_mode(value, self.GainMode)
        self.iio_source.set_gain_mode(0, self.GainMode)
        if self.GainMode == "manual":
            self.iio_source.set_gain(0, self.Gain)

    def set_Gain(self, value) -> None:
        requested = float(value)
        if self.rx_profile == "broadcast" and not (
            _BROADCAST_RX_GAIN_HARD_MIN <= requested <= self.broadcast_gain_max
        ):
            raise ValueError(
                f"broadcast RX gain must be within "
                f"{_BROADCAST_RX_GAIN_HARD_MIN:g}..{self.broadcast_gain_max:g} dB, got {value!r}"
            )
        self.Gain = requested
        if self.GainMode == "manual":
            self.iio_source.set_gain(0, self.Gain)

    def get_rx_gain_diagnostics(self):
        return {
            "profile": self.rx_profile,
            "mode": self.GainMode,
            "gain_db": float(self.Gain),
            "configured_min_db": (
                float(_BROADCAST_RX_GAIN_HARD_MIN)
                if self.rx_profile == "broadcast"
                else None
            ),
            "configured_max_db": (
                float(self.broadcast_gain_max)
                if self.rx_profile == "broadcast"
                else None
            ),
        }

    def set_rx_tracking(self, value) -> None:
        if isinstance(value, str):
            value = value.strip().lower() in ("1", "true", "yes", "on")
        self.rx_tracking = bool(value)
        self.iio_source.set_quadrature(self.rx_tracking)
        self.iio_source.set_rfdc(self.rx_tracking)
        self.iio_source.set_bbdc(self.rx_tracking)

    def set_BT(self, value) -> None:
        self.BT = float(value)
        setter = getattr(self.demodulator, "set_bt", None)
        if setter is not None:
            setter(self.BT)

    def get_demod_mode(self):
        return self.demod_mode

    def get_frontend_profile(self):
        return self.frontend_profile

    def get_demod_configuration(self):
        configuration = getattr(self.demodulator, "configuration", None)
        if configuration is not None:
            return configuration()
        return {
            "mode": "legacy_gfsk_demod",
            "input_sps": self.sps,
            "bt": self.BT,
        }

    def set_demod_bits_callback(self, callback) -> None:
        self.demod_bit_callback_sink.set_callback(callback)

    def set_iq_callback(self, callback) -> None:
        self.iq_callback_sink.set_callback(callback)

    def set_filtered_iq_callback(self, callback) -> None:
        self.filtered_iq_callback_sink.set_callback(callback)

    def closeEvent(self, event) -> None:
        self.stop()
        self.wait()
        event.accept()
