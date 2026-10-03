"""Continuous GFSK receive chain for the RoboMaster information wave.

The block follows the useful part of dfdtc/RMUC2026_RF's V1.4 broadcast
receiver while keeping SharkRadio's access-code and referee-frame parsers:

    complex IQ -> digital AGC -> decimate -> FM discriminator
               -> Gaussian filter -> float AGC -> M&M clock recovery
               -> binary slicer

The live SDR runs at 2 MS/s and 94 samples/symbol.  Decimating by two makes
the timing loop operate at the rule-book 1 MS/s and 47 samples/symbol, which
also makes the discriminator sensitivity match the published 1.5628 value.
"""

from __future__ import annotations

from gnuradio import analog, digital, filter, gr
from gnuradio.filter import firdes


def _agc(block_type, *, rate: float, reference: float, gain: float, max_gain: float):
    """Construct an AGC across GNU Radio versions with 3/4-argument bindings."""

    block = block_type(float(rate), float(reference), float(gain))
    setter = getattr(block, 'set_max_gain', None)
    if setter is not None:
        setter(float(max_gain))
    return block


class ContinuousGfskDemod(gr.hier_block2):
    """Hard-bit GFSK demodulator with continuous M&M symbol-clock tracking."""

    def __init__(
        self,
        *,
        input_sps: float = 94.0,
        input_sensitivity: float = 1.5628 / 2.0,
        bt: float = 0.35,
        decimation: int = 2,
        gaussian_taps: int = 60,
        loop_bandwidth: float = 0.046,
        damping_factor: float = 1.0,
        ted_gain: float = 0.85,
        max_deviation: float = 1.5,
    ):
        gr.hier_block2.__init__(
            self,
            'continuous_gfsk_demod',
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(1, 1, gr.sizeof_char),
        )

        if float(input_sps) <= 1.0:
            raise ValueError(f'input_sps must be greater than 1, got {input_sps!r}')
        if abs(float(input_sensitivity)) <= 1e-12:
            raise ValueError('input_sensitivity must be non-zero')
        if int(decimation) < 1:
            raise ValueError(f'decimation must be positive, got {decimation!r}')
        if int(gaussian_taps) < 3:
            raise ValueError(f'gaussian_taps must be at least 3, got {gaussian_taps!r}')
        if float(bt) <= 0.0:
            raise ValueError(f'bt must be positive, got {bt!r}')

        self._input_sps = float(input_sps)
        self._input_sensitivity = float(input_sensitivity)
        self._bt = float(bt)
        self._decimation = int(decimation)
        self._gaussian_taps = int(gaussian_taps)

        self.complex_agc = _agc(
            analog.agc_cc,
            rate=1e-4,
            reference=1.0,
            gain=1.0,
            max_gain=4.0,
        )
        self.decimator = filter.rational_resampler_ccc(
            interpolation=1,
            decimation=self._decimation,
            taps=[],
            fractional_bw=0,
        )
        self.fm_discriminator = analog.quadrature_demod_cf(self._discriminator_gain())
        self.gaussian_filter = filter.fir_filter_fff(1, self._make_gaussian_taps())
        self.float_agc = _agc(
            analog.agc_ff,
            rate=1e-4,
            reference=1.0,
            gain=1.0,
            max_gain=65536.0,
        )

        # M&M only needs a two-point real slicer. constellation_bpsk is
        # equivalent to the [-1, +1] calcdist constellation in the reference
        # flowgraph and is compatible with more GNU Radio Python bindings.
        constellation = digital.constellation_bpsk().base()
        self.clock_recovery = digital.symbol_sync_ff(
            digital.TED_MOD_MUELLER_AND_MULLER,
            self.output_sps,
            float(loop_bandwidth),
            float(damping_factor),
            float(ted_gain),
            float(max_deviation),
            1,
            constellation,
            digital.IR_PFB_NO_MF,
            32,
            [],
        )
        self.binary_slicer = digital.binary_slicer_fb()

        self.connect(
            self,
            self.complex_agc,
            self.decimator,
            self.fm_discriminator,
            self.gaussian_filter,
            self.float_agc,
            self.clock_recovery,
            self.binary_slicer,
            self,
        )

    @property
    def output_sps(self) -> float:
        return self._input_sps / float(self._decimation)

    @property
    def output_sensitivity(self) -> float:
        # Phase change per output sample grows by the decimation factor.
        return self._input_sensitivity * float(self._decimation)

    def _discriminator_gain(self) -> float:
        return 1.0 / self.output_sensitivity

    def _make_gaussian_taps(self):
        return firdes.gaussian(
            1.2,
            self.output_sps,
            self._bt,
            self._gaussian_taps,
        )

    def set_input_sensitivity(self, value: float) -> None:
        value = float(value)
        if abs(value) <= 1e-12:
            raise ValueError('input_sensitivity must be non-zero')
        self._input_sensitivity = value
        self.fm_discriminator.set_gain(self._discriminator_gain())

    def set_input_sps(self, value: float) -> None:
        value = float(value)
        if value <= 1.0:
            raise ValueError(f'input_sps must be greater than 1, got {value!r}')
        self._input_sps = value
        self.clock_recovery.set_sps(self.output_sps)
        self.gaussian_filter.set_taps(self._make_gaussian_taps())

    def set_bt(self, value: float) -> None:
        value = float(value)
        if value <= 0.0:
            raise ValueError(f'bt must be positive, got {value!r}')
        self._bt = value
        self.gaussian_filter.set_taps(self._make_gaussian_taps())

    def configuration(self) -> dict:
        """Return operator-facing values used by the live receive chain."""

        return {
            'input_sps': self._input_sps,
            'decimation': self._decimation,
            'output_sps': self.output_sps,
            'input_sensitivity': self._input_sensitivity,
            'output_sensitivity': self.output_sensitivity,
            'bt': self._bt,
            'gaussian_taps': self._gaussian_taps,
            'timing_error_detector': 'mueller_and_muller',
        }
