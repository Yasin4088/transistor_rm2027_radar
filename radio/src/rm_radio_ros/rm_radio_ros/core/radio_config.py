from __future__ import annotations


BROADCAST_FREQUENCIES = {
    'red': 433_200_000,
    'blue': 433_920_000,
}

OFFICIAL_GFSK_SAMPLE_RATE = 1_000_000
OFFICIAL_GFSK_SPS = 47

GFSK_FLOWGRAPH_SAMPLE_RATE = 2_000_000
GFSK_FLOWGRAPH_SPS = 94

# Keep the generated TX literal as well as symbol-period compatible with the
# RM2026 V2.0.1 air-interface definition.
GFSK_TX_FLOWGRAPH_SAMPLE_RATE = OFFICIAL_GFSK_SAMPLE_RATE
GFSK_TX_FLOWGRAPH_SPS = OFFICIAL_GFSK_SPS

BROADCAST_RX_GAIN_DEFAULT = 20.0
# Both deployed RX devices report AD9361 manual hardwaregain_available as
# ``[-1 1 73]``. Align the operator range with the hardware instead of keeping
# the former 0..25 dB application-only restriction.
BROADCAST_RX_GAIN_MIN = -1.0
BROADCAST_RX_GAIN_MAX = 73.0
BROADCAST_RX_SEN_RE = 1.5628 / 2.0
INTERFERENCE_RX_SEN_RE = {
    1: 2.8194 / 2.0,
    2: 2.5681 / 2.0,
    3: 0.6517 / 2.0,
}

BROADCAST_RX_SETTERS = {
    side: {
        'cen_f': freq,
        'center_F': freq,
        'BW': 540_000,
        'LowPass': 260_000,
        'bw_re': 540_000,
        'sen_re': BROADCAST_RX_SEN_RE,
        'rx_tracking': True,
        'GainMode': 'manual',
        'Gain': BROADCAST_RX_GAIN_DEFAULT,
    }
    for side, freq in BROADCAST_FREQUENCIES.items()
}


INTERFERENCE_LEVEL_SETTERS = {
    'red': {
        1: {'center_f': 432_200_000, 'BW_ganrao': 940_000},
        2: {'center_f': 432_500_000, 'BW_ganrao': 860_000},
        3: {'center_f': 432_800_000, 'BW_ganrao': 250_000},
    },
    'blue': {
        1: {'center_f': 434_920_000, 'BW_ganrao': 940_000},
        2: {'center_f': 434_620_000, 'BW_ganrao': 860_000},
        3: {'center_f': 434_320_000, 'BW_ganrao': 250_000},
    },
}


INTERFERENCE_RX_LEVEL_SETTERS = {
    'red': {
        1: {
            'cen_f': 432_200_000,
            'center_F': 432_200_000,
            'BW': 940_000,
            'LowPass': 500_000,
            'bw_re': 940_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[1],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
        2: {
            'cen_f': 432_500_000,
            'center_F': 432_500_000,
            'BW': 860_000,
            'LowPass': 500_000,
            'bw_re': 860_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[2],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
        3: {
            'cen_f': 432_800_000,
            'center_F': 432_800_000,
            'BW': 250_000,
            'LowPass': 160_000,
            'bw_re': 250_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[3],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
    },
    'blue': {
        1: {
            'cen_f': 434_920_000,
            'center_F': 434_920_000,
            'BW': 940_000,
            'LowPass': 500_000,
            'bw_re': 940_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[1],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
        2: {
            'cen_f': 434_620_000,
            'center_F': 434_620_000,
            'BW': 860_000,
            'LowPass': 500_000,
            'bw_re': 860_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[2],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
        3: {
            'cen_f': 434_320_000,
            'center_F': 434_320_000,
            'BW': 250_000,
            'LowPass': 160_000,
            'bw_re': 250_000,
            'sncy': 'sncy_gr',
            'sen_re': INTERFERENCE_RX_SEN_RE[3],
            'rx_tracking': True,
            'GainMode': 'fast_attack',
            'Gain': 20,
        },
    },
}


def normalize_radio_side(side: str) -> str:
    clean = str(side).strip().lower()
    if clean not in INTERFERENCE_LEVEL_SETTERS:
        raise ValueError(f'radio side must be red or blue, got {side!r}')
    return clean


def normalize_interference_level(level: int) -> int:
    value = int(level)
    if value not in (1, 2, 3):
        raise ValueError(f'interference level must be 1, 2, or 3, got {level!r}')
    return value


def interference_setters_for_side_level(side: str, level: int) -> dict:
    clean_side = normalize_radio_side(side)
    clean_level = normalize_interference_level(level)
    return dict(INTERFERENCE_LEVEL_SETTERS[clean_side][clean_level])


def interference_rx_setters_for_side_level(side: str, level: int) -> dict:
    clean_side = normalize_radio_side(side)
    clean_level = normalize_interference_level(level)
    return dict(INTERFERENCE_RX_LEVEL_SETTERS[clean_side][clean_level])


def normalize_broadcast_rx_gain(gain: float) -> float:
    value = float(gain)
    if not BROADCAST_RX_GAIN_MIN <= value <= BROADCAST_RX_GAIN_MAX:
        raise ValueError(
            f'broadcast RX gain must be within '
            f'{BROADCAST_RX_GAIN_MIN:g}..{BROADCAST_RX_GAIN_MAX:g} dB, got {gain!r}'
        )
    return value


def broadcast_rx_setters_for_side(side: str, gain: float = BROADCAST_RX_GAIN_DEFAULT) -> dict:
    clean_side = normalize_radio_side(side)
    setters = dict(BROADCAST_RX_SETTERS[clean_side])
    setters['Gain'] = normalize_broadcast_rx_gain(gain)
    return setters
