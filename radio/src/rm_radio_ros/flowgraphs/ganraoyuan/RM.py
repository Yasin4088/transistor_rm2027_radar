#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: RM
# Author: EGO
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
import os
import sys
import logging as log

def get_state_directory() -> str:
    oldpath = os.path.expanduser("~/.grc_gnuradio")
    try:
        from gnuradio.gr import paths
        newpath = paths.persistent()
        if os.path.exists(newpath):
            return newpath
        if os.path.exists(oldpath):
            log.warning(f"Found persistent state path '{newpath}', but file does not exist. " +
                     f"Old default persistent state path '{oldpath}' exists; using that. " +
                     "Please consider moving state to new location.")
            return oldpath
        # Default to the correct path if both are configured.
        # neither old, nor new path exist: create new path, return that
        os.makedirs(newpath, exist_ok=True)
        return newpath
    except (ImportError, NameError):
        log.warning("Could not retrieve GNU Radio persistent state directory from GNU Radio. " +
                 "Trying defaults.")
        xdgstate = os.getenv("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
        xdgcand = os.path.join(xdgstate, "gnuradio")
        if os.path.exists(xdgcand):
            return xdgcand
        if os.path.exists(oldpath):
            log.warning(f"Using legacy state path '{oldpath}'. Please consider moving state " +
                     f"files to '{xdgcand}'.")
            return oldpath
        # neither old, nor new path exist: create new path, return that
        os.makedirs(xdgcand, exist_ok=True)
        return xdgcand

sys.path.append(os.environ.get('GRC_HIER_PATH', get_state_directory()))

from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
from gnuradio import digital
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import iio
from jiang import jiang  # grc-generated hier_block
import math
import numpy as np
import sip
import threading


class _ComplexCallbackSink(gr.sync_block):
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='tx_complex_callback_sink',
            in_sig=[np.complex64],
            out_sig=None,
        )
        self._callback = None
        self._lock = threading.RLock()

    def set_callback(self, callback):
        with self._lock:
            self._callback = callback

    def work(self, input_items, _output_items):
        samples = np.asarray(input_items[0], dtype=np.complex64)
        with self._lock:
            callback = self._callback
        if callback is not None and samples.size:
            callback(samples.copy())
        return len(input_items[0])


_INTERFERENCE_TX_SETTINGS = {
    'red': {
        1: {'center_f': 432200000, 'BW_ganrao': 940000},
        2: {'center_f': 432500000, 'BW_ganrao': 860000},
        3: {'center_f': 432800000, 'BW_ganrao': 250000},
    },
    'blue': {
        1: {'center_f': 434920000, 'BW_ganrao': 940000},
        2: {'center_f': 434620000, 'BW_ganrao': 860000},
        3: {'center_f': 434320000, 'BW_ganrao': 250000},
    },
}


_OFFICIAL_SYMBOL_RATE = 1_000_000.0 / 47.0
_DEFAULT_TX_SAMPLE_RATE = 1_000_000


def _env_side():
    side = os.environ.get('RM_RADIO_SIDE', 'red').strip().lower()
    return side if side in _INTERFERENCE_TX_SETTINGS else 'red'


def _env_interference_level():
    try:
        level = int(os.environ.get('RM_RADIO_INTERFERENCE_LEVEL', '1'))
    except ValueError:
        level = 1
    return level if level in (1, 2, 3) else 1


def _env_tx_buffer_size():
    try:
        return max(16384, int(os.environ.get('RM_RADIO_TX_BUFFER_SIZE', '1048576')))
    except ValueError:
        return 1048576


def _env_tx_sample_rate():
    try:
        return max(1, int(os.environ.get('RM_RADIO_SAMPLE_RATE', str(_DEFAULT_TX_SAMPLE_RATE))))
    except ValueError:
        return _DEFAULT_TX_SAMPLE_RATE


def _sps_for_sample_rate(sample_rate):
    return max(2, int(round(float(sample_rate) / _OFFICIAL_SYMBOL_RATE)))


class RM(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "RM", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("RM")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "RM")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.sample_rate = sample_rate = _env_tx_sample_rate()
        self.sps = sps = _sps_for_sample_rate(sample_rate)
        self.bw3 = bw3 = 250000
        self.symbol_rate = symbol_rate = sample_rate/sps
        tx_defaults = _INTERFERENCE_TX_SETTINGS[_env_side()][_env_interference_level()]
        self.BW_ganrao = BW_ganrao = tx_defaults['BW_ganrao']
        self.to_F_ganrao = to_F_ganrao = BW_ganrao/2-symbol_rate
        self.center_f = center_f = tx_defaults['center_f']
        self.x = x = 0
        self.sen_ganrao = sen_ganrao = 2*math.pi*to_F_ganrao/sample_rate
        self.repeat = repeat = 1
        self.data = data = 6
        self.cmd_id = cmd_id = [0x0A,0x06]
        self.center_F_ganrao = center_F_ganrao = center_f
        self.bw2 = bw2 = 860000
        self.bw1 = bw1 = 940000
        self.attend_gr = attend_gr = 12.5
        self.access_xinxibo = access_xinxibo = [0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E]
        self.access_ganraobo = access_ganraobo = [0x16, 0xE8, 0xD3, 0x77, 0x15, 0x1C, 0x71, 0x2D]
        self.SOF = SOF = 0xA5
        self.BT = BT = 0.35
        self.disable_gui_sinks = os.environ.get('RM_RADIO_DISABLE_GUI_SINKS', '0').lower() in ('1', 'true', 'yes', 'on')
        self.tx_buffer_size = tx_buffer_size = _env_tx_buffer_size()

        ##################################################
        # Blocks
        ##################################################

        self._attend_gr_range = qtgui.Range(10, 89.75, 0.25, 12.5, 200)
        self._attend_gr_win = qtgui.RangeWidget(self._attend_gr_range, self.set_attend_gr, "干扰源衰减（dB）", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._attend_gr_win, 0, 0, 1, 2)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_waterfall_sink_x_0_0_0 = qtgui.waterfall_sink_c(
            4096, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            (center_F_ganrao*0), #fc
            sample_rate, #bw
            "干扰源发送瀑布图", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_waterfall_sink_x_0_0_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0_0_0.enable_grid(True)
        self.qtgui_waterfall_sink_x_0_0_0.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0_0_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0_0_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0_0_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0_0_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0_0_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_0_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0_0_0.qwidget(), Qt.QWidget)

        self.top_grid_layout.addWidget(self._qtgui_waterfall_sink_x_0_0_0_win, 3, 2, 1, 2)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_time_sink_x_0_1_4_1 = qtgui.time_sink_f(
            ((data+9)*10), #size
            sample_rate, #samp_rate
            "发送端二进制信号", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0_1_4_1.set_update_time(0.10)
        self.qtgui_time_sink_x_0_1_4_1.set_y_axis(-12, 12)

        self.qtgui_time_sink_x_0_1_4_1.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0_1_4_1.enable_tags(True)
        self.qtgui_time_sink_x_0_1_4_1.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, 'packet_len')
        self.qtgui_time_sink_x_0_1_4_1.enable_autoscale(True)
        self.qtgui_time_sink_x_0_1_4_1.enable_grid(True)
        self.qtgui_time_sink_x_0_1_4_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_0_1_4_1.enable_control_panel(False)
        self.qtgui_time_sink_x_0_1_4_1.enable_stem_plot(True)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0_1_4_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0_1_4_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0_1_4_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0_1_4_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0_1_4_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0_1_4_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0_1_4_1.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_1_4_1_win = sip.wrapinstance(self.qtgui_time_sink_x_0_1_4_1.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_time_sink_x_0_1_4_1_win, 2, 2, 1, 2)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_time_sink_x_0_1_1_1_0 = qtgui.time_sink_c(
            (1024*1), #size
            500000*2, #samp_rate
            "干扰源发送调制后的基带信号", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0_1_1_1_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0_1_1_1_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0_1_1_1_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0_1_1_1_0.enable_tags(True)
        self.qtgui_time_sink_x_0_1_1_1_0.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "packet_len")
        self.qtgui_time_sink_x_0_1_1_1_0.enable_autoscale(True)
        self.qtgui_time_sink_x_0_1_1_1_0.enable_grid(False)
        self.qtgui_time_sink_x_0_1_1_1_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0_1_1_1_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0_1_1_1_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_0_1_1_1_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0_1_1_1_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0_1_1_1_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0_1_1_1_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0_1_1_1_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0_1_1_1_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0_1_1_1_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0_1_1_1_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_1_1_1_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0_1_1_1_0.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_time_sink_x_0_1_1_1_0_win, 2, 0, 1, 2)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_freq_sink_x_0_0 = qtgui.freq_sink_c(
            4096, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            (center_F_ganrao*0), #fc
            sample_rate, #bw
            "干扰源发送频谱", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0_0.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_0_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0_0.enable_grid(True)
        self.qtgui_freq_sink_x_0_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0_0.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_0_0_win, 3, 0, 1, 2)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.jiang_0 = jiang(
            Access_Code=access_ganraobo,
            Period=100,
            com_id=cmd_id,
            payload_size=data,
            symbol_rate=symbol_rate,
        )
        # Bound scheduler read-ahead to the minimum GNU Radio permits for this
        # Python source. This keeps hot payload changes from accumulating an
        # unnecessarily large queue ahead of the hardware sink.
        self.jiang_0.to_basic_block().set_max_output_buffer(4096)
        self.interference_tx_uri = os.environ.get('RM_RADIO_INTERFERENCE_TX_URI', 'ip:192.168.1.10').strip() or 'ip:192.168.1.10'
        self.iio_fmcomms2_sink_0_0 = iio.fmcomms2_sink_fc32(self.interference_tx_uri, [True, True, False, False], tx_buffer_size, False)
        self.iio_fmcomms2_sink_0_0.set_len_tag_key('')
        self.iio_fmcomms2_sink_0_0.set_bandwidth(BW_ganrao)
        self.iio_fmcomms2_sink_0_0.set_frequency(int(center_F_ganrao))
        self.iio_fmcomms2_sink_0_0.set_samplerate(int(sample_rate))
        if True:
            self.iio_fmcomms2_sink_0_0.set_attenuation(0, attend_gr)
        if False:
            self.iio_fmcomms2_sink_0_0.set_attenuation(1, 20.0)
        self.iio_fmcomms2_sink_0_0.set_filter_params('Auto', '', 0, 0)
        self.digital_gfsk_mod_0_0 = digital.gfsk_mod(
            samples_per_symbol=sps,
            sensitivity=sen_ganrao,
            bt=BT,
            verbose=False,
            log=False,
            do_unpack=False)
        # Create the options list
        self._center_f_options = [432200000, 432500000, 432800000, 434920000, 434620000, 434320000]
        # Create the labels list
        self._center_f_labels = ['红方干扰源一', '红方干扰源二', '红方干扰源三', '蓝方干扰源一', '蓝方干扰源二', '蓝方干扰源三']
        # Create the combo box
        # Create the radio buttons
        self._center_f_group_box = Qt.QGroupBox("干扰源中心频点选择" + ": ")
        self._center_f_box = Qt.QHBoxLayout()
        class variable_chooser_button_group(Qt.QButtonGroup):
            def __init__(self, parent=None):
                Qt.QButtonGroup.__init__(self, parent)
            @pyqtSlot(int)
            def updateButtonChecked(self, button_id):
                self.button(button_id).setChecked(True)
        self._center_f_button_group = variable_chooser_button_group()
        self._center_f_group_box.setLayout(self._center_f_box)
        for i, _label in enumerate(self._center_f_labels):
            radio_button = Qt.QRadioButton(_label)
            self._center_f_box.addWidget(radio_button)
            self._center_f_button_group.addButton(radio_button, i)
        self._center_f_callback = lambda i: Qt.QMetaObject.invokeMethod(self._center_f_button_group, "updateButtonChecked", Qt.Q_ARG("int", self._center_f_options.index(i)))
        self._center_f_callback(self.center_f)
        self._center_f_button_group.buttonClicked[int].connect(
            lambda i: self.set_center_f(self._center_f_options[i]))
        self.top_grid_layout.addWidget(self._center_f_group_box, 0, 2, 1, 2)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(2, 4):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.blocks_uchar_to_float_2_0_1 = blocks.uchar_to_float()
        self.null_sink_complex_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.null_sink_complex_1 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.null_sink_complex_2 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.tx_iq_callback_sink = _ComplexCallbackSink()
        self.null_sink_float_0 = blocks.null_sink(gr.sizeof_float*1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.jiang_0, 0), (self.digital_gfsk_mod_0_0, 0))
        self.connect((self.jiang_0, 0), (self.blocks_uchar_to_float_2_0_1, 0))
        self.connect((self.blocks_uchar_to_float_2_0_1, 0), (self.null_sink_float_0 if self.disable_gui_sinks else self.qtgui_time_sink_x_0_1_4_1, 0))
        self.connect((self.digital_gfsk_mod_0_0, 0), (self.iio_fmcomms2_sink_0_0, 0))
        self.connect((self.digital_gfsk_mod_0_0, 0), (self.tx_iq_callback_sink, 0))
        self.connect((self.digital_gfsk_mod_0_0, 0), (self.null_sink_complex_0 if self.disable_gui_sinks else self.qtgui_freq_sink_x_0_0, 0))
        self.connect((self.digital_gfsk_mod_0_0, 0), (self.null_sink_complex_1 if self.disable_gui_sinks else self.qtgui_time_sink_x_0_1_1_1_0, 0))
        self.connect((self.digital_gfsk_mod_0_0, 0), (self.null_sink_complex_2 if self.disable_gui_sinks else self.qtgui_waterfall_sink_x_0_0_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "RM")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_symbol_rate(self.sample_rate/self.sps)

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_sen_ganrao(2*math.pi*self.to_F_ganrao/self.sample_rate)
        self.set_symbol_rate(self.sample_rate/self.sps)
        self.iio_fmcomms2_sink_0_0.set_samplerate(int(self.sample_rate))
        self.qtgui_freq_sink_x_0_0.set_frequency_range((self.center_F_ganrao*0), self.sample_rate)
        self.qtgui_time_sink_x_0_1_4_1.set_samp_rate(self.sample_rate)
        self.qtgui_waterfall_sink_x_0_0_0.set_frequency_range((self.center_F_ganrao*0), self.sample_rate)

    def get_bw3(self):
        return self.bw3

    def set_bw3(self, bw3):
        self.bw3 = bw3
        self.set_BW_ganrao(self.bw3)

    def get_symbol_rate(self):
        return self.symbol_rate

    def get_tx_stream_diagnostics(self):
        diagnostics = dict(self.jiang_0.get_stream_diagnostics())
        diagnostics.update({
            'configured_sample_rate_hz': int(self.sample_rate),
            'samples_per_symbol': int(self.sps),
            'iio_filter_mode': 'Auto',
            'hardware_sink': 'iio.fmcomms2_sink_fc32',
        })
        return diagnostics

    def set_symbol_rate(self, symbol_rate):
        self.symbol_rate = symbol_rate
        self.set_to_F_ganrao(self.BW_ganrao/2-self.symbol_rate)
        self.jiang_0.set_symbol_rate(self.symbol_rate)

    def get_BW_ganrao(self):
        return self.BW_ganrao

    def set_BW_ganrao(self, BW_ganrao):
        self.BW_ganrao = BW_ganrao
        self.set_to_F_ganrao(self.BW_ganrao/2-self.symbol_rate)
        self.iio_fmcomms2_sink_0_0.set_bandwidth(self.BW_ganrao)

    def get_to_F_ganrao(self):
        return self.to_F_ganrao

    def set_to_F_ganrao(self, to_F_ganrao):
        self.to_F_ganrao = to_F_ganrao
        self.set_sen_ganrao(2*math.pi*self.to_F_ganrao/self.sample_rate)

    def get_center_f(self):
        return self.center_f

    def set_center_f(self, center_f):
        self.center_f = center_f
        self.set_center_F_ganrao(self.center_f)
        if self.center_f in self._center_f_options:
            self._center_f_callback(self.center_f)

    def get_x(self):
        return self.x

    def set_x(self, x):
        self.x = x

    def get_sen_ganrao(self):
        return self.sen_ganrao

    def set_sen_ganrao(self, sen_ganrao):
        self.sen_ganrao = sen_ganrao
        self.digital_gfsk_mod_0_0.fmmod.set_sensitivity(self.sen_ganrao)

    def get_repeat(self):
        return self.repeat

    def set_repeat(self, repeat):
        self.repeat = repeat

    def get_data(self):
        return self.data

    def set_data(self, data):
        self.data = data
        self.jiang_0.set_payload_size(self.data)

    def get_payload_data(self):
        return self.jiang_0.get_payload_data()

    def set_payload_data(self, payload_data):
        self.data = len(payload_data)
        self.jiang_0.set_payload_data(payload_data)

    def get_command_cycle(self):
        return self.jiang_0.get_command_cycle()

    def set_command_cycle(self, command_cycle):
        self.jiang_0.set_command_cycle(command_cycle)

    def get_cmd_id(self):
        return self.cmd_id

    def set_cmd_id(self, cmd_id):
        self.cmd_id = cmd_id
        self.jiang_0.set_com_id(self.cmd_id)

    def get_center_F_ganrao(self):
        return self.center_F_ganrao

    def set_center_F_ganrao(self, center_F_ganrao):
        self.center_F_ganrao = center_F_ganrao
        self.iio_fmcomms2_sink_0_0.set_frequency(int(self.center_F_ganrao))
        self.qtgui_freq_sink_x_0_0.set_frequency_range((self.center_F_ganrao*0), self.sample_rate)
        self.qtgui_waterfall_sink_x_0_0_0.set_frequency_range((self.center_F_ganrao*0), self.sample_rate)

    def get_bw2(self):
        return self.bw2

    def set_bw2(self, bw2):
        self.bw2 = bw2

    def get_bw1(self):
        return self.bw1

    def set_bw1(self, bw1):
        self.bw1 = bw1

    def get_attend_gr(self):
        return self.attend_gr

    def set_attend_gr(self, attend_gr):
        self.attend_gr = attend_gr
        self.iio_fmcomms2_sink_0_0.set_attenuation(0, self.attend_gr)

    def get_access_xinxibo(self):
        return self.access_xinxibo

    def set_access_xinxibo(self, access_xinxibo):
        self.access_xinxibo = access_xinxibo

    def get_access_ganraobo(self):
        return self.access_ganraobo

    def set_access_ganraobo(self, access_ganraobo):
        self.access_ganraobo = access_ganraobo
        self.jiang_0.set_Access_Code(self.access_ganraobo)

    def set_access(self, access):
        self.jiang_0.set_Access_Code(access)

    def get_access(self):
        return self.jiang_0.get_Access_Code()

    def set_Period(self, Period):
        self.jiang_0.set_Period(Period)

    def get_Period(self):
        return self.jiang_0.get_Period()

    def get_SOF(self):
        return self.SOF

    def set_SOF(self, SOF):
        self.SOF = SOF

    def get_BT(self):
        return self.BT

    def set_BT(self, BT):
        self.BT = BT

    def set_iq_callback(self, callback):
        self.tx_iq_callback_sink.set_callback(callback)




def main(top_block_cls=RM, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
