#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# GNU Radio version: 3.7.13.5
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from gnuradio import analog
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from gnuradio.wxgui import forms
from gnuradio.wxgui import scopesink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import wx


class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")

        ##################################################
        # Variables
        ##################################################
        self.choose_freq = choose_freq = 100
        self.mid_freq = mid_freq = choose_freq
        self.high_freq = high_freq = mid_freq*2
        self.width_freq = width_freq = 0x010110
        self.samp_rate = samp_rate = 2*high_freq
        self.message = message = 0x010110
        self.low_freq = low_freq = mid_freq/2
        self.delay_tag = delay_tag = 0x010110
        self.Reply_Code = Reply_Code = 0x010110

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_scopesink2_0 = scopesink2.scope_sink_c(
        	self.GetWin(),
        	title='Scope Plot',
        	sample_rate=samp_rate,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=1,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label='Counts',
        )
        self.Add(self.wxgui_scopesink2_0.win)
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=0,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title='FFT Plot',
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_0.win)
        _choose_freq_sizer = wx.BoxSizer(wx.VERTICAL)
        self._choose_freq_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_choose_freq_sizer,
        	value=self.choose_freq,
        	callback=self.set_choose_freq,
        	label='choose_freq',
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._choose_freq_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_choose_freq_sizer,
        	value=self.choose_freq,
        	callback=self.set_choose_freq,
        	minimum=10,
        	maximum=500,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_choose_freq_sizer)
        self.band_pass_filter_0 = filter.fir_filter_ccf(1, firdes.band_pass(
        	10, samp_rate, low_freq, high_freq, width_freq, firdes.WIN_HAMMING, 6.76))
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, mid_freq, 1, 0)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.band_pass_filter_0, 0))
        self.connect((self.band_pass_filter_0, 0), (self.wxgui_fftsink2_0, 0))
        self.connect((self.band_pass_filter_0, 0), (self.wxgui_scopesink2_0, 0))

    def get_choose_freq(self):
        return self.choose_freq

    def set_choose_freq(self, choose_freq):
        self.choose_freq = choose_freq
        self.set_mid_freq(self.choose_freq)
        self._choose_freq_slider.set_value(self.choose_freq)
        self._choose_freq_text_box.set_value(self.choose_freq)

    def get_mid_freq(self):
        return self.mid_freq

    def set_mid_freq(self, mid_freq):
        self.mid_freq = mid_freq
        self.set_low_freq(self.mid_freq/2)
        self.set_high_freq(self.mid_freq*2)
        self.analog_sig_source_x_0.set_frequency(self.mid_freq)

    def get_high_freq(self):
        return self.high_freq

    def set_high_freq(self, high_freq):
        self.high_freq = high_freq
        self.set_samp_rate(2*self.high_freq)
        self.band_pass_filter_0.set_taps(firdes.band_pass(10, self.samp_rate, self.low_freq, self.high_freq, self.width_freq, firdes.WIN_HAMMING, 6.76))

    def get_width_freq(self):
        return self.width_freq

    def set_width_freq(self, width_freq):
        self.width_freq = width_freq
        self.band_pass_filter_0.set_taps(firdes.band_pass(10, self.samp_rate, self.low_freq, self.high_freq, self.width_freq, firdes.WIN_HAMMING, 6.76))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.wxgui_scopesink2_0.set_sample_rate(self.samp_rate)
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)
        self.band_pass_filter_0.set_taps(firdes.band_pass(10, self.samp_rate, self.low_freq, self.high_freq, self.width_freq, firdes.WIN_HAMMING, 6.76))
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)

    def get_message(self):
        return self.message

    def set_message(self, message):
        self.message = message

    def get_low_freq(self):
        return self.low_freq

    def set_low_freq(self, low_freq):
        self.low_freq = low_freq
        self.band_pass_filter_0.set_taps(firdes.band_pass(10, self.samp_rate, self.low_freq, self.high_freq, self.width_freq, firdes.WIN_HAMMING, 6.76))

    def get_delay_tag(self):
        return self.delay_tag

    def set_delay_tag(self, delay_tag):
        self.delay_tag = delay_tag

    def get_Reply_Code(self):
        return self.Reply_Code

    def set_Reply_Code(self, Reply_Code):
        self.Reply_Code = Reply_Code


def main(top_block_cls=top_block, options=None):

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
