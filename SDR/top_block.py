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
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import pmt
import wx


class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 32000
        self.constellation = constellation = 8
        self.CODE = CODE = '0x1011011100010110111000'

        ##################################################
        # Blocks
        ##################################################
        self.message_source = analog.sig_source_c(samp_rate, analog.GR_SIN_WAVE, 3.1e9, 1, 0)
        self.input_file1 = blocks.file_source(gr.sizeof_char*1, 'C:\\Users\\camil\\Desktop\\11Bark.txt', True)
        self.input_file1.set_begin_tag(pmt.PMT_NIL)
        self.digital_psk_mod_0 = digital.psk.psk_mod(
          constellation_points=constellation,
          mod_code="none",
          differential=False,
          samples_per_symbol=16,
          excess_bw=0.35,
          verbose=False,
          log=False,
          )
        self.digital_psk_demod_0 = digital.psk.psk_demod(
          constellation_points=constellation,
          differential=False,
          samples_per_symbol=16,
          excess_bw=0.35,
          phase_bw=6.28/100.0,
          timing_bw=6.28/100.0,
          mod_code="none",
          verbose=False,
          log=False,
          )
        self.digital_correlate_access_code_xx_ts_0 = digital.correlate_access_code_bb_ts(CODE,
          1, '')
        self.digital_clock_recovery_mm_xx_0 = digital.clock_recovery_mm_cc(16*samp_rate*(1+0.0), 0.25*0.175*0.175, 0.5, 0.175, 0.005)
        self.blocks_vector_sink_x_0 = blocks.vector_sink_b(1, 1024)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_file_sink_0_0_0 = blocks.file_sink(gr.sizeof_char*1, 'C:\\Users\\camil\\Desktop\\TestDemod', False)
        self.blocks_file_sink_0_0_0.set_unbuffered(False)
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_char*1, 'C:\\Users\\camil\\Desktop\\TestCorrel', False)
        self.blocks_file_sink_0_0.set_unbuffered(False)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.digital_clock_recovery_mm_xx_0, 0))
        self.connect((self.digital_clock_recovery_mm_xx_0, 0), (self.digital_psk_demod_0, 0))
        self.connect((self.digital_correlate_access_code_xx_ts_0, 0), (self.blocks_file_sink_0_0, 0))
        self.connect((self.digital_correlate_access_code_xx_ts_0, 0), (self.blocks_vector_sink_x_0, 0))
        self.connect((self.digital_psk_demod_0, 0), (self.blocks_file_sink_0_0_0, 0))
        self.connect((self.digital_psk_demod_0, 0), (self.digital_correlate_access_code_xx_ts_0, 0))
        self.connect((self.digital_psk_mod_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.input_file1, 0), (self.digital_psk_mod_0, 0))
        self.connect((self.message_source, 0), (self.blocks_multiply_xx_0, 0))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.message_source.set_sampling_freq(self.samp_rate)
        self.digital_clock_recovery_mm_xx_0.set_omega(16*self.samp_rate*(1+0.0))
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_constellation(self):
        return self.constellation

    def set_constellation(self, constellation):
        self.constellation = constellation

    def get_CODE(self):
        return self.CODE

    def set_CODE(self, CODE):
        self.CODE = CODE


def main(top_block_cls=top_block, options=None):

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
