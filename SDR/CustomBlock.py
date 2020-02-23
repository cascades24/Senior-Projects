"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, frequency=3.1e9, expected='10110100111'):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self.frequency=frequency
            self.name='Poll Check'# will show up in G
            self.expected=expected

        )


    def work(self, input_stream, output_items):
        """example: multiply with constant"""
        corr[0][:] = np.correlate(input_stream[0], self.expected)
        IDX=find(corr==max(corr))
        return IDX*1/(self.frequency)
