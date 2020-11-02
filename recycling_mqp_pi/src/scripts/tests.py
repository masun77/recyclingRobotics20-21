#test file for teensy comms

import unittest
import recycling_mqp_pi.src.scripts.teensy_comms

class Tester(unittest.TestCase):
    def testDecode(self):
        return 0


    def testEncode(self):

        return 0

    def encodeHelper(self):
        #todo create dummy packets to test decoding teensy packets
        # same for dummy packets to test decoding the encoded pi packets
        return