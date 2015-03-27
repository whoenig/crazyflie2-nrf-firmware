#!/usr/bin/python2.7

# This script contains a helper class to use SEGGERs Real-Time-Terminal (RTT).
# Make sure to execute "./JLinkExe -device NRF51 -speed 4000 -if SWD" in another terminal to get RTT.

import socket
import struct
import time

class RTT:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(("localhost", 19021))
        # self.data = bytearray([])
        dummy = self.recv(106) # ignore header
        print(dummy)

    def recv(self, num_bytes):
        data = bytearray([])
        while len(data) < num_bytes:
            data += self.s.recv(num_bytes - len(data))
        return data

    # def recv(self, num_bytes):
    #     print("have {} bytes".format(len(self.data)))
    #     while len(self.data) < num_bytes:
    #         self.data += self.s.recv(1024)
    #         time.sleep(0.01)
    #     result = self.data[0:num_bytes]
    #     self.data = self.data[num_bytes:]
    #     return result

    def get(self, fmt):
        size = struct.calcsize(fmt)
        data = self.recv(size)
        return struct.unpack(fmt, data)
