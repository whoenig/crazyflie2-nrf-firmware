from crazyradio import Crazyradio
import struct

with Crazyradio(0) as radio:
	radio.set_data_rate(Crazyradio.DR_250KPS)
	radio.set_channel(100)
	radio.set_address([0xE7, 0xE7, 0xE7, 0xE7, 0xE8])
	for j in range(0, 10):
		res = radio.send_packet([1])
		print(res)
