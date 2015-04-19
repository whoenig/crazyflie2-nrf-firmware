from crazyradio import Crazyradio
import struct

TOTALNUM = 2

with Crazyradio(0) as radio:
	radio.set_data_rate(Crazyradio.DR_250KPS)
	radio.set_channel(100)
	radio.set_address([0xE7, 0xE7, 0xE7, 0xE7, 0xE8])
	# for j in range(0, 10):
	res = radio.send_packet([1])
	print(res)
	radio.set_radio_mode(Crazyradio.MODE_PRX)
	for j in range(0, TOTALNUM * TOTALNUM):
		res = radio.receive()
		print(res)
		receiverId, senderId, rssi_count, rssi_sum = struct.unpack("<BBII", res)
		if rssi_count > 0:
			print("{}<-{}: {} (count: {})".format(receiverId, senderId, rssi_sum / rssi_count, rssi_count))
