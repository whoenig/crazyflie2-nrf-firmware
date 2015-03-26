import sys
import time
import numpy as np
import pyqtgraph as pg
from RTT import RTT

rtt = RTT()

p = pg.plot()
p.setXRange(0, 125)
p.setLabel('bottom', 'Channel')
p.setLabel('left', 'RSSI [-dBM]')
p.addLegend()

# for txp in range(0, 2):
# for dr in range(0, 3):
# 	if dr == 0:
# 		color = "r"
# 		name = "250K" + str(txp)
# 	elif dr == 1:
# 		color = "g"
# 		name = "1M" + str(txp)
# 	elif dr == 2:
# 		color = "b"
# 		name = "2M" + str(txp)

for txp in range(0, 8):
	if txp == 0:
		color = "r"
		name = "+4dBm"
	elif txp == 1:
		color = "g"
		name = "0dBm"
	elif txp == 2:
		color = "b"
		name = "-4dBm"
	elif txp == 3:
		color = "b"
		name = "-8dBm"
	elif txp == 4:
		color = "b"
		name = "-12dBm"
	elif txp == 5:
		color = "b"
		name = "-16dBm"
	elif txp == 6:
		color = "b"
		name = "-20dBm"
	elif txp == 7:
		color = "b"
		name = "-30dBm"

	x = []
	y = []
	curve = p.plot(pen = color, name = name)
	curve.setData(x = x, y = y)
	started = False
	while True:
		result = rtt.get("<BBB")
		channel = result[0]
		count = result[1]
		print("{},{},{}".format(channel, result[1], result[2]))

		x.append(channel)
		y.append(result[2])

		if count == 0:
			curve.setData(x = x, y = y)

		if count == 0 and channel == 0:
			started = True

		if started and count == 20 and channel == 125:
			break
		pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
