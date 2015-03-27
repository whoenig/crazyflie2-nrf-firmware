import sys
import time
import numpy as np
import pyqtgraph as pg
from RTT import RTT

rtt = RTT()

p = pg.plot()
p.setXRange(0, 125)
p.setLabel('bottom', 'Channel')
p.setLabel('left', 'RSSI [dBM]')

x = []
y = []
curve = p.plot()
curve.setData(x = x, y = y)
started = False
while True:
	(channel, count, rssi_count, rssi_sum) = rtt.get("<BBBH")
	print("{},{},{},{}".format(channel, count, rssi_count, rssi_sum))

	x.append(channel)
	y.append(-rssi_sum / rssi_count)

	if count == 0:
		curve.setData(x = x, y = y)

	if count == 0 and channel == 0:
		started = True

	if started and count == 10 and channel == 125:
		break
	pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
