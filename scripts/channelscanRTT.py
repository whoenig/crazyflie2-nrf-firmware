import sys
import time
import numpy as np
import pyqtgraph as pg
from RTT import RTT

rtt = RTT()

p = pg.plot()
p.setXRange(0, 125)
p.setYRange(-80, -30)
p.setLabel('bottom', 'Channel')
p.setLabel('left', 'RSSI [dBM]')

x = []
y = []
curve = p.plot()
while True:
	(channel, rssi_count, rssi_sum) = rtt.get("<BBH")
	print("{},{},{}".format(channel, rssi_count, rssi_sum))

	x.append(channel)
	y.append(-rssi_sum / rssi_count)

	curve.setData(x = x, y = y)

	if channel == 125:
		del x[:]
		del y[:]
	pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
