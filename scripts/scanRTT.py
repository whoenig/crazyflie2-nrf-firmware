import sys
import time
import numpy as np
import pyqtgraph as pg
from RTT import RTT

rtt = RTT()

p = pg.plot()
# p.setXRange(0, 125)
p.setYRange(-80, -30)
p.setLabel('bottom', 'Time')
p.setLabel('left', 'RSSI [dBM]')

x = []
y = []

curve = p.plot()

starttime = time.time()

while True:
	(rssi_count, rssi_sum) = rtt.get("<II")
	print("{},{}".format(rssi_count, rssi_sum))

	x.append(time.time() - starttime)
	y.append(-rssi_sum / rssi_count)

	if len(x) > 100:
		del x[0:len(x)-100]
		del y[0:len(y)-100]
	curve.setData(x = x, y = y)

	pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
