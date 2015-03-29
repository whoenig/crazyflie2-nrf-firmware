import sys
import time
import numpy as np
import pyqtgraph as pg
from RTT import RTT

rtt = RTT()

p = pg.plot()
p.setYRange(-80, -30)
p.setLabel('bottom', 'Time')
p.setLabel('left', 'RSSI [dBM]')
p.addLegend()

x = [[],[],[],[],[],[],[],[]]
y = [[],[],[],[],[],[],[],[]]

curves = []
curves.append(p.plot(pen = "r", name = "4dBm"))
curves.append(p.plot(pen = "g", name = "0dBm"))
curves.append(p.plot(pen = "b", name = "-4dBm"))
curves.append(p.plot(pen = "c", name = "-8dBm"))
curves.append(p.plot(pen = "m", name = "-12dBm"))
curves.append(p.plot(pen = "y", name = "-16dBm"))
curves.append(p.plot(pen = (100,100,100), name = "-20dBm"))
curves.append(p.plot(pen = "w", name = "-30dBm"))

starttime = time.time()

while True:
	(power, rssi_count, rssi_sum) = rtt.get("<BBH")
	if power == 0x04:
		pos = 0
	elif power == 0x00:
		pos = 1
	elif power == 0xFC:
		pos = 2
	elif power == 0xF8:
		pos = 3
	elif power == 0xF4:
		pos = 4
	elif power == 0xF0:
		pos = 5
	elif power == 0xEC:
		pos = 6
	elif power == 0xD8:
		pos = 7

	print("{},{},{}".format(pos, rssi_count, rssi_sum))

	x[pos].append(time.time() - starttime)
	y[pos].append(-rssi_sum / rssi_count)

	if pos == 7:
		for i in range(0,8):
			if len(x[i]) > 100:
				del x[i][0:len(x[i])-100]
				del y[i][0:len(y[i])-100]
			curves[i].setData(x = x[i], y = y[i])

	pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
