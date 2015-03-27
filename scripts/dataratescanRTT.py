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
p.addLegend()

x = [[],[],[]]
y = [[],[],[]]

curves = []
curves.append(p.plot(pen = "r", name = "250K"))
curves.append(p.plot(pen = "g", name = "1M"))
curves.append(p.plot(pen = "b", name = "2M"))

starttime = time.time()

# curve.setData(x = x, y = y)
while True:
	(datarate, count, rssi) = rtt.get("<BBB")
	print("{},{},{}".format(datarate, count, rssi))

	# x[datarate].append(datarate)
	x[datarate].append(time.time() - starttime)
	y[datarate].append(-rssi)

	if count == 0:
		for i in range(0,3):
			if len(x[i]) > 500:
				del x[i][0:len(x[i])-500]
				del y[i][0:len(y[i])-500]
			curves[i].setData(x = x[i], y = y[i])

	# if count == 10 and datarate == 2:
		# break
	pg.QtGui.QApplication.processEvents()

pg.QtGui.QApplication.exec_()
