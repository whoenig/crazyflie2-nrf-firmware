Crazyflie 2.0 NRF51 firmware
============================

This branch is used to collect RSSI data between two Crazyflies.
The receiver node needs to be connected via a J-Link EDU in order to read the results.
There are separate helper scripts to automate the data collection on the PC-side.

Usage
-----

0. Clean: `make clean`
0. Compile: `make CFMODE=RX` or `make CFMODE=TX`
0. Press the button on the CF for 3 seconds until the two blue LEDs start to blink.
0. Flash using Crazyradio: `make CFMODE=RX cload` or `make CFMODE=TX cload`
0. Connect receiving Crazyflie via nrf Debug adapter and J-Link EDU to PC
0. Run `./JLinkExe -device NRF51 -speed 4000 -if SWD` on the PC
0. Run `python3 channelscanRTT.py` from the scripts folder
0. Turn transmitting Crazyflie on

Modes
-----

0. None

This mode measures RSSI continously (fixed channel, power, and datarate). Results are averaged and sent via RTT to the PC.

```
make SCAN_MODE=NONE CF_CHANNEL=100 CF_POWER=Neg20dBm CF_DATARATE=250K CFMODE=[TX,RX] [clean, cload]
python3 scanRTT.py
```

0. Channelscan

This modes measures RSSI vs. channel (fixed power, and datarate).

```
make SCAN_MODE=NONE CF_POWER=Neg20dBm CF_DATARATE=250K CFMODE=[TX,RX] [clean, cload]
python3 channelscanRTT.py
```

0. Powerscan

This modes measures RSSI vs. power (fixed channel, and datarate).

```
make SCAN_MODE=NONE CHANNEL=100 CF_DATARATE=250K CFMODE=[TX,RX] [clean, cload]
python3 powerscanRTT.py
```

0. Dataratescan

This modes measures RSSI vs. datarate (fixed channel, and power).

```
make SCAN_MODE=NONE CHANNEL=100 CF_POWER=Neg20dBm CFMODE=[TX,RX] [clean, cload]
python3 dataratescanRTT.py
```

Setup J-Link
------------

We use a J-Link EDU for debugging.

* Download software and documentation from SEGGER: https://www.segger.com/jlink-software.html
* Follow the instructions (In case of V4.98 it was enough to extract the archive, add a udev-rule, and reboot the system)
* Run `./JLinkExe -device NRF51 -speed 4000 -if SWD` in order to test the connection

Debugging
---------

We use openocd for debugging. However, the SEGGER gdbserver might work as well.
The openocd version of Ubuntu is too old (0.7). The latest released version is too old as well (0.8).
Instead, install the latest version:

```
git clone https://github.com/ntfreak/openocd.git
cd openocd
./bootstrap
./configure
make
sudo make install
```

Now you can debug using `make openocd` and `make gdb`.
It is also possible to connect to openocd using Eclipse or Qt Creator.

Real-Time Terminal
------------------

SEGGERs Real-Time Terminal (RTT) allows to use printf where the data is communicated via JTAG.
It is faster than traditional methods (UART/SWO): https://www.segger.com/jlink-real-time-terminal.html.
The code already uses RTT. To see the messages on the client side, do the following:

```
./JLinkExe -device NRF51 -speed 4000 -if SWD
./JLinkRTTClient (or use telnet or use API)
```
