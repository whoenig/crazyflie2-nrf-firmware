Crazyflie 2.0 NRF51 firmware
============================

This branch is used to enable relative localization of a stationary Crazyflie swarm based on RSSI readings.

Usage
-----

```
make CF_CHANNEL=100 CF_POWER=Neg20dBm CF_DATARATE=250K CF_TOTALNUM=2 CF_ID=0 [clean, cload]
```

0. Clean: `make clean`
0. Compile: see above; every CF needs to have its own ID (requires clean build!)
0. Press the button on the CF for 3 seconds until the two blue LEDs start to blink.
0. Flash using Crazyradio: `make cload`
0. Run `python3 clocksync.py` to synchronize clock and start localization

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
