Crazyflie 2.0 NRF51 firmware
============================

This branch sends continuously packages via the radio in PTX mode.

Setup J-Link
------------

We use a J-Link EDU for debugging.

* Download software and documentation from SEGGOR: https://www.segger.com/jlink-software.html
* Follow the instructions (In case of V4.96l it was enough to extract the archive, add a udev-rule, and reboot the system)
* Run `./JLinkExe -device NRF51822` in order to test the connection

Debugging
---------

We use openocd for debugging. However, the SEGGOR gdbserver might work as well.
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

Now you can debug using `make openocd BLE=0` and `make gdb BLE=0`.
It is also possible to connect to openocd using Eclipse or Qt Creator.



Original Docs
-------------

Source code of the firmware running in the Crazyflie 2.0 nRF51822.
This microcontroller have a couple of roles:
 - Power management (ON/OFF logic and battery handling)
 - Radio communication
   - Enhanced Shockburst compatible with Crazyradio (PA)
   - Bluetooth low energy using the Nordic Semiconductor S110 stack
 - One-wire memory access

Compiling with bluetooth support currently requires the nRF51_SDK and
S110 packages. These can be downloaded from the Nordic semiconductor
website. See s110/readme and nrf51_sdk/readme for exact version.

License
-------

Most of the code is licensed under LGPL-3.0.

Some files under src/ble/ are modified from Nordic semiconductor examples.

Downloading the nrf51_sdk and nordic S110 softdevice require to aquire one
of the Nordic Semiconductor development kit. Discussion is in progress
with Nordic to solve this situation.

Compiling
---------

To compile arm-none-eabi- tools from https://launchpad.net/gcc-arm-embedded
should be in the path.

Compilation options can be saved in config.mk. Main targets:

```
make                 # Make with BLE support
make BLE=0           # Make without BLE support
make BLE=0 S110=0    # Make without BLE and without Softdevice in flash (see bellow)

make flash           # Flash firmware with jtag
make factory_reset   # Erase device and flash softdevice, bootloaders, and firmware
```

Architecture
------------

When running without softdevice (S110=0) the firmware is loaded at the
beginning of the flash and is running alone in the CPU.

When running with Softdevive (S110=1) independent of if BLE is activated
or not, the flash is filled as follow:
```
+--------------+ 256k
|     MBS      |    Write protected
+--------------+ 252k
|  Bootloader  |
+--------------+ 232k
|              |
|              |
|              |
|              |
|              |
|  Firmware    |
+--------------+ 88K
|              |
|              |
|              |
|              |
|              |
|              |
|  Softdevice  |
+--------------+ 4K
|     MBR      |    Write protected
+--------------+ 0
```

 - **MBR** Softdevice Master Boot Record.
 - **SoftDevice** S110 Bluetooth stack
 - **Firmware** This firmware
 - **Bootloader** Bluetooth/Shockburst bootloader
 - **MBS** Master Boot Switch

Boot sequence:
```
 MBR ----> MBS ----> Bootloader ----> Firmware
```

The MBR is part of the Softdevice. It boots the CPU and jump to MBS.
The MBR contains methods to start the Softdevice and can flash softdevice
and bootloader.

The MBS handles the ON/OFF button and comunicate the duration of the press to
the bootloader so that the bootloader knows what to boot. The reason for the
MBS is to allow updating the bootloader over the air while still having a
write-protected piece of software that can start the STM32 in USB DFU mode
for recovery (the STM32 has access to the NRF51 SWD programming port). The boot
switch is as follow:

| Press time      | Blue LED state | Program booted                               |
| --------------- | -------------- | -------------------------------------------- |
| Short           | Still          | Firmware                                     |
| Long (>3s)      | Slow blink     | Bootloader                                   |
| Very long (>5s) | Fast blink     | Stays in MBS and power STM32 in USB DFU mode |

The bootloader, if selected, starts the STM32 in bootloader mode and initialize
both BLE and Shockburst (ESB) radio. It can flash everything but MBR and MBS.
It also acts as a bridge to the STM32 bootloader.

If not selected, the bootloader jumps to the firmware.
