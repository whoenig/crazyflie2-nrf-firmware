#Put your personal build config in config.mk and DO NOT COMMIT IT!
-include config.mk

CLOAD_SCRIPT ?= ../crazyflie-clients-python/bin/cfloader

S110 ?= 1     # SoftDevice flashed or not
CFMODE ?= RX  # RX or TX
SCAN_MODE ?= CHANNEL # NONE, CHANNEL, POWER, DATARATE
CF_CHANNEL ?= 0 # 0 - 125
CF_DATARATE ?= 250K # 250K, 1M, 2M
CF_POWER ?= Pos4dBm # Pos4dBm, 0dBm, Neg4dBm, Neg8dBm, Neg12dBm, Neg16dBm, Neg20dBm, Neg30dBm

CROSS_COMPILE?=arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
GDB=$(CROSS_COMPILE)gdb

OPENOCD           ?= openocd
OPENOCD_DIR       ?=
OPENOCD_INTERFACE ?= $(OPENOCD_DIR)interface/jlink.cfg
OPENOCD_TARGET    ?= target/nrf51.cfg


NRF51_SDK ?= nrf51_sdk/nrf51822
NRF_S110 ?= s110

INCLUDES= -I Include -I Include/gcc -Iinterface

PERSONAL_DEFINES ?=

PROCESSOR = -mcpu=cortex-m0 -mthumb
NRF= -DNRF51
PROGRAM=cf2_nrf

CFLAGS=$(PROCESSOR) $(NRF) $(PERSONAL_DEFINES) $(INCLUDES) -g3 -O0 -Wall# -fdata-sections
CFLAGS+= -fsingle-precision-constant -ffast-math
# --specs=nano.specs -flto
ASFLAGS=$(PROCESSOR)
LDFLAGS=$(PROCESSOR) -O0 --specs=nano.specs -Wl,-Map=$(PROGRAM).map# -Wl,--gc-sections
ifdef SEMIHOSTING
LDFLAGS+= --specs=rdimon.specs -lc -lrdimon
CFLAGS+= -DSEMIHOSTING
endif

ifeq ($(strip $(S110)), 1)
LDFLAGS += -T gcc_nrf51_s110_xxaa.ld
CFLAGS += -DS110=1
else
LDFLAGS += -T gcc_nrf51_blank_xxaa.ld
endif

CFLAGS += -DCFMODE=CFMODE_$(strip $(CFMODE))
CFLAGS += -DSCAN_MODE=SCAN_MODE_$(strip $(SCAN_MODE))

CFLAGS += -DCF_CHANNEL=$(strip $(CF_CHANNEL))
CFLAGS += -DCF_DATARATE=esbDatarate$(strip $(CF_DATARATE))
CFLAGS += -DCF_POWER=RADIO_TXPOWER_TXPOWER_$(strip $(CF_POWER))


OBJS += src/main.o gcc_startup_nrf51.o system_nrf51.o \
        src/systick.o src/button.o src/uart.o src/syslink.o \
        src/esb.o \
        src/SEGGER_RTT.o src/SEGGER_RTT_printf.o

all: $(PROGRAM).elf $(PROGRAM).bin $(PROGRAM).hex
	$(SIZE) $(PROGRAM).elf
ifeq ($(strip $(S110)),1)
	@echo "S110 Activated"
else
	@echo "S110 Disabled"
endif

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) $^ -O ihex $@

$(PROGRAM).bin: $(PROGRAM).elf
	$(OBJCOPY) $^ -O binary $@

$(PROGRAM).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^

clean:
	rm -f $(PROGRAM).bin $(PROGRAM).elf $(OBJS)


## Flash and debug targets

flash: $(PROGRAM).hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROGRAM).hex" -c "verify_image $(PROGRAM).hex" \
                 -c "reset run" -c shutdown

flash_s110: $(NRF_S110)/s110_nrf51822_7.0.0_softdevice.hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "nrf51 mass_erase" \
                 -c "flash write_image erase s110/s110_nrf51822_7.0.0_softdevice.hex" \
                 -c "reset run" -c shutdown

flash_mbs: bootloaders/nrf_mbs_v1.0.hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $^" -c "verify_image $^" -c "reset halt" \
	               -c "mww 0x4001e504 0x01" -c "mww 0x10001014 0x3F000" \
	               -c "reset run" -c shutdown

flash_cload: bootloaders/cload_nrf_v1.0.hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $^" -c "verify_image $^" -c "reset halt" \
	               -c "mww 0x4001e504 0x01" -c "mww 0x10001014 0x3F000" \
	               -c "mww 0x4001e504 0x01" -c "mww 0x10001080 0x3A000" -c "reset run" -c shutdown


mass_erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "nrf51 mass_erase" -c shutdown

reset:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets \
	               -c reset -c shutdown

openocd: $(PROGRAM).elf
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -c "transport select swd" -f $(OPENOCD_TARGET) -c init -c targets


semihosting: $(PROGRAM).elf
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c reset -c "arm semihosting enable" -c reset

gdb: $(PROGRAM).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

flash_jlink:
	JLinkExe -if swd -device NRF51822 flash.jlink

cload: $(PROGRAM).bin
ifeq ($(strip $(S110)), 1)
	$(CLOAD_SCRIPT) flash $(PROGRAM).bin nrf51-fw
else
	@echo "Only S110 build can be bootloaded. Launch build and cload with S110=1"
endif


factory_reset:
	make mass_erase
ifeq ($(strip $(S110)),1)
	make flash_s110
	make flash_mbs
	make flash_cload
endif
	make flash
