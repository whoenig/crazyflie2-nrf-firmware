/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#include <nrf.h>

#ifdef BLE
#include <nrf_soc.h>
#endif

#include <stdio.h>
#include <string.h>

#if CFMODE == 2 || CFMODE == 3
  #include "esb.h"
#endif
#include "led.h"
#include "button.h"
#include "pm.h"
#include "pinout.h"
#include "systick.h"

#include "nrf_gpio.h"

#include "SEGGER_RTT.h"

#ifdef BLE
#include "ble_crazyflies.h"
#endif

extern void  initialise_monitor_handles(void);
extern int ble_init(void);

#ifndef SEMIHOSTING
#define printf(...)
#endif

static void mainloop(void);

#if BLE==0
#undef BLE
#endif

static bool boottedFromBootloader;

#if CFMODE == 2
static unsigned int channel;
static int count = 0;
static int channelSwitchAckTime;
static EsbDatarate datarate;
static int txpower;
void packetReceivedHandler(EsbPacket* received, EsbPacket* ack)
{
  // int i;
  count = received->data[1];

  ack->size = 3;
  ack->data[0] = channel;
  ack->data[1] = count;
  ack->data[2] = received->rssi;
  SEGGER_RTT_Write(0, (const char*)&(ack->data[0]), 3);
  // SEGGER_RTT_printf(0, "R");

  if (count == 20)// && channel == rx_payload.data[0])
  {
    // SEGGER_RTT_printf(0, "%d,%d,%d\n", channel, count, received->rssi);
    // SEGGER_RTT_printf(0, "CSAT");
    channelSwitchAckTime = systickGetTick();
  }

}
#endif

#if CFMODE == 3
static unsigned int channel;
static int count = 0;
static EsbDatarate datarate;
static int txpower;
#endif

int main()
{
  systickInit();

#ifdef BLE
  ble_init();
#else
  NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);
#endif

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  if ((NRF_POWER->GPREGRET & 0x80) && ((NRF_POWER->GPREGRET&(0x3<<1))==0)) {
    buttonInit(buttonShortPress);
  } else {
    buttonInit(buttonIdle);
  }

  if  (NRF_POWER->GPREGRET & 0x20) {
    boottedFromBootloader = true;
    NRF_POWER->GPREGRET &= ~0x20;
  }

  pmInit();

  if ((NRF_POWER->GPREGRET&0x01) == 0) {
		  pmSetState(pmSysRunning);
  }

  LED_ON();


  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  #if CFMODE == 2 || CFMODE == 3 //Bitcraze RX-mode & Bitcraze TX Mode
    channel = 0;
    count = 0;
    datarate = esbDatarate250K;

    esbSetDatarate(datarate);
    esbSetChannel(channel);
    esbSetTxPower(txpower);
    esbInit();
  #endif

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  #if CFMODE == 2 // Bitcraze RX Mode
    esbSetPacketReceivedHandler(packetReceivedHandler);
    esbStartRx();
  #endif

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

void delayms(int delay)
{
  int starttime = systickGetTick();
  while (systickGetTick() < starttime + delay);
}

void mainloop()
{
#if CFMODE == 3
  EsbPacket packet;
  EsbPacket* ack;
  packet.data[0] = 0xCA;
  packet.data[1] = 0xFE;
  packet.size = 1;
#endif

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);

  while(1)
  {
    PmState state = pmGetState();
    if (state != pmSysOff)
    {
      LED_ON();
      nrf_gpio_pin_set(RADIO_PAEN_PIN);

      #if CFMODE == 2 // Bitcraze RX Mode
        if (count == 20 && systickGetTick() >= channelSwitchAckTime + 50)
        {
          count = 0;
          channel += 1;
          if (channel == 126) {
            // switch (datarate)
            // {
            //   case esbDatarate250K:
            //     datarate = esbDatarate1M;
            //     break;
            //   case esbDatarate1M:
            //     datarate = esbDatarate2M;
            //     break;
            //   case esbDatarate2M:
            //     datarate = esbDatarate250K;
            //     switch(txpower)
            //     {
            //     case RADIO_TXPOWER_TXPOWER_Pos4dBm:
            //       txpower = RADIO_TXPOWER_TXPOWER_0dBm;
            //       break;
            //     case RADIO_TXPOWER_TXPOWER_0dBm:
            //       txpower = RADIO_TXPOWER_TXPOWER_Neg4dBm;
            //       break;
            //     }
            //     esbSetTxPower(txpower);
            //     break;
            // }
            // esbSetDatarate(datarate);

            switch(txpower)
            {
            case RADIO_TXPOWER_TXPOWER_Pos4dBm:
              txpower = RADIO_TXPOWER_TXPOWER_0dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_0dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg4dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg4dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg8dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg8dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg12dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg12dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg16dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg16dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg20dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg20dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Neg30dBm;
              break;
            case RADIO_TXPOWER_TXPOWER_Neg30dBm:
              txpower = RADIO_TXPOWER_TXPOWER_Pos4dBm;
              break;
            }
            esbSetTxPower(txpower);

            channel = 0;
          }
          // esbReset();
          // esbStartRx();
          esbStopRx();
          // delayms(250);
          esbSetChannel(channel);
          // delayms(250);
          esbStartRx();
          // SEGGER_RTT_printf(0, "Channel: %d\n", channel);
        }
        //if (esbIsRxPacket())
        //{
        //  EsbPacket* packet = esbGetRxPacket();
        //  esbReleaseRxPacket(packet);
        //}
      #endif
      #if CFMODE == 3 // Bitcraze TX Mode
        packet.pid = (packet.pid + 1) % 4;
        packet.size = 16;
        packet.data[0] = channel;
        packet.data[1] = count;
        ack = esbSendPacket(&packet);
        if (ack && ack->size)
        {
          if (ack->data[1] == count)
          {
            if (count == 20)
            {
              channel = ack->data[0] + 1;
              if (channel == 126) {
                // switch (datarate)
                // {
                //   case esbDatarate250K:
                //     datarate = esbDatarate1M;
                //     break;
                //   case esbDatarate1M:
                //     datarate = esbDatarate2M;
                //     break;
                //   case esbDatarate2M:
                //     datarate = esbDatarate250K;
                //     break;
                // }
                // esbSetDatarate(datarate);

                switch(txpower)
                {
                case RADIO_TXPOWER_TXPOWER_Pos4dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_0dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_0dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg4dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg4dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg8dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg8dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg12dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg12dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg16dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg16dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg20dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg20dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Neg30dBm;
                  break;
                case RADIO_TXPOWER_TXPOWER_Neg30dBm:
                  txpower = RADIO_TXPOWER_TXPOWER_Pos4dBm;
                  break;
                }
                esbSetTxPower(txpower);

                channel = 0;
              }
              esbSetChannel(channel);
              count = 0;
            }
            else
            {
              count += 1;
            }
          }
          //SEGGER_RTT_printf(0, "%d\n", ack->data[1]);
        }
        // else
        // {
        //   SEGGER_RTT_printf(0, "-");
        // }
      #endif
    }
    else
    {
      LED_OFF();
    }

    // Button event handling
    ButtonEvent be = buttonGetState();
    bool usbConnected = pmUSBPower();
    if ((pmGetState() != pmSysOff) && be && !usbConnected)
    {
      pmSetState(pmAllOff);
      /*swdInit();
      swdTest();*/
    }
    else if ((pmGetState() != pmSysOff) && be && usbConnected)
    {
    	//pmSetState(pmSysOff);
      pmSetState(pmAllOff);
        /*swdInit();
        swdTest();*/
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonShortPress))
    {
      //Normal boot
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && boottedFromBootloader)
    {
      //Normal boot after bootloader
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonLongPress))
    {
      //stm bootloader
      pmSysBootloader(true);
      pmSetState(pmSysRunning);
    }
    boottedFromBootloader = false;

    // processes loop
    buttonProcess();
    pmProcess();
    //owRun();       //TODO!
  }
}

