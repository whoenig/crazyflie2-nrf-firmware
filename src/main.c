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

#include <stdio.h>
#include <string.h>

#include "esb.h"

#include "led.h"
#include "button.h"
#include "pinout.h"
#include "systick.h"

#include "nrf_gpio.h"

#include "SEGGER_RTT.h"

#define CFMODE_RX 0
#define CFMODE_TX 1

#define SCAN_MODE_NONE     0
#define SCAN_MODE_CHANNEL  1
#define SCAN_MODE_POWER    2
#define SCAN_MODE_DATARATE 3

static void mainloop(void);

#if CFMODE == CFMODE_RX
  static unsigned int channel;
  static EsbDatarate datarate;
  static int txpower;

  #if SCAN_MODE == SCAN_MODE_CHANNEL
    static int channelSwitchAckTime = 0;
  #endif
  #if SCAN_MODE == SCAN_MODE_DATARATE
    static int datarateSwitchAckTime = 0;
  #endif
  #if SCAN_MODE == SCAN_MODE_NONE
    static uint32_t rssi_sum;
    static uint32_t rssi_count;
  #endif
  void packetReceivedHandler(EsbPacket* received, EsbPacket* ack)
  {
    #if SCAN_MODE == SCAN_MODE_CHANNEL
      ack->data[0] = channel;
      channelSwitchAckTime = systickGetTick();
    #endif
    #if SCAN_MODE == SCAN_MODE_POWER
      ack->data[0] = txpower;
    #endif
    #if SCAN_MODE == SCAN_MODE_DATARATE
      ack->data[0] = datarate;
      datarateSwitchAckTime = systickGetTick();
    #endif

    #if SCAN_MODE == SCAN_MODE_NONE
      ack->size = 4;
      ack->data[1] = received->rssi_count;
      ack->data[2] = received->rssi_sum & 0xFF;
      ack->data[3] = (received->rssi_sum >> 8) & 0xFF;
      rssi_sum += received->rssi_sum;
      rssi_count += received->rssi_count;
    #else
      ack->size = 4;
      ack->data[1] = received->rssi_count;
      ack->data[2] = received->rssi_sum & 0xFF;
      ack->data[3] = (received->rssi_sum >> 8) & 0xFF;
      SEGGER_RTT_Write(0, (const char*)&(ack->data[0]), 4);
    #endif

  }
#endif

#if CFMODE == CFMODE_TX
  static unsigned int channel;
  static EsbDatarate datarate;
  static int txpower;
#endif

int main()
{
  systickInit();

  NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  buttonInit(buttonIdle);
  LED_ON();

  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  channel = CF_CHANNEL;
  datarate = CF_DATARATE;
  txpower = CF_POWER;

  esbSetDatarate(datarate);
  esbSetChannel(channel);
  esbSetTxPower(txpower);
  esbInit();

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  #if CFMODE == CFMODE_RX
    esbSetPacketReceivedHandler(packetReceivedHandler);
    esbStartRx();
  #endif

  mainloop();

  // The main loop should never end
  while(1);

  return 0;
}

void delayms(int delay)
{
  int starttime = systickGetTick();
  while (systickGetTick() < starttime + delay);
}

int getNextChannel(int ch)
{
  return (ch + 1) % 126;
}

int getNextPower(int power)
{
  switch(power)
  {
  case RADIO_TXPOWER_TXPOWER_Pos4dBm:
    return RADIO_TXPOWER_TXPOWER_0dBm;
  case RADIO_TXPOWER_TXPOWER_0dBm:
    return RADIO_TXPOWER_TXPOWER_Neg4dBm;
  case RADIO_TXPOWER_TXPOWER_Neg4dBm:
    return RADIO_TXPOWER_TXPOWER_Neg8dBm;
  case RADIO_TXPOWER_TXPOWER_Neg8dBm:
    return RADIO_TXPOWER_TXPOWER_Neg12dBm;
  case RADIO_TXPOWER_TXPOWER_Neg12dBm:
    return RADIO_TXPOWER_TXPOWER_Neg16dBm;
  case RADIO_TXPOWER_TXPOWER_Neg16dBm:
    return RADIO_TXPOWER_TXPOWER_Neg20dBm;
  case RADIO_TXPOWER_TXPOWER_Neg20dBm:
    return RADIO_TXPOWER_TXPOWER_Neg30dBm;
  case RADIO_TXPOWER_TXPOWER_Neg30dBm:
    return RADIO_TXPOWER_TXPOWER_Pos4dBm;
  }
  return RADIO_TXPOWER_TXPOWER_Pos4dBm;
}

EsbDatarate getNextDatarate(EsbDatarate dr)
{
  switch (dr)
  {
    case esbDatarate250K:
      return esbDatarate1M;
    case esbDatarate1M:
      return esbDatarate2M;
    case esbDatarate2M:
      return esbDatarate250K;
  }
  return esbDatarate250K;
}

void mainloop()
{
#if CFMODE == CFMODE_TX
  int i;
  EsbPacket packet;
  EsbPacket* ack;
  for (i = 0; i < 32; ++i) {
    packet.data[i] = 0x5;
  }
#endif

#if CFMODE == CFMODE_RX
  #if SCAN_MODE == SCAN_MODE_NONE
    int lastPacket = systickGetTick();
  #endif
#endif

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);
  nrf_gpio_pin_set(RADIO_PAEN_PIN);

  while(1)
  {

    #if CFMODE == CFMODE_RX
      #if SCAN_MODE == SCAN_MODE_CHANNEL
        if (channelSwitchAckTime && systickGetTick() >= channelSwitchAckTime + 10)
        {
          channel = getNextChannel(channel);
          esbStopRx();
          esbSetChannel(channel);
          esbStartRx();
          channelSwitchAckTime = 0;
        }
      #endif
      #if SCAN_MODE == SCAN_MODE_POWER
        txpower = getNextPower(txpower);
        esbSetTxPower(txpower);
      #endif
      #if SCAN_MODE == SCAN_MODE_DATARATE
        if (datarateSwitchAckTime && systickGetTick() >= datarateSwitchAckTime + 50)
        {
            datarate = getNextDatarate(datarate);
            esbStopRx();
            esbSetDatarate(datarate);
            esbStartRx();
            datarateSwitchAckTime = 0;
        }
      #endif
      #if SCAN_MODE == SCAN_MODE_NONE
        if (rssi_count > 0 && systickGetTick() >= lastPacket + 100)
        {
          int rssi = rssi_sum / rssi_count;
          SEGGER_RTT_Write(0, (const char*)&(rssi), 1);
          rssi_count = 0;
          rssi_sum = 0;
          lastPacket = systickGetTick();
        }
      #endif
    #endif
    #if CFMODE == CFMODE_TX
      packet.pid = (packet.pid + 1) % 4;
      packet.size = 32;
      packet.data[0] = channel;
      ack = esbSendPacket(&packet);
      if (ack && ack->size)
      {
        #if SCAN_MODE == SCAN_MODE_CHANNEL
          channel = getNextChannel(channel);
          esbSetChannel(channel);
        #endif
        #if SCAN_MODE == SCAN_MODE_POWER
          txpower = getNextPower(txpower);
          esbSetTxPower(txpower);
        #endif
        #if SCAN_MODE == SCAN_MODE_DATARATE
          datarate = getNextDatarate(datarate);
          esbSetDatarate(datarate);
        #endif
      }

    #endif

    // Button event handling
    ButtonEvent be = buttonGetState();
    if (be == buttonShortPress)
    {
      //stop NRF
      LED_OFF();
      nrf_gpio_cfg_input(PM_VBAT_SINK_PIN, NRF_GPIO_PIN_NOPULL);

      NRF_POWER->SYSTEMOFF = 1UL;
    }

    // processes loop
    buttonProcess();
  }
}
