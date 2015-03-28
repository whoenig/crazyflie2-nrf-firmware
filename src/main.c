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

enum
{
  PIPE_CF = 0,
  PIPE_PC = 1,
};

enum
{
  CMD_CLOCKSYNC = 1,
};

#if CFMODE == CFMODE_RX
static unsigned int channel;
static int count = 0;
static int channelSwitchAckTime;
static EsbDatarate datarate;
static int txpower;
static int rssi_sum;
static int rssi_count;
void packetReceivedHandler(EsbPacket* received, EsbPacket* ack)
{
  // int i;
  count = received->data[1];

  #if SCAN_MODE == SCAN_MODE_CHANNEL
    ack->data[0] = channel;
  #endif
  #if SCAN_MODE == SCAN_MODE_POWER
    ack->data[0] = txpower;
  #endif
  #if SCAN_MODE == SCAN_MODE_DATARATE
    ack->data[0] = datarate;
  #endif

  #if SCAN_MODE == SCAN_MODE_NONE
    ack->size = 3;
    ack->data[0] = received->rssi;
    rssi_sum += received->rssi;
    ++rssi_count;
    // SEGGER_RTT_Write(0, (const char*)&(ack->data[0]), 1);
  #else
    ack->size = 3;
    ack->data[1] = count;
    ack->data[2] = received->rssi;
    SEGGER_RTT_Write(0, (const char*)&(ack->data[0]), 3);

    if (count == 10)// && channel == rx_payload.data[0])
    {
      // SEGGER_RTT_printf(0, "%d,%d,%d\n", channel, count, received->rssi);
      // SEGGER_RTT_printf(0, "CSAT");
      channelSwitchAckTime = systickGetTick();
    }
  #endif

  if (received->pipe == PIPE_PC)
  {
    switch(received->data[0])
    {
      case CMD_CLOCKSYNC:
        // ToDo: reset clocks here!
        LED_OFF();
        break;
    }
  }
  else if (received->pipe == PIPE_CF)
  {
    ack->size = 1;
    ack->data[0] = received->rssi;
    // ToDo: handle any messages from the swarm here
    //       This is part of an interrupt handler, so keep it short!
  }

}
#endif

#if CFMODE == CFMODE_TX
static unsigned int channel;
static int count = 0;
static EsbDatarate datarate;
static int txpower;
#endif

int main()
{
  systickInit();

  NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  buttonInit(buttonIdle);
  LED_ON();

  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  channel = CF_CHANNEL;
  count = 0;
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
  EsbPacket packet;
  EsbPacket* ack;
  packet.data[0] = 0xCA;
  packet.data[1] = 0xFE;
  packet.size = 1;
#endif

#if CFMODE == CFMODE_RX
  int lastPacket = systickGetTick();
#endif

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);
  nrf_gpio_pin_set(RADIO_PAEN_PIN);

  while(1)
  {

    #if CFMODE == CFMODE_RX
      #if SCAN_MODE == SCAN_MODE_CHANNEL
        if (count == 10 && systickGetTick() >= channelSwitchAckTime + 50)
        {
          channel = getNextChannel(channel);
          esbStopRx();
          esbSetChannel(channel);
          esbStartRx();
          count = 0;
        }
      #endif
      #if SCAN_MODE == SCAN_MODE_POWER
        if (count == 10)
        {
            txpower = getNextPower(txpower);
            esbSetTxPower(txpower);
            count = 0;
        }
      #endif
      #if SCAN_MODE == SCAN_MODE_DATARATE
        if (count == 10 && systickGetTick() >= channelSwitchAckTime + 50)
        {
            datarate = getNextDatarate(datarate);
            esbStopRx();
            esbSetDatarate(datarate);
            esbStartRx();
            count = 0;
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
      packet.size = 16;
      packet.data[0] = channel;
      packet.data[1] = count;
      ack = esbSendPacket(&packet, PIPE_CF);
        if (ack && ack->size)
        {
          if (ack->data[1] == count)
          {
            if (count == 10)
            {
              #if SCAN_MODE == SCAN_MODE_CHANNEL
                channel = ack->data[0] + 1;
                if (channel == 126) {
                  channel = 0;
                }
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
              count = 0;
            }
            else
            {
              count += 1;
            }
          }
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
