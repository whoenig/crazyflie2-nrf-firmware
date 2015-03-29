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

static void mainloop(void);

static unsigned int totalnum = CF_TOTALNUM;
static unsigned int ID = CF_ID;
static unsigned int channel = CF_CHANNEL;
static EsbDatarate datarate = CF_DATARATE;
static int txpower = CF_POWER;

static int num_nbrs = 0; // Number of neighbors
static int nbr_flag = 0;
struct {
  uint32_t rssi_count;
  uint32_t rssi_sum;
} RSSI_Nbr[CF_TOTALNUM];
/*
received->data[] contains:
[0] = ID
[1] = CMD
[2-31] = MSG
*/

static int crazy_state; // Crazyflie State

enum CrazyState {waitToSync, signalTx, signalRx, dataShareTx, dataShareRx, localize};
enum{CMD_CLOCKSYNC = 1};
enum{PIPE_CF = 0, PIPE_PC = 1};

static const int SIGNAL_TX_TIME = 200; //ms
static const int DATASHARE_TX_TIME = 200; //ms

void packetReceivedHandler(EsbPacket* received, EsbPacket* ack)
{
  uint8_t nbr;

  if (received->pipe == PIPE_PC)
  {
    switch(crazy_state)
    {
      case waitToSync: // Reset clock
        switch(received->data[0])
        {
          case CMD_CLOCKSYNC:
            systickSetTick(0);
            crazy_state = ((ID==0) ? signalTx : signalRx);
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
  else if (received->pipe == PIPE_CF)
  {
    switch(crazy_state)
    {
      case signalRx: // Extract ID, RSSI from MSG. Filter RSSI and store in array. Check for number of neighbors.
        nbr = received->data[0];
        RSSI_Nbr[nbr].rssi_count += received->rssi_count;
        RSSI_Nbr[nbr].rssi_sum += received->rssi_sum;
        break;
      case dataShareRx: // ToDo: Implement apt functionality here
        break;
      default:
        break;
    }
  }
}

int main()
{
  int i = 0;
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

  crazy_state = waitToSync;

  for(i = 0; i < totalnum; i++)
  {
    RSSI_Nbr[i].rssi_sum = 0;
    RSSI_Nbr[i].rssi_count = 0;
  }

  esbSetDatarate(datarate);
  esbSetChannel(channel);
  esbSetTxPower(txpower);
  esbInit();

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  // setup as receiver initially
  esbSetPacketReceivedHandler(packetReceivedHandler);
  esbStartRx();

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

void mainloop()
{
  EsbPacket packet;
  int i = 0;

  for (i = 0; i < 32; ++i) {
    packet.data[i] = 0x5;
  }

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);
  nrf_gpio_pin_set(RADIO_PAEN_PIN);

  while(1)
  {
    // Decide current crazy_state
    if(!(crazy_state==waitToSync))
    {
      int cur_time = systickGetTick();
      if((cur_time >= ID*SIGNAL_TX_TIME) && (cur_time < (ID+1)*SIGNAL_TX_TIME))
      {
        if(!(crazy_state==signalTx))
        {
          crazy_state = signalTx;
          esbStopRx();
        }
      }
      else if((cur_time < ID*SIGNAL_TX_TIME) || ((cur_time >= (ID+1)*SIGNAL_TX_TIME) && (cur_time < totalnum*SIGNAL_TX_TIME)))
      {
        if(!(crazy_state==signalRx))
        {
          crazy_state = signalRx;
          esbStartRx();
        }
      }
      else if((cur_time >= totalnum*SIGNAL_TX_TIME + ID*DATASHARE_TX_TIME) && (cur_time < totalnum*SIGNAL_TX_TIME + (ID+1)*DATASHARE_TX_TIME))
      {
        if(!(crazy_state==dataShareTx))
        {
          if(!nbr_flag)
          {
            num_nbrs = 0;
            nbr_flag = 1;
            for (i = 0; i < totalnum; i++)
            {
              if(RSSI_Nbr[i].rssi_count!=0)
                num_nbrs++;
            }
          }
          crazy_state = dataShareTx;
          esbStopRx();
        }
      }
      else if((cur_time < totalnum*SIGNAL_TX_TIME + ID*DATASHARE_TX_TIME) || ((cur_time >= totalnum*SIGNAL_TX_TIME + (ID+1)*DATASHARE_TX_TIME) && (cur_time < totalnum*(SIGNAL_TX_TIME+DATASHARE_TX_TIME))))
      {
        if(!(crazy_state==dataShareRx))
        {
          if(!nbr_flag)
          {
            num_nbrs = 0;
            nbr_flag = 1;
            for (i = 0; i < totalnum; i++)
            {
              if(RSSI_Nbr[i].rssi_count!=0)
                num_nbrs++;
            }
          }
          crazy_state = dataShareRx;
          esbStartRx();
        }
      }
      else
      {
        if(!(crazy_state==localize))
        {
          crazy_state = localize;
          esbStopRx();
          for(i = 0; i < totalnum; ++i)
          {
            SEGGER_RTT_printf(0, "%d, %d, %d\n", i, RSSI_Nbr[i].rssi_count, RSSI_Nbr[i].rssi_sum);
          }
        }
      }
    }

    //Take appropriate action according to current state
    switch(crazy_state)
    {
      case signalTx:
        packet.data[0] = ID;
        packet.size = 32; // longer messages give us more samples
        packet.pid = (packet.pid + 1) % 4;
        esbSendPacket(&packet, PIPE_CF);
        break;
      case dataShareTx: // ToDo
        break;
      case localize: // ToDo
        break;
      default:
        break;
    }

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
