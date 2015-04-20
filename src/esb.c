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
 *
 * esb.c - Implementation of the Nordic ESB protocol in PRX mode for nRF51822
 */
#include <stdlib.h>
#include <stdbool.h>

#include "esb.h"
#include "systick.h"

#include <nrf.h>

#include "SEGGER_RTT.h"

static void esbInterruptHandler();

static bool isInit = true;

static unsigned int channel = 2;

uint32_t rssi_sum = 0;
uint32_t rssi_count = 0;

void RADIO_IRQHandler()
{
  esbInterruptHandler();
}

void esbInterruptHandler()
{
  if(NRF_RADIO->EVENTS_RSSIEND)
  {
    NRF_RADIO->EVENTS_RSSIEND = 0; // clear interrupt

    rssi_sum += NRF_RADIO->RSSISAMPLE;
    ++rssi_count;
    // restart the task to get another sample
    NRF_RADIO->TASKS_RSSISTART = 1;
  }
}


/* Public API */

// S1 is used for compatibility with NRF24L0+. These three bits are used
// to store the PID and NO_ACK.
#define PACKET0_S1_SIZE                  (3UL)
// S0 is not used
#define PACKET0_S0_SIZE                  (0UL)
// The size of the packet length field is 6 bits
#define PACKET0_PAYLOAD_SIZE             (6UL)
// The size of the base address field is 4 bytes
#define PACKET1_BASE_ADDRESS_LENGTH      (4UL)
// Don't use any extra added length besides the length field when sending
#define PACKET1_STATIC_LENGTH            (0UL)
// Max payload allowed in a packet
#define PACKET1_PAYLOAD_SIZE             (32UL)

void esbInit()
{
  NRF_RADIO->POWER = 1;
  // Enable Radio interrupts
  NVIC_SetPriority(RADIO_IRQn, 3);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->FREQUENCY = channel;
  NRF_RADIO->TEST = 2;

  // Enable interrupt for end event
  NRF_RADIO->INTENSET = RADIO_INTENSET_RSSIEND_Msk;

  // Set all shorts so that RSSI is measured and only END is required interrupt
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RSSISTOP_Enabled;

  isInit = true;
}

void esbReset()
{
  // esbDeinit();
  // esbInit();
  if (!isInit) return;

  __disable_irq();

  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->INTENCLR = 0xFFFFFFFF;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while(NRF_RADIO->EVENTS_DISABLED == 0U);
  NRF_RADIO->POWER = 0;

  NVIC_GetPendingIRQ(RADIO_IRQn);
  __enable_irq();

  esbInit();

}

void esbDeinit()
{
  NVIC_DisableIRQ(RADIO_IRQn);

  NRF_RADIO->INTENCLR = RADIO_INTENSET_END_Msk;
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  NRF_RADIO->POWER = 0;
}

void esbStartRx()
{
  NRF_RADIO->TASKS_RXEN = 1U;
  rssi_sum = 0;
  rssi_count = 0;
  NRF_RADIO->TASKS_RSSISTART = 1;
}

void esbStopRx()
{
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while(NRF_RADIO->EVENTS_DISABLED == 0);
}

void esbSetChannel(unsigned int ch)
{
  if (ch < 126) {
    channel = ch;
    NRF_RADIO->FREQUENCY = channel;
  }

}
