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
#include "syslink.h"
#include "uart.h"

#include "nrf_gpio.h"

#include "SEGGER_RTT.h"

static void mainloop(void);
static void delayms(int delay);

static unsigned int channel;
extern uint32_t rssi_sum;
extern uint32_t rssi_count;

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

  // Configure GPIO
  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  /* PM-side IOs */
  nrf_gpio_cfg_output(PM_VCCEN_PIN);
  nrf_gpio_pin_clear(PM_VCCEN_PIN);
  nrf_gpio_cfg_input(PM_PGOOD_PIN, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(PM_CHG_PIN, NRF_GPIO_PIN_PULLUP);

  // Configure radio
  channel = 0;

  // esbSetDatarate(datarate);
  esbSetChannel(channel);
  // esbSetTxPower(txpower);
  esbInit();

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

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

int getNextChannel(int ch)
{
  return (ch + 1) % 126;
}

void mainloop()
{
  int lastSwitch = systickGetTick();

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);
  nrf_gpio_pin_set(RADIO_PAEN_PIN);

  while(1)
  {
    if (systickGetTick() >= lastSwitch + 10)
    {
      struct __attribute__ ((__packed__))
      {
        uint8_t channel;
        uint32_t rssi_count;
        uint32_t rssi_sum;
      } result = {channel, rssi_count, rssi_sum};

      esbStopRx();
      // SEGGER_RTT_printf(0, "%d, %d, %d\n", channel, rssi_count, rssi_sum);
      // SEGGER_RTT_printf("BUH\n");
      SEGGER_RTT_Write(0, (const char*)&result, sizeof(result));
      channel = getNextChannel(channel);
      esbSetChannel(channel);
      esbStartRx();
      lastSwitch = systickGetTick();
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
