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

#if CFMODE == 2
  #include "esb.h"
#else
  #include "micro_esb.h"
  #include "uesb_error_codes.h"
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

#if CFMODE == 0 || CFMODE == 1
static uesb_payload_t rx_payload;
static uesb_payload_t ack_payload;
static uint8_t channel;
static uesb_bitrate_t bitrate = UESB_BITRATE_250KBPS;

static int count = 0;

static int channelSwitchAckTime = 0;

void uesb_event_handler()
{
    static uint32_t rf_interrupts;
    // static uint32_t tx_attempts;

    uesb_get_clear_interrupts(&rf_interrupts);

    // if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
    // {
    // }

    if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
    {
        uesb_flush_tx();
    }

    // if(rf_interrupts & UESB_INT_RX_DR_MSK)
    // {
    //     uesb_read_rx_payload(&rx_payload);
    //     NRF_GPIO->OUTCLR = 0xFUL << 8;
    //     NRF_GPIO->OUTSET = (uint32_t)((rx_payload.data[2] & 0x0F) << 8);
    // }
    if(rf_interrupts & UESB_INT_RX_DR_MSK)
    {
    #if CFMODE == 0
      LED_OFF();
      uesb_read_rx_payload(&rx_payload);

      count = rx_payload.data[1];

      ack_payload.length = 3;
      ack_payload.data[0] = channel;
      ack_payload.data[1] = count;
      ack_payload.data[2] = rx_payload.rssi;

      uesb_write_ack_payload(&ack_payload);
      // SEGGER_RTT_Write(0, (const char*)&ack_payload.data[0], 3);

      SEGGER_RTT_printf(0, "%d,%d,%d\n", channel, count, rx_payload.rssi);
      // char bla[10];
      // itoa(count, bla, 10);
      // int len = strlen(bla);
      // bla[len] = '\n';
      // bla[len+1] = '\0';
      // SEGGER_RTT_Write(0, (const char*)&count, sizeof(count));
      // SEGGER_RTT_Write(0, "\n", 1);
      //++count;
      if (!channelSwitchAckTime && count == 20)// && channel == rx_payload.data[0])
      {
        // SEGGER_RTT_printf(0, "%d,%d,%d\n", channel, count, rx_payload.rssi);
        // SEGGER_RTT_printf(0, "CSAT");
        channelSwitchAckTime = systickGetTick();
      }
    #endif
    #if CFMODE == 1
      uesb_read_rx_payload(&rx_payload);
      // SEGGER_RTT_printf(0, "%d,%d\n", rx_payload.data[0], rx_payload.data[1]);
      if (channel == rx_payload.data[0] && count == rx_payload.data[1])
      {
        //if (!channelSwitchAckTime && count == 20)
        if (count == 20)
        {
          channel = rx_payload.data[0];
          channel += 1;
          if (channel == 126) {
            switch (bitrate)
            {
              case UESB_BITRATE_250KBPS:
                bitrate = UESB_BITRATE_1MBPS;
                break;
              case UESB_BITRATE_1MBPS:
                bitrate = UESB_BITRATE_2MBPS;
                break;
              case UESB_BITRATE_2MBPS:
                bitrate = UESB_BITRATE_250KBPS;
                break;
            }
            uesb_set_bitrate(bitrate);
            channel = 0;
          }
          uesb_set_rf_channel(channel);
          count = 0;
          // channelSwitchAckTime = systickGetTick();
        }
        else// if (count < 20)
        {
          count += 1;
        }
      }
    #endif
    }

    // uesb_get_tx_attempts(&tx_attempts);
    // NRF_GPIO->OUTCLR = 0xFUL << 12;
    // NRF_GPIO->OUTSET = (tx_attempts & 0x0F) << 12;
}
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

  #if CFMODE == 2 //Bitcraze RX-mode
    esbInit();
    esbSetDatarate(esbDatarate2M);
    esbSetChannel(100);
  #else // RX or TX (u-esb lib)
  channel = 100;
  uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
  uesb_config.rf_channel          = channel;
  uesb_config.crc                 = UESB_CRC_16BIT; // TODO
  //uesb_config.retransmit_count    = 6;
  uesb_config.retransmit_delay    = 500;
  uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
  uesb_config.bitrate             = UESB_BITRATE_2MBPS;
  uesb_config.event_handler       = uesb_event_handler;
  uesb_config.tx_output_power     = UESB_TX_POWER_0DBM,
  uesb_config.dynamic_ack_enabled = 1;
  #if CFMODE == 0 // RX
    uesb_config.mode                = UESB_MODE_PRX;
  #endif

  uesb_init(&uesb_config);

  uint8_t addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uesb_set_address(UESB_ADDRESS_PIPE0, addr);

  #endif

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  #if CFMODE == 0 // RX
    uesb_start_rx();
  #endif

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

void mainloop()
{
#if CFMODE == 1
  uesb_payload_t tx_payload;

  tx_payload.length  = 16;
  tx_payload.pipe    = 0;
  tx_payload.data[0] = 0xCA;
  tx_payload.data[1] = 0xFE;
  tx_payload.data[2] = 0xBA;
  tx_payload.data[3] = 0xBE;
#endif

  nrf_gpio_cfg_output(RADIO_PAEN_PIN);

  while(1)
  {
    PmState state = pmGetState();
    if (state != pmSysOff)
    {
      LED_ON();
      nrf_gpio_pin_set(RADIO_PAEN_PIN);

      #if CFMODE == 2
        if (esbIsRxPacket())
        {
          EsbPacket* packet = esbGetRxPacket();
          esbReleaseRxPacket(packet);
        }
      #endif
      #if CFMODE==1 // Tx
        tx_payload.data[0] = channel;
        tx_payload.data[1] = count;
        uint32_t res = uesb_write_tx_payload(&tx_payload);
        //SEGGER_RTT_printf(0, "%d\n", res);
        if (res == UESB_SUCCESS)
        {
          LED_OFF();
          //SEGGER_RTT_printf(0, "%d\n", count);
          //++count;
        }
      #endif

      //#if CFMODE==0
      // switch channel 10ms after we ACK'ed the 20th packet
      //if (count == 20 && systickGetTick() >= channelSwitchAckTime + 20)
      //#elif CFMODE == 1
      //if (count == 21 && systickGetTick() >= channelSwitchAckTime + 20)
      //#endif
      #if CFMODE==0 || CFMODE==1
      if (count == 20 && channelSwitchAckTime && systickGetTick() >= channelSwitchAckTime + 20)
      {
        channel += 1;
        // if (channel > 125) {
          // channel = 0;
        // }
        #if CFMODE==0
        uesb_stop_rx();
        #endif
        uesb_flush_rx();
        uesb_flush_tx();
        if (channel == 126) {
          switch (bitrate)
          {
            case UESB_BITRATE_250KBPS:
              bitrate = UESB_BITRATE_1MBPS;
              break;
            case UESB_BITRATE_1MBPS:
              bitrate = UESB_BITRATE_2MBPS;
              break;
            case UESB_BITRATE_2MBPS:
              bitrate = UESB_BITRATE_250KBPS;
              break;
          }
          uesb_set_bitrate(bitrate);
          channel = 0;
        }
        uesb_set_rf_channel(channel);
        #if CFMODE==0
        uesb_start_rx();
        #endif
        //SEGGER_RTT_printf(0, "Update Channel: %d\n", channel);
        count = 0;
        channelSwitchAckTime = 0;
      }

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

