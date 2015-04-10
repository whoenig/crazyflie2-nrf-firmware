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

#define CFMODE_RX 0
#define CFMODE_TX 1

#define SCAN_MODE_NONE     0
#define SCAN_MODE_CHANNEL  1
#define SCAN_MODE_POWER    2
#define SCAN_MODE_DATARATE 3

static void mainloop(void);
static void delayms(int delay);

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
  #if SCAN_MODE == SCAN_MODE_POWER
    static int powerSwitchAckTime = 0;
  #endif
  #if SCAN_MODE == SCAN_MODE_NONE
    struct resultType
    {
      uint32_t rssi_count;
      uint32_t rssi_sum;
      int16_t theta1; // yaw of the receiver, in degrees
      int16_t theta2; // yaw of the transmitter, in degrees
    };
    static struct resultType result = {};

  #endif
  void packetReceivedHandler(EsbPacket* received, EsbPacket* ack)
  {
    #if SCAN_MODE == SCAN_MODE_CHANNEL
      ack->data[0] = received->data[0];
      channelSwitchAckTime = systickGetTick();
    #endif
    #if SCAN_MODE == SCAN_MODE_POWER
      ack->data[0] = received->data[0];
      powerSwitchAckTime = systickGetTick();
    #endif
    #if SCAN_MODE == SCAN_MODE_DATARATE
      ack->data[0] = received->data[0];
      datarateSwitchAckTime = systickGetTick();
    #endif

    #if SCAN_MODE == SCAN_MODE_NONE
      ack->size = 4;
      ack->data[1] = received->rssi_count;
      ack->data[2] = received->rssi_sum & 0xFF;
      ack->data[3] = (received->rssi_sum >> 8) & 0xFF;
      result.rssi_sum += received->rssi_sum;
      result.rssi_count += received->rssi_count;
      result.theta2 = (received->data[1] << 8) | received->data[0];
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

void enableSTM(bool enable, bool bootloader)
{
  if(enable)
  {
    NRF_GPIO->PIN_CNF[STM_NRST_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                    | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
                                    | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                    | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                    | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    nrf_gpio_pin_clear(STM_NRST_PIN); //Hold STM reset
    nrf_gpio_pin_set(PM_VCCEN_PIN);
    delayms(2);
    nrf_gpio_cfg_output(STM_BOOT0_PIN);
    if (bootloader) {
      nrf_gpio_pin_set(STM_BOOT0_PIN);
    } else {
      nrf_gpio_pin_clear(STM_BOOT0_PIN);
    }

    delayms(2);
    nrf_gpio_pin_set(STM_NRST_PIN);

     //Activate UART TX pin (otherwise bootloader won't execute firmware)
    nrf_gpio_cfg_output(UART_TX_PIN);
    nrf_gpio_pin_set(UART_TX_PIN);

     // Set 500mA current
    // nrf_gpio_cfg_output(PM_EN1);
    // nrf_gpio_pin_set(PM_EN1);
    // nrf_gpio_cfg_output(PM_EN2);
    // nrf_gpio_pin_clear(PM_EN2);

    // Sink battery divider
    // nrf_gpio_cfg_output(PM_VBAT_SINK_PIN);
    // nrf_gpio_pin_clear(PM_VBAT_SINK_PIN);
  }
  else
  {
    // Disable UART
    nrf_gpio_cfg_input(UART_TX_PIN, NRF_GPIO_PIN_PULLDOWN);
    // Hold reset
    nrf_gpio_pin_clear(STM_NRST_PIN);
    nrf_gpio_cfg_input(STM_BOOT0_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(STM_NRST_PIN, NRF_GPIO_PIN_PULLDOWN);
    // Power off
    nrf_gpio_pin_clear(PM_VCCEN_PIN);
  }

}

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

  enableSTM(true, false);
  uartInit();

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
  int16_t yaw;
  for (i = 0; i < 32; ++i) {
    packet.data[i] = 0x5;
  }
#endif

#if CFMODE == CFMODE_RX
  #if SCAN_MODE == SCAN_MODE_NONE
    int lastPacket = systickGetTick();
  #endif
#endif

#if SCAN_MODE == SCAN_MODE_NONE
  struct syslinkPacket slRxPacket;
  bool slReceived;
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
        if (powerSwitchAckTime && systickGetTick() >= powerSwitchAckTime + 10)
        {
          txpower = getNextPower(txpower);
          esbSetTxPower(txpower);
          powerSwitchAckTime = 0;
        }
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
        if (result.rssi_count > 0 && systickGetTick() >= lastPacket + 100)
        {
          SEGGER_RTT_Write(0, (const char*)&(result), sizeof(result));
          result.rssi_count = 0;
          result.rssi_sum = 0;
          lastPacket = systickGetTick();
        }
        slReceived = syslinkReceive(&slRxPacket);
        if (slReceived)
        {
          if (slRxPacket.type == SYSLINK_SENSORS_POSE &&
              slRxPacket.length == 3 * sizeof(float))
          {
            float roll, pitch, yaw;
            memcpy(&roll, &slRxPacket.data[0], sizeof(float));
            memcpy(&pitch, &slRxPacket.data[4], sizeof(float));
            memcpy(&yaw, &slRxPacket.data[8], sizeof(float));
            result.theta1 = (int16_t)yaw;
            // SEGGER_RTT_printf(0, "%d,%d,%d\n", (int)roll, (int)pitch, (int)yaw);
          }
        }
      #endif
    #endif
    #if CFMODE == CFMODE_TX
      packet.pid = (packet.pid + 1) % 4;
      packet.size = 32;
      #if SCAN_MODE == SCAN_MODE_CHANNEL
        packet.data[0] = channel;
      #endif
      #if SCAN_MODE == SCAN_MODE_POWER
        packet.data[0] = txpower;
      #endif
      #if SCAN_MODE == SCAN_MODE_DATARATE
        packet.data[0] = datarate;
      #endif
      #if SCAN_MODE == SCAN_MODE_NONE
        slReceived = syslinkReceive(&slRxPacket);
        if (slReceived)
        {
          if (slRxPacket.type == SYSLINK_SENSORS_POSE &&
              slRxPacket.length == 3 * sizeof(float))
          {
            float rollf, pitchf, yawf;
            memcpy(&rollf, &slRxPacket.data[0], sizeof(float));
            memcpy(&pitchf, &slRxPacket.data[4], sizeof(float));
            memcpy(&yawf, &slRxPacket.data[8], sizeof(float));
            yaw = (int16_t)yawf;
          }
        }
        packet.data[0] = (uint8_t)yaw;
        packet.data[1] = (uint8_t)(yaw>>8);
        packet.noack = 1;
      #endif
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
      //stop STM
      enableSTM(false, false);

      //stop NRF
      LED_OFF();
      nrf_gpio_cfg_input(PM_VBAT_SINK_PIN, NRF_GPIO_PIN_NOPULL);

      NRF_POWER->SYSTEMOFF = 1UL;
    }

    // processes loop
    buttonProcess();
  }
}
