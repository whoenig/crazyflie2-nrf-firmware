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
static int datarate = esbDatarate2M;
static int txpower = RADIO_TXPOWER_TXPOWER_0dBm;
static bool contwave = false;
static uint64_t address0 = 0xE7E7E7E7E7ULL;
static uint64_t address1 = 0xE7E7E7E7E8ULL;

static enum {doNothing, doAck, doRx, doPtxTx, doPtxAck} rs;      //Radio state

static EsbPacket recvPacket;
static EsbPacket ackPacket;
static esbPacketReceivedHandler_t packetReceivedHandler;
static bool receivedAck = false;

static uint16_t rssi_sum = 0;
static uint8_t rssi_count = 0;
static bool addressmatch = false;

/* helper functions */

static uint32_t swap_bits(uint32_t inp)
{
  uint32_t i;
  uint32_t retval = 0;

  inp = (inp & 0x000000FFUL);

  for(i = 0; i < 8; i++)
  {
    retval |= ((inp >> i) & 0x01) << (7 - i);
  }

  return retval;
}

static uint32_t bytewise_bitswap(uint32_t inp)
{
  return (swap_bits(inp >> 24) << 24)
       | (swap_bits(inp >> 16) << 16)
       | (swap_bits(inp >> 8) << 8)
       | (swap_bits(inp));
}

/* Radio protocol implementation */

static bool isRetry(EsbPacket *pk)
{
  static int prevPid;
  static int prevCrc;

  bool retry = false;

  if ((prevPid == pk->pid) && (prevCrc == pk->crc)) {
    retry = true;
  }

  prevPid = pk->pid;
  prevCrc = pk->crc;

  return retry;
}

// Handles the queue
static void setupAck(bool retry) {
  static EsbPacket * lastSentPacket;

  // Ack to the sender
  NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;

  if (retry) {
    NRF_RADIO->PACKETPTR = (uint32_t)lastSentPacket;
  } else {

    if (packetReceivedHandler)
    {
      packetReceivedHandler(&recvPacket, &ackPacket);
    }
    else
    {
      ackPacket.size = 0;
    }
    NRF_RADIO->PACKETPTR = (uint32_t)&ackPacket;
    lastSentPacket = &ackPacket;
  }

  //After being disabled the radio will automatically send the ACK
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  rs = doAck;
  NRF_RADIO->TASKS_DISABLE = 1UL;
}

static void setupRx() {
  NRF_RADIO->PACKETPTR = (uint32_t)&recvPacket;

  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
  rs = doRx;
  NRF_RADIO->TASKS_DISABLE = 1UL;
}

void RADIO_IRQHandler()
{
  esbInterruptHandler();
}

void esbInterruptHandler()
{
  EsbPacket *pk;

  if (NRF_RADIO->EVENTS_END) {
    NRF_RADIO->EVENTS_END = 0UL;

    // SEGGER_RTT_printf(0, "B");

    switch (rs){
    case doNothing:
      break;
    case doRx:
      // SEGGER_RTT_printf(0, "R");
      //Wrong CRC packet are dropped
      if (!NRF_RADIO->CRCSTATUS) {
        rssi_sum = 0;
        rssi_count = 0;
        addressmatch = false;
        NRF_RADIO->TASKS_START = 1UL;
        // SEGGER_RTT_printf(0, "C");
        return;
      }

      pk = &recvPacket;
      pk->crc = NRF_RADIO->RXCRC;
      pk->pipe = NRF_RADIO->RXMATCH;
      pk->rssi_sum = rssi_sum;
      pk->rssi_count = rssi_count;

      rssi_sum = 0;
      rssi_count = 0;
      addressmatch = false;

      // If this packet is a retry, send the same ACK again
      if (!pk->noack)
      {
        if (isRetry(pk)) {
          setupAck(true);
          // SEGGER_RTT_printf(0, "D");
          return;
        }

        // Good packet received, yea!
        setupAck(false);
      }
      else
      {
        if (packetReceivedHandler)
        {
          packetReceivedHandler(&recvPacket, &ackPacket);
        }
        // continue receiving
        NRF_RADIO->TASKS_START = 1UL;
      }
      break;
    case doAck:
      //Setup RX for next packet
      setupRx();
      break;
    case doPtxTx:
      // SEGGER_RTT_printf(0, "T");
      if (!((EsbPacket*)NRF_RADIO->PACKETPTR)->noack)
      {
        NRF_RADIO->PACKETPTR = (uint32_t)&recvPacket;
        // switch to receive ack after task is disabled
        // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
        // NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
        rs = doPtxAck;
        // NRF_RADIO->TASKS_TXEN;
        // NRF_RADIO->TASKS_DISABLE = 1UL;
      }
      else
      {
        // disable radio (and make sure no short triggers another RX or Tx)
        // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
        // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
        // NRF_RADIO->TASKS_DISABLE = 1UL;
        rs = doNothing;
      }
      break;
    case doPtxAck:
      // SEGGER_RTT_printf(0, "R");
      rs = doNothing;
      //Wrong CRC packet are dropped
      if (!NRF_RADIO->CRCSTATUS) {
        NRF_RADIO->TASKS_START = 1UL;
        return;
      }
      // disable radio (and make sure no short triggers another RX or Tx)
      NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
      NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
      NRF_RADIO->TASKS_DISABLE = 1UL;
      receivedAck = true;
      break;
    }
  }

  if(NRF_RADIO->EVENTS_RSSIEND)
  {
    NRF_RADIO->EVENTS_RSSIEND = 0; // clear interrupt

    // only use the sample if we are still in RX mode
    if (addressmatch && NRF_RADIO->STATE == 3) //Rx
    {
      rssi_sum += NRF_RADIO->RSSISAMPLE;
      ++rssi_count;
      // restart the task to get another sample
      NRF_RADIO->TASKS_RSSISTART = 1;
    }
  }

  if(NRF_RADIO->EVENTS_ADDRESS)
  {
    NRF_RADIO->EVENTS_ADDRESS = 0; // clear interrupt
    addressmatch = true;
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

  NRF_RADIO->TXPOWER = (txpower << RADIO_TXPOWER_TXPOWER_Pos);

  switch (datarate) {
  case esbDatarate250K:
      NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);
      break;
  case esbDatarate1M:
      NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
      break;
  case esbDatarate2M:
      NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
      break;
  }

  NRF_RADIO->FREQUENCY = channel;

  if (contwave) {
    NRF_RADIO->TEST = 3;
    NRF_RADIO->TASKS_RXEN = 1U;
    return;
  }

  // Radio address config
  // Using logical address 0 so only BASE0 and PREFIX0 & 0xFF are used
  NRF_RADIO->PREFIX0 = (bytewise_bitswap(address0 >> 32) & 0xFF) |
                      ((bytewise_bitswap(address1 >> 32) & 0xFF) << 8);  // Prefix byte of addresses 3 to 0
  // NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;  // Prefix byte of addresses 7 to 4
  NRF_RADIO->BASE0   = bytewise_bitswap((uint32_t)address0);  // Base address for prefix 0
  NRF_RADIO->BASE1   = bytewise_bitswap((uint32_t)address1);  // Base address for prefix 1 to 7
  NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
  NRF_RADIO->RXADDRESSES = 0x03UL;    // Enable device address 0 and 1 to use which receiving

  // Packet configuration
  NRF_RADIO->PCNF0 = (PACKET0_S1_SIZE << RADIO_PCNF0_S1LEN_Pos) |
                     (PACKET0_S0_SIZE << RADIO_PCNF0_S0LEN_Pos) |
                     (PACKET0_PAYLOAD_SIZE << RADIO_PCNF0_LFLEN_Pos);

  // Packet configuration
   NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos)    |
                      (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)           |
                      (PACKET1_BASE_ADDRESS_LENGTH << RADIO_PCNF1_BALEN_Pos)       |
                      (PACKET1_STATIC_LENGTH << RADIO_PCNF1_STATLEN_Pos)           |
                      (PACKET1_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos);

  // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
  NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
  NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

  // Enable interrupt for end event
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk |
                        RADIO_INTENSET_RSSIEND_Msk |
                        RADIO_INTENSET_ADDRESS_Msk;

  // Set all shorts so that RSSI is measured and only END is required interrupt
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RSSISTOP_Enabled;
  // NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;

  // Set RX buffer and start RX
  rs = doNothing;
  //NRF_RADIO->PACKETPTR = (uint32_t)&recvPacket;
  //NRF_RADIO->TASKS_RXEN = 1U;

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

void esbSetPacketReceivedHandler(esbPacketReceivedHandler_t handler)
{
  packetReceivedHandler = handler;
}

void esbStartRx()
{
  rs = doRx;
  NRF_RADIO->PACKETPTR = (uint32_t)&recvPacket;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  NRF_RADIO->TASKS_RXEN = 1U;
}

void esbStopRx()
{
  rs = doNothing;
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while(NRF_RADIO->EVENTS_DISABLED == 0);
}

EsbPacket* esbSendPacket(EsbPacket* packet, uint8_t pipe)
{
  receivedAck = false;
  int startTime = systickGetTick();

  NRF_RADIO->TXADDRESS = pipe;
  NRF_RADIO->PACKETPTR = (uint32_t)packet;

  // start transmitting after task is disabled
  // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  // NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  // rs = doNothing;
  // NRF_RADIO->TASKS_DISABLE = 1UL;
  // while(NRF_RADIO->EVENTS_DISABLED == 0U);

  // NRF_RADIO->TASKS_TXEN  = 1;
  // rs = doPtxTx;
  // NRF_RADIO->TASKS_DISABLE = 1UL;

  // rs = doNothing;
  // NRF_RADIO->TASKS_DISABLE = 1UL;
  // while(NRF_RADIO->EVENTS_DISABLED == 0U);

  // NRF_RADIO->PACKETPTR = (uint32_t)packet;
  // After being disabled the radio will automatically receive
  if (!packet->noack)
  {
    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
  }
  else
  {
    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  }
  NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  rs = doPtxTx;
  // NRF_RADIO->TASKS_RXEN = 1U;
  NRF_RADIO->TASKS_TXEN  = 1;
  // NRF_RADIO->TASKS_DISABLE = 1UL;

  // SEGGER_RTT_printf(0, "%d\n", NRF_RADIO->STATE);

  if (!packet->noack)
  {
    // wait 2ms max
    while (startTime + 2 >= systickGetTick()) {
      if (receivedAck) {
        return &recvPacket;
      }
    }

    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
    NRF_RADIO->SHORTS &= ~RADIO_SHORTS_END_DISABLE_Msk;
    NRF_RADIO->TASKS_DISABLE = 1UL;
  }
  else
  {
    while(NRF_RADIO->EVENTS_DISABLED == 0U);
  }
  return NULL;
}

// EsbPacket* esbSendPacket(EsbPacket* packet)
// {
//   int startTime = systickGetTick();

//     // Set payload pointer
//     NRF_RADIO->PACKETPTR = (uint32_t)packet;

//     // enable transmission
//     NRF_RADIO->EVENTS_READY = 0U;
//     NRF_RADIO->TASKS_TXEN = 1;
//     while (NRF_RADIO->EVENTS_READY == 0U) {}

//     // start transmission
//     NRF_RADIO->EVENTS_END = 0U;
//     NRF_RADIO->TASKS_START = 1U;
//     while(NRF_RADIO->EVENTS_END == 0U){}

//     // Disable radio
//     NRF_RADIO->EVENTS_DISABLED = 0U;
//     NRF_RADIO->TASKS_DISABLE = 1U;
//     while(NRF_RADIO->EVENTS_DISABLED == 0U) {}

//       while (startTime + 100 >= systickGetTick()) {}

//     return NULL;
// }

void esbSetDatarate(EsbDatarate dr)
{
  datarate = dr;

  esbReset();
}

void esbSetContwave(bool enable)
{
  contwave = enable;

  esbReset();
}

void esbSetChannel(unsigned int ch)
{
  if (ch < 126) {
    channel = ch;
    NRF_RADIO->FREQUENCY = channel;
    // esbReset();
  }

  // esbReset();
}

void esbSetTxPower(int power)
{
  txpower = power;
  NRF_RADIO->TXPOWER = (txpower << RADIO_TXPOWER_TXPOWER_Pos);

  //esbReset();
}

void esbSetAddress0(uint64_t addr)
{
  address0 = addr;

  esbReset();
}

void esbSetAddress1(uint64_t addr)
{
  address1 = addr;

  esbReset();
}
