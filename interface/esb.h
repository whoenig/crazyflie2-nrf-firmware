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
 * Implementation of the Nordic ESB protocol in PRX mode for nRF51822
 */
#ifndef __ESB_H__
#define __ESB_H__

#include <stdbool.h>
#include <stdint.h>

/* ESB Radio packet */
typedef struct esbPacket_s {
  /* Part that is written by the radio DMA */
  struct {
    uint8_t size;
    union {
      uint8_t s1;
      struct {
        uint8_t noack :1;
        uint8_t pid :2;
      };
    };
    uint8_t data[32];
  } __attribute__((packed));
  unsigned int crc;
  uint8_t pipe;
  uint16_t rssi_sum;
  uint8_t rssi_count;
} EsbPacket;

typedef enum esbDatarate_e { esbDatarate250K=0,
                             esbDatarate1M=1,
                             esbDatarate2M=2 } EsbDatarate;

/*** For compatibility ***/
#define RADIO_RATE_250K esbDatarate250K
#define RADIO_RATE_1M esbDatarate1M
#define RADIO_RATE_2M esbDatarate2M

/* Initialize the radio for ESB */
void esbInit();

/* Stop ESB and free the radio */
void esbDeinit();

typedef void (*esbPacketReceivedHandler_t)(EsbPacket* received, EsbPacket* ack);

void esbSetPacketReceivedHandler(esbPacketReceivedHandler_t handler);

void esbStartRx();
void esbStopRx();

EsbPacket* esbSendPacket(EsbPacket* packet, uint8_t pipe);

/* Set datarate */
void esbSetDatarate(EsbDatarate datarate);

/* Set channel */
void esbSetChannel(unsigned int channel);

/* Set output power */
void esbSetTxPower(int power);

/* Set of disable radio continuous wave */
void esbSetContwave(bool enable);

/* Set the address of the radio */
void esbSetAddress0(uint64_t address); // pipe0
void esbSetAddress1(uint64_t address); // pipe1

#endif //__ESB_H__
