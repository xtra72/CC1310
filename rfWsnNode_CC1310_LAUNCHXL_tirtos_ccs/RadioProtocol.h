/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RADIOPROTOCOL_H_
#define RADIOPROTOCOL_H_

#include "stdint.h"
#include "easylink/EasyLink.h"

#define RADIO_CONCENTRATOR_ADDRESS     0x00

/*
 * Uncomment to change the modulation away from the default found in the 
 * EASYLINK_PARAM_CONFIG macro in easylink_config.h
 *
 * Valid values can be found in the EasyLink_PhyType enum in EasyLink.h
 */
//#define DEFINED_RADIO_EASYLINK_MODULATION     EasyLink_Phy_Custom

#define RADIO_PACKET_TYPE_ACK_PACKET            0
#define RADIO_PACKET_TYPE_RAW_DATA_PACKET       1
#define RADIO_PACKET_TYPE_TEST_RESET            2

#define RADIO_PACKET_OPTIONS_CRC                (1 << 0)

struct  PacketHeader {
    uint8_t     sourceAddress;
    uint8_t     packetType;
    uint8_t     options;
    uint8_t     length;
};

struct  TestResetPacket {
    struct      PacketHeader header;
};

struct  TestConfigPacket {
    struct      PacketHeader header;
    uint32_t    packetCount;
};

struct  RawDataPacket {
    struct      PacketHeader header;
    uint16_t    crc;
    uint8_t     data[EASYLINK_MAX_DATA_LENGTH - sizeof(struct PacketHeader) - sizeof(uint16_t)];
};

struct AckPacket {
    struct PacketHeader header;
};

union Packet {
    struct PacketHeader     header;
    struct TestConfigPacket testConfig;
    struct RawDataPacket    rawData;
    struct AckPacket        ack;
};

#endif /* RADIOPROTOCOL_H_ */
