/*
 *   Copyright (C) 2015,2016,2018,2020,2021 by Jonathan Naylor G4KLX
 *   Copyright (C) 2018 by Andy Uribe CA6JAU
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(SERIALPORT_H)
#define  SERIALPORT_H

#include "Globals.h"

enum DVM_COMMANDS {
    CMD_GET_VERSION = 0x00U,
    CMD_GET_STATUS = 0x01U,
    CMD_SET_CONFIG = 0x02U,
    CMD_SET_MODE = 0x03U,

    CMD_SET_SYMLVLADJ = 0x04U,
    CMD_SET_RXLEVEL = 0x05U,
    CMD_SET_RFPARAMS = 0x06U,

    CMD_CAL_DATA = 0x08U,
    CMD_RSSI_DATA = 0x09U,

    CMD_SEND_CWID = 0x0AU,

    CMD_DMR_DATA1 = 0x18U,
    CMD_DMR_LOST1 = 0x19U,
    CMD_DMR_DATA2 = 0x1AU,
    CMD_DMR_LOST2 = 0x1BU,
    CMD_DMR_SHORTLC = 0x1CU,
    CMD_DMR_START = 0x1DU,
    CMD_DMR_ABORT = 0x1EU,

    CMD_P25_DATA = 0x31U,
    CMD_P25_LOST = 0x32U,
    CMD_P25_CLEAR = 0x33U,

    CMD_ACK = 0x70U,
    CMD_NAK = 0x7FU,

    CMD_DEBUG1 = 0xF1U,
    CMD_DEBUG2 = 0xF2U,
    CMD_DEBUG3 = 0xF3U,
    CMD_DEBUG4 = 0xF4U,
    CMD_DEBUG5 = 0xF5U,
    CMD_DEBUG_DUMP = 0xFAU,
};

enum CMD_REASON_CODE {
    RSN_OK = 0U, 
    RSN_NAK = 1U, 

    RSN_ILLEGAL_LENGTH = 2U, 
    RSN_INVALID_REQUEST = 4U, 
    RSN_RINGBUFF_FULL = 8U, 

    RSN_INVALID_FDMA_PREAMBLE = 10U,
    RSN_INVALID_MODE = 11U,
    
    RSN_INVALID_DMR_CC = 12U,
    RSN_INVALID_DMR_SLOT = 13U,
    RSN_INVALID_DMR_START = 14U,
    RSN_INVALID_DMR_RX_DELAY = 15U,

    RSN_INVALID_P25_CORR_COUNT = 16U,

    RSN_DMR_DISABLED = 63U,
    RSN_P25_DISABLED = 64U,
};

const uint8_t DVM_FRAME_START = 0xFEU;


class CSerialPort {
public:
  CSerialPort();

  void start();

  void process();

#if defined(SERIAL_REPEATER) || defined(SERIAL_REPEATER_USART1)
  void writeSerialRpt(const uint8_t* data, uint8_t length);
#endif

  void writeDMRData(bool slot, const uint8_t* data, uint8_t length);
  void writeDMRLost(bool slot);

  void writeP25Data(const uint8_t* data, uint8_t length);
  void writeP25Lost();

  void writeNXDNData(const uint8_t* data, uint8_t length);
  void writeNXDNLost();

#if defined(SEND_RSSI_DATA)
  void writeRSSIData(const uint8_t* data, uint8_t length);
#endif

#if defined(ENABLE_DEBUG)
  void writeDebug(const char* text);
  void writeDebug(const char* text, int16_t n1);
  void writeDebugI(const char* text, int32_t n1);
  void writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3);
  void writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4);
#endif
  void writeDebug(const char* text, int16_t n1, int16_t n2);

private:
  uint8_t m_buffer[256U];
  uint8_t m_ptr;
  uint8_t m_len;
  uint8_t m_serial_buffer[128U];
  uint8_t m_serial_len;

  bool    m_debug;
  bool    m_firstCal;

  void    sendACK();
  void    sendNAK(uint8_t err);
  void    getStatus();
  void    getVersion();
  uint8_t setConfig(const uint8_t* data, uint8_t length);
  uint8_t setMode(const uint8_t* data, uint8_t length);
  void    setMode(DVM_STATE modemState);
  uint8_t setFreq(const uint8_t* data, uint8_t length);
  uint8_t setSymbolLvlAdj(const uint8_t* data, uint8_t length);

  // Hardware versions
  void    beginInt(uint8_t n, int speed);
  int     availableInt(uint8_t n);
  uint8_t readInt(uint8_t n);
  void    writeInt(uint8_t n, const uint8_t* data, uint16_t length, bool flush = false);
};

#endif
