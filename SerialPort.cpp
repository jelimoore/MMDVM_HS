/*
 *   Copyright (C) 2013,2015,2016,2018,2020,2021 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *   Copyright (C) 2016,2017,2018,2019 by Andy Uribe CA6JAU
 *   Copyright (C) 2019 by Florian Wolters DF2ET
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

#include "Config.h"
#include "Globals.h"
#include "version.h"
#include "SerialPort.h"

const uint8_t MMDVM_FRAME_START  = 0xFEU;

const uint8_t MMDVM_GET_VERSION  = 0x00U;
const uint8_t MMDVM_GET_STATUS   = 0x01U;
const uint8_t MMDVM_SET_CONFIG   = 0x02U;
const uint8_t MMDVM_SET_MODE     = 0x03U;
const uint8_t MMDVM_SET_FREQ     = 0x06U;
const uint8_t MMDVM_SET_SYMBOLLVL= 0x04U;

const uint8_t MMDVM_CAL_DATA     = 0x08U;
const uint8_t MMDVM_RSSI_DATA    = 0x09U;

const uint8_t MMDVM_SEND_CWID    = 0x0AU;

const uint8_t MMDVM_DMR_DATA1    = 0x18U;
const uint8_t MMDVM_DMR_LOST1    = 0x19U;
const uint8_t MMDVM_DMR_DATA2    = 0x1AU;
const uint8_t MMDVM_DMR_LOST2    = 0x1BU;
const uint8_t MMDVM_DMR_SHORTLC  = 0x1CU;
const uint8_t MMDVM_DMR_START    = 0x1DU;
const uint8_t MMDVM_DMR_ABORT    = 0x1EU;

const uint8_t MMDVM_P25_HDR      = 0x30U;
const uint8_t MMDVM_P25_LDU      = 0x31U;
const uint8_t MMDVM_P25_LOST     = 0x32U;
const uint8_t MMDVM_P25_CLEAR    = 0x33U;

const uint8_t MMDVM_ACK          = 0x70U;
const uint8_t MMDVM_NAK          = 0x7FU;

const uint8_t MMDVM_SERIAL       = 0x80U;

const uint8_t MMDVM_TRANSPARENT  = 0x90U;
const uint8_t MMDVM_QSO_INFO     = 0x91U;

const uint8_t MMDVM_DEBUG1       = 0xF1U;
const uint8_t MMDVM_DEBUG2       = 0xF2U;
const uint8_t MMDVM_DEBUG3       = 0xF3U;
const uint8_t MMDVM_DEBUG4       = 0xF4U;
const uint8_t MMDVM_DEBUG5       = 0xF5U;

const uint8_t PROTOCOL_VERSION   = 2U;

#if defined(ENABLE_UDID)
char UDID[] = "00000000000000000000000000000000";
#endif

CSerialPort::CSerialPort() :
m_buffer(),
m_ptr(0U),
m_len(0U),
m_serial_buffer(),
m_serial_len(0U),
m_debug(false),
m_firstCal(false)
{
}

void CSerialPort::sendACK()
{
  uint8_t reply[4U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 4U;
  reply[2U] = MMDVM_ACK;
  reply[3U] = m_buffer[2U];

  writeInt(1U, reply, 4);
}

void CSerialPort::sendNAK(uint8_t err)
{
  uint8_t reply[5U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 5U;
  reply[2U] = MMDVM_NAK;
  reply[3U] = m_buffer[2U];
  reply[4U] = err;

  writeInt(1U, reply, 5);
}

void CSerialPort::getStatus()
{
  io.resetWatchdog();

  uint8_t reply[15U];

  // Send all sorts of interesting internal values
  reply[0U]  = MMDVM_FRAME_START;
  reply[1U]  = 14U;
  reply[2U]  = MMDVM_GET_STATUS;

  reply[3U]  = 0x00U;
  if (m_dmrEnable)
    reply[3U] |= 0x02U;
  if (m_p25Enable)
    reply[3U] |= 0x08U;

  reply[4U]  = uint8_t(m_modemState);

  reply[5U]  = m_tx  ? 0x01U : 0x00U;

  if (io.hasRXOverflow())
    reply[5U] |= 0x04U;

  if (io.hasTXOverflow())
    reply[5U] |= 0x08U;

  // DStar
  reply[6U] = 0U;

  if (m_dmrEnable) {
#if defined(DUPLEX)
    if (m_duplex) {
      reply[7U] = dmrTX.getSpace1();
      reply[8U] = dmrTX.getSpace2();
    } else {
      reply[7U] = 10U;
      reply[8U] = dmrDMOTX.getSpace();
    }
#else
    reply[7U] = 10U;
    reply[8U] = dmrDMOTX.getSpace();
#endif
  } else {
    reply[7U] = 0U;
    reply[8U] = 0U;
  }

  // YSF
    reply[9U] = 0U;

  if (m_p25Enable)
    reply[10U] = p25TX.getSpace();
  else
    reply[10U] = 0U;

  // NXDN
    reply[11U] = 0U;

  // POCSAG
    reply[12U] = 0U;

  // M17
    reply[13U] = 0U;

  writeInt(1U, reply, 14);
}

void CSerialPort::getVersion()
{
  uint8_t reply[132U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_GET_VERSION;

  reply[3U] = PROTOCOL_VERSION;

  uint8_t count = 4U;
  for (uint8_t i = 0U; HARDWARE[i] != 0x00U; i++, count++)
    reply[count] = HARDWARE[i];

#if defined(ENABLE_UDID)
  reply[count++] = '\0';
  for (uint8_t i = 0U; UDID[i] != 0x00U; i++, count++)
    reply[count] = UDID[i];
#endif

  reply[1U] = count;

  writeInt(1U, reply, count);
}

uint8_t CSerialPort::setConfig(const uint8_t* data, uint8_t length)
{
  if (length < 10U)
    return 4U;

  bool simplex   = (data[0U] & 0x80U) == 0x80U;

  m_debug = (data[0U] & 0x10U) == 0x10U;

  bool dmrEnable    = (data[1U] & 0x02U) == 0x02U;
  bool p25Enable    = (data[1U] & 0x08U) == 0x08U;

  uint8_t txDelay = data[2U];
  if (txDelay > 255U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[3U]);

  if (modemState != STATE_IDLE && modemState != STATE_DMR && modemState != STATE_P25 && modemState != STATE_DSTARCAL && modemState != STATE_DMRCAL && modemState != STATE_DMRDMO1K && modemState != STATE_INTCAL && modemState != STATE_RSSICAL && modemState != STATE_POCSAGCAL)
    return 4U;
  if (modemState == STATE_DMR && !dmrEnable)
    return 4U;
  if (modemState == STATE_P25 && !p25Enable)
    return 4U;

  uint8_t colorCode = data[6U];
  if (colorCode > 15U)
    return 4U;

#if defined(DUPLEX)
  uint8_t dmrDelay = data[7U];
#endif

  m_cwIdTXLevel = data[5U]>>2;

  uint8_t dmrTXLevel    = data[10U];
  uint8_t p25TXLevel    = data[12U];

  io.setDeviations(dmrTXLevel, p25TXLevel);

  m_dmrEnable    = dmrEnable;
  m_p25Enable    = p25Enable;

  uint16_t nac = (data[8U] << 4) + (data[9U] >> 4);
  uint8_t p25CorrCount = data[11U];
  p25RX.setNAC(nac);
  //p25RX.setCorrCount(p25CorrCount);

  if (modemState == STATE_DMRCAL || modemState == STATE_DMRDMO1K || modemState == STATE_RSSICAL || modemState == STATE_INTCAL) {
    m_dmrEnable = true;
    m_modemState = STATE_DMR;
    m_calState = modemState;
    if (m_firstCal)
      io.updateCal();
    if (modemState == STATE_RSSICAL)
      io.ifConf(STATE_DMR, true);
  } else {
    m_modemState = modemState;
    m_calState = STATE_IDLE;
  }

  m_duplex      = !simplex;

#if !defined(DUPLEX)
  if (m_duplex && m_calState == STATE_IDLE && modemState != STATE_DSTARCAL) {
    DEBUG1("Full duplex not supported with this firmware");
    return 6U;
  }
#elif defined(DUPLEX) && (defined(ZUMSPOT_ADF7021) || defined(LONESTAR_USB) || defined(SKYBRIDGE_HS))
  if (io.isDualBand() && m_duplex && m_calState == STATE_IDLE && modemState != STATE_DSTARCAL) {
    DEBUG1("Full duplex is not supported on this board");
    return 6U;
  }
#endif

  //dstarTX.setTXDelay(txDelay);
  //ysfTX.setTXDelay(txDelay);
  p25TX.setTXDelay(txDelay);
  //nxdnTX.setTXDelay(txDelay);
  //pocsagTX.setTXDelay(txDelay);
  dmrDMOTX.setTXDelay(txDelay);

#if defined(DUPLEX)
  dmrTX.setColorCode(colorCode);
  dmrRX.setColorCode(colorCode);
  dmrRX.setDelay(dmrDelay);
  dmrIdleRX.setColorCode(colorCode);
#endif

  dmrDMORX.setColorCode(colorCode);

  if (!m_firstCal || (modemState != STATE_DMRCAL && modemState != STATE_DMRDMO1K && modemState != STATE_RSSICAL && modemState != STATE_INTCAL && modemState != STATE_POCSAGCAL)) {
    if(m_dmrEnable)
      io.ifConf(STATE_DMR, true);
    else if(m_p25Enable)
      io.ifConf(STATE_P25, true);
  }

  io.start();
#if defined(ENABLE_DEBUG)
  io.printConf();
#endif

  if (modemState == STATE_DMRCAL || modemState == STATE_DMRDMO1K || modemState == STATE_RSSICAL || modemState == STATE_INTCAL || modemState == STATE_POCSAGCAL)
    m_firstCal = true;

  return 0U;
}

uint8_t CSerialPort::setMode(const uint8_t* data, uint8_t length)
{
  if (length < 1U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[0U]);
  MMDVM_STATE tmpState;

  if (modemState == m_modemState)
    return 0U;

  if (modemState != STATE_IDLE  && modemState != STATE_DMR && modemState != STATE_P25 && modemState != STATE_DMRCAL && modemState != STATE_DMRDMO1K && modemState != STATE_RSSICAL && modemState != STATE_INTCAL && modemState != STATE_POCSAGCAL)
    return 4U;
  if (modemState == STATE_DMR && !m_dmrEnable)
    return 4U;
  if (modemState == STATE_P25 && !m_p25Enable)
    return 4U;

  if (modemState == STATE_DMRCAL || modemState == STATE_DMRDMO1K || modemState == STATE_RSSICAL || modemState == STATE_INTCAL) {
    m_dmrEnable = true;
    tmpState = STATE_DMR;
    m_calState = modemState;
    if (m_firstCal)
      io.updateCal();
  } else {
    tmpState  = modemState;
    m_calState = STATE_IDLE;
  }

  setMode(tmpState);

  return 0U;
}

uint8_t CSerialPort::setFreq(const uint8_t* data, uint8_t length)
{
  uint32_t freq_rx, freq_tx;
  uint8_t rf_power;

  if (length < 9U)
    return 4U;

  // Very old MMDVMHost, set full power
  if (length == 9U)
    rf_power = 255U;

  // Current MMDVMHost, set power from MMDVM.ini
  if (length >= 10U)
    rf_power = data[9U];

  freq_rx  = data[1U] << 0;
  freq_rx |= data[2U] << 8;
  freq_rx |= data[3U] << 16;
  freq_rx |= data[4U] << 24;

  freq_tx  = data[5U] << 0;
  freq_tx |= data[6U] << 8;
  freq_tx |= data[7U] << 16;
  freq_tx |= data[8U] << 24;

  return io.setFreq(freq_rx, freq_tx, rf_power);
}

uint8_t CSerialPort::setSymbolLvlAdj(const uint8_t* data, uint8_t length)
{
    if (length < 4U)
        return 4U;

    int8_t dmrSymLvl3Adj = int8_t(data[0U]) - 128;
    if (dmrSymLvl3Adj > 128)
        return 4U;
    if (dmrSymLvl3Adj < -128)
        return 4U;

    int8_t dmrSymLvl1Adj = int8_t(data[1U]) - 128;
    if (dmrSymLvl1Adj > 128)
        return 4U;
    if (dmrSymLvl1Adj < -128)
        return 4U;

    int8_t p25SymLvl3Adj = int8_t(data[2U]) - 128;
    if (p25SymLvl3Adj > 128)
        return 4U;
    if (p25SymLvl3Adj < -128)
        return 4U;

    int8_t p25SymLvl1Adj = int8_t(data[3U]) - 128;
    if (p25SymLvl1Adj > 128)
        return 4U;
    if (p25SymLvl1Adj < -128)
        return 4U;

    //p25TX.setSymbolLvlAdj(p25SymLvl3Adj, p25SymLvl1Adj);

    //dmrDMOTX.setSymbolLvlAdj(dmrSymLvl3Adj, dmrSymLvl1Adj);
    //dmrTX.setSymbolLvlAdj(dmrSymLvl3Adj, dmrSymLvl1Adj);

    return 0;
}

void CSerialPort::setMode(MMDVM_STATE modemState)
{
  switch (modemState) {
    case STATE_DMR:
      DEBUG1("Mode set to DMR");
      p25RX.reset();
      cwIdTX.reset();
      break;
    case STATE_P25:
      DEBUG1("Mode set to P25");
#if defined(DUPLEX)
      dmrIdleRX.reset();
      dmrRX.reset();
#endif
      dmrDMORX.reset();
      cwIdTX.reset();
      break;
    default:
      DEBUG1("Mode set to Idle");
      // STATE_IDLE
      break;
  }

  m_modemState = modemState;

  if ((modemState != STATE_IDLE) && (m_modemState_prev != modemState)) {
    DEBUG1("setMode: configuring Hardware");
    io.ifConf(modemState, true);
  }

  io.setMode(m_modemState);
}

void CSerialPort::start()
{
  beginInt(1U, 115200);

#if defined(SERIAL_REPEATER) || defined(SERIAL_REPEATER_USART1)
  beginInt(3U, SERIAL_REPEATER_BAUD);
#endif
}

void CSerialPort::process()
{
  while (availableInt(1U)) {
    uint8_t c = readInt(1U);

    if (m_ptr == 0U) {
      if (c == MMDVM_FRAME_START) {
        // Handle the frame start correctly
        m_buffer[0U] = c;
        m_ptr = 1U;
        m_len = 0U;
      }
      else {
        m_ptr = 0U;
        m_len = 0U;
      }
    } else if (m_ptr == 1U) {
      // Handle the frame length
      m_len = m_buffer[m_ptr] = c;
      m_ptr = 2U;
    } else {
      // Any other bytes are added to the buffer
      m_buffer[m_ptr] = c;
      m_ptr++;

      // The full packet has been received, process it
      if (m_ptr == m_len) {
        uint8_t err = 2U;

        switch (m_buffer[2U]) {
          case MMDVM_GET_STATUS:
            getStatus();
            break;

          case MMDVM_GET_VERSION:
            getVersion();
            break;

          case MMDVM_SET_CONFIG:
            err = setConfig(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_SET_MODE:
            err = setMode(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_SET_FREQ:
            err = setFreq(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_SET_SYMBOLLVL:
            err = setSymbolLvlAdj(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_CAL_DATA:
            if (m_calState == STATE_DMRCAL || m_calState == STATE_DMRDMO1K) {
              err = calDMR.write(m_buffer + 3U, m_len - 3U);
            } else if (m_calState == STATE_RSSICAL || m_calState == STATE_INTCAL) {
              err = 0U;
            }
            if (err == 0U) {
              sendACK();
            } else {
              DEBUG2("Received invalid calibration data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_SEND_CWID:
            err = 5U;
            if (m_modemState == STATE_IDLE) {
              m_cwid_state = true;
              io.ifConf(STATE_CWID, true);
              err = cwIdTX.write(m_buffer + 3U, m_len - 3U);
            }
            if (err != 0U) {
              DEBUG2("Invalid CW Id data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DMR_DATA1:
          #if defined(DUPLEX)
            if (m_dmrEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR) {
                if (m_duplex)
                  err = dmrTX.writeData1(m_buffer + 3U, m_len - 3U);
              }
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DMR);
            } else {
              DEBUG2("Received invalid DMR data", err);
              sendNAK(err);
            }
          #endif
            break;

          case MMDVM_DMR_DATA2:
            if (m_dmrEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR) {
              #if defined(DUPLEX)
                if (m_duplex)
                  err = dmrTX.writeData2(m_buffer + 3U, m_len - 3U);
                else
                  err = dmrDMOTX.writeData(m_buffer + 3U, m_len - 3U);
              #else
                  err = dmrDMOTX.writeData(m_buffer + 3U, m_len - 3U);
              #endif
              }
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DMR);
            } else {
              DEBUG2("Received invalid DMR data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DMR_START:
          #if defined(DUPLEX)
            if (m_dmrEnable) {
              err = 4U;
              if (m_len == 4U) {
                if (m_buffer[3U] == 0x01U && m_modemState == STATE_DMR) {
                  if (!m_tx)
                    dmrTX.setStart(true);
                  err = 0U;
                } else if (m_buffer[3U] == 0x00U && m_modemState == STATE_DMR) {
                  if (m_tx)
                    dmrTX.setStart(false);
                  err = 0U;
                }
              }
            }
            if (err != 0U) {
              DEBUG2("Received invalid DMR start", err);
              sendNAK(err);
            }
          #endif
            break;

          case MMDVM_DMR_SHORTLC:
          #if defined(DUPLEX)
            if (m_dmrEnable)
              err = dmrTX.writeShortLC(m_buffer + 3U, m_len - 3U);
            if (err != 0U) {
              DEBUG2("Received invalid DMR Short LC", err);
              sendNAK(err);
            }
          #endif
            break;

          case MMDVM_DMR_ABORT:
          #if defined(DUPLEX)
            if (m_dmrEnable)
              err = dmrTX.writeAbort(m_buffer + 3U, m_len - 3U);
            if (err != 0U) {
              DEBUG2("Received invalid DMR Abort", err);
              sendNAK(err);
            }
          #endif
            break;

          case MMDVM_P25_HDR:
            if (m_p25Enable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                err = p25TX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_P25);
            } else {
              DEBUG2("Received invalid P25 header", err);
              sendNAK(err);
            }
            break;

          case MMDVM_P25_LDU:
            if (m_p25Enable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                err = p25TX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_P25);
            } else {
              DEBUG2("Received invalid P25 LDU", err);
              sendNAK(err);
            }
            break;

          case MMDVM_TRANSPARENT:
          case MMDVM_QSO_INFO:
            // Do nothing on the MMDVM.
            break;

#if defined(SERIAL_REPEATER) || defined(SERIAL_REPEATER_USART1)
          case MMDVM_SERIAL:
            writeInt(3U, m_buffer + 3U, m_len - 3U);
            break;
#endif

          default:
            // Handle this, send a NAK back
            sendNAK(1U);
            break;
        }

        m_ptr = 0U;
        m_len = 0U;
      }
    }
  }

  if (io.getWatchdog() >= 48000U) {
    m_ptr = 0U;
    m_len = 0U;
  }

#if defined(SERIAL_REPEATER) || defined(SERIAL_REPEATER_USART1)
  // Check for any incoming serial data from a device/screen on UART2
  //  !!Notice!! on powerup the Nextion screen dumps FF FF FF 88 FF FF FF to the serial port.
  while (availableInt(3U))
  {
    // read UART2
    m_serial_buffer[m_serial_len++] = readInt(3U);

    if (m_serial_len >=3 && (m_serial_buffer[m_serial_len - 3] == 0xFF) && (m_serial_buffer[m_serial_len - 2] == 0xFF) && (m_serial_buffer[m_serial_len - 1] == 0xFF))
    {
      if (m_serial_len > 3)
        serial.writeSerialRpt(m_serial_buffer, m_serial_len);

      //  if the last 3 bytes are FF's then the screen is done sending data so send the m_serial_buffer to serial.writeSerialRpt()
      //
      //  TODO - BG5HHP
      //   modem serial data repeat should be generic instead of coupling with the nextion protocol.
      //
      m_serial_len = 0U;
      continue;
    }

    if (m_serial_len == sizeof(m_serial_buffer))
    {
      // buffer overflow
      m_serial_len = 0U;
      continue;
    }
  }
#endif
}

#if defined(SERIAL_REPEATER) || defined(SERIAL_REPEATER_USART1)
void CSerialPort::writeSerialRpt(const uint8_t* data, uint8_t length)
{
  if (length == 0)
    return;

  uint8_t head[3];
  head[0U] = MMDVM_FRAME_START;
  head[1U] = length + 3;
  head[2U] = MMDVM_SERIAL;

  writeInt(1U, head, 3U);
  writeInt(1U, data, length, true);
}
#endif

void CSerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[40U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = slot ? MMDVM_DMR_DATA2 : MMDVM_DMR_DATA1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  writeInt(1U, reply, count);
}

void CSerialPort::writeDMRLost(bool slot)
{
  if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = slot ? MMDVM_DMR_LOST2 : MMDVM_DMR_LOST1;

  writeInt(1U, reply, 3);
}

void CSerialPort::writeP25Data(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[250U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_P25_LDU;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  writeInt(1U, reply, count);
}

void CSerialPort::writeP25Lost()
{
  if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_P25_LOST;

  writeInt(1U, reply, 3);
}

#if defined(SEND_RSSI_DATA)

void CSerialPort::writeRSSIData(const uint8_t* data, uint8_t length)
{
  if (m_calState != STATE_RSSICAL)
    return;

  uint8_t reply[30U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_RSSI_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  writeInt(1U, reply, count);
}

#endif

#if defined(ENABLE_DEBUG)
void CSerialPort::writeDebug(const char* text)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}

void CSerialPort::writeDebugI(const char* text, int32_t n1)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = ' ';

  i2str(&reply[count], 130U - count, n1);

  count += 9U;

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}

void CSerialPort::writeDebug(const char* text, int16_t n1)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG2;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}
#endif

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG3;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}

#if defined(ENABLE_DEBUG)
void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG4;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[count++] = (n3 >> 8) & 0xFF;
  reply[count++] = (n3 >> 0) & 0xFF;

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG5;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[count++] = (n3 >> 8) & 0xFF;
  reply[count++] = (n3 >> 0) & 0xFF;

  reply[count++] = (n4 >> 8) & 0xFF;
  reply[count++] = (n4 >> 0) & 0xFF;

  reply[1U] = count;

  writeInt(1U, reply, count, true);
}
#endif
