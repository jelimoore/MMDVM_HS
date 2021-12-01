/*
 *   Copyright (C) 2015,2016,2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Mathis Schmieder DB9MAT
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

#if defined(STM32F10X_MD) || defined(STM32F4XX) || defined(STM32F7XX)

#include "Globals.h"

// Global variables
DVM_STATE m_modemState = STATE_IDLE;
DVM_STATE m_calState = STATE_IDLE;
DVM_STATE m_modemState_prev = STATE_IDLE;

bool m_cwid_state = false;

uint8_t m_cwIdTXLevel = 30;

uint32_t m_modeTimerCnt;

bool m_dmrEnable    = true;
bool m_p25Enable    = true;

bool m_duplex = false;

bool m_tx  = false;
bool m_dcd = false;


uint8_t    m_control;

#if defined(DUPLEX)
CDMRIdleRX dmrIdleRX;
CDMRRX     dmrRX;
CDMRTX     dmrTX;
#endif

CDMRDMORX  dmrDMORX;
CDMRDMOTX  dmrDMOTX;

CP25RX     p25RX;
CP25TX     p25TX;

CCalDMR    calDMR;

#if defined(SEND_RSSI_DATA)
CCalRSSI   calRSSI;
#endif

CCWIdTX    cwIdTX;

CSerialPort serial;
CIO io;

#if defined(STM32_I2C_HOST)
CI2CHost i2c;
#endif

void setup()
{
  serial.start();
}

void loop()
{
  io.process();
  
  serial.process();

  if (m_dmrEnable && m_modemState == STATE_DMR && m_calState == STATE_IDLE) {
#if defined(DUPLEX)
    if (m_duplex)
      dmrTX.process();
    else
      dmrDMOTX.process();
#else
    dmrDMOTX.process();
#endif
  }

  if (m_p25Enable && m_modemState == STATE_P25)
    p25TX.process();

  if (m_calState == STATE_DMR_CAL || m_calState == STATE_DMR_DMO_CAL_1K || m_calState == STATE_INT_CAL)
    calDMR.process();

#if defined(SEND_RSSI_DATA)
  if (m_calState == STATE_RSSI_CAL)
    calRSSI.process();
#endif

  if (m_modemState == STATE_IDLE)
    cwIdTX.process();
}

int main()
{
  setup();

  for (;;)
    loop();
}

#endif
