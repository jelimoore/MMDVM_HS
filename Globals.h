/*
 *   Copyright (C) 2015,2016,2020 by Jonathan Naylor G4KLX
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

#if !defined(GLOBALS_H)
#define  GLOBALS_H

#if defined(STM32F10X_MD)
#include <stm32f10x.h>
#include "string.h"
#elif defined(STM32F4XX)
#include "stm32f4xx.h"
#include "string.h"
#elif defined(STM32F7XX)
#include "stm32f7xx.h"
#include "string.h"
#else
#include <Arduino.h>
#endif

enum DVM_STATE {
    STATE_IDLE = 0U,
    // DMR
    STATE_DMR = 1U,
    // Project 25
    STATE_P25 = 2U,

    // CW
    STATE_CW = 10U,

    // Calibration States
    STATE_P25_LF_CAL = 91U,
    STATE_P25_CAL_1K = 92U,

    STATE_DMR_DMO_CAL_1K = 93U,
    STATE_DMR_CAL_1K = 94U,
    STATE_DMR_LF_CAL = 95U,

    STATE_RSSI_CAL = 96U,

    STATE_P25_CAL = 97U,
    STATE_DMR_CAL = 98U,
    STATE_INT_CAL = 99U
};

const uint8_t  MARK_SLOT1 = 0x08U;
const uint8_t  MARK_SLOT2 = 0x04U;
const uint8_t  MARK_NONE  = 0x00U;

// Bidirectional Data pin (Enable Standard TX/RX Data Interface of ADF7021):
#define BIDIR_DATA_PIN

#include "IO.h"
#include "SerialPort.h"
#include "DMRDMORX.h"
#include "DMRDMOTX.h"

#if defined(DUPLEX)
#include "DMRIdleRX.h"
#include "DMRRX.h"
#include "DMRTX.h"
#endif

#include "P25RX.h"
#include "P25TX.h"
#include "CWIdTX.h"
#include "CalRSSI.h"
#include "CalDMR.h"
#include "Debug.h"
#include "Utils.h"
#include "I2CHost.h"

extern DVM_STATE m_modemState;
extern DVM_STATE m_calState;
extern DVM_STATE m_modemState_prev;

extern bool m_cwid_state;

extern uint8_t m_cwIdTXLevel;

extern uint32_t m_modeTimerCnt;

extern bool m_dmrEnable;
extern bool m_p25Enable;

extern bool m_duplex;

extern bool m_tx;
extern bool m_dcd;

extern CIO io;
extern CSerialPort serial;

extern uint8_t m_control;

#if defined(DUPLEX)
extern CDMRIdleRX dmrIdleRX;
extern CDMRRX dmrRX;
extern CDMRTX dmrTX;
#endif

extern CDMRDMORX dmrDMORX;
extern CDMRDMOTX dmrDMOTX;

extern CP25RX p25RX;
extern CP25TX p25TX;

extern CCalDMR  calDMR;

#if defined(SEND_RSSI_DATA)
extern CCalRSSI calRSSI;
#endif

extern CCWIdTX cwIdTX;

#if defined(STM32_I2C_HOST)
extern CI2CHost i2c;
#endif

#endif

