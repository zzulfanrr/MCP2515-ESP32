/*

mcp2515.cpp - Source File for Define Function in MCP2515 Library


Zulfan Rachmadi
2024
*/

#include "mcp2515.h"

/*********************************************************************************************************
 SPI transfer
transfer : send instruction to/from MCP2515
*********************************************************************************************************/
#define SPI_transfer MCP2515_SPI->transfer  //Read and Write Register, Sending Instructions
#define SPI_read() SPI_transfer(0x00)       //0x00 = data/instruction read

/*********************************************************************************************************
Private
*********************************************************************************************************/

/*********************************************************************************************************
* Function      : mcp2515_reset
* void MCP2515::mcp2515_reset(void)
* Description   : Reset MCP2515, Read-Write Register: MCP2515_RESET
 *********************************************************************************************************/

void MCP2515::mcp2515_reset(void) {
  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_RESET);
  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
  delay(5);
}

/*********************************************************************************************************
* Function      : mcp2515_readRegister
* uint8_t mcp2515_readRegister(const uint8_t reg)
* Description   : Read Register, Read-Write Register: MCP2515_READ, send Read Instruction to register & "spi_reg", Read the destined Register and save it to "res"
 *********************************************************************************************************/

uint8_t MCP2515::mcp2515_readRegister(const uint8_t spi_reg) {
  uint8_t res;

  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_READ);
  SPI_transfer(spi_reg);
  res = SPI_read();
  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();

  return res;
}

/*********************************************************************************************************
* Function      : mcp2515_readRegisterS
* void mcp2515_readRegisterS(const uint8_t reg, uint8_t values[], const uint8_t n)
* Description   : Read Register(s)
 *********************************************************************************************************/

void MCP2515::mcp2515_readRegisterS(const uint8_t spi_reg, uint8_t values[], const uint8_t n) {
  uint8_t i;

  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_READ);
  SPI_transfer(spi_reg);

  for (i = 0; i < n; i++)
    values[i] = SPI_read();

  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
}

/*********************************************************************************************************
* Function      : mcp2515_setRegister
* void mcp2515_setRegister(const uint8_t reg, const uint8_t value)
* Description   : Set Register (Delete All Bits - Change All Bits)
**********************************************************************************************************/

void MCP2515::mcp2515_setRegister(const uint8_t spi_reg, const uint8_t value) {
  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_WRITE);
  SPI_transfer(spi_reg);
  SPI_transfer(value);
  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
}

/*********************************************************************************************************
* Function      : mcp2515_setRegisterS
* void mcp2515_setRegister(const uint8_t reg, const uint8_t value)
* Description   : Set Register(s) Sequentially 
**********************************************************************************************************/


void MCP2515::mcp2515_setRegisterS(const uint8_t spi_reg, uint8_t values[], const uint8_t n) {
  uint8_t i;

  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_WRITE);
  SPI_transfer(spi_reg);

  for (i = 0; i < n; i++)
    SPI_transfer(values[i]);

  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
}

/*********************************************************************************************************
* Function name:           mcp2515_modifyRegister
* void mcp2515_modifyRegister(const uint8_t reg, const uint8_t mask, const uint8_t data)
* Descriptions:            Set specific bits of a register 
*********************************************************************************************************/
void MCP2515::mcp2515_modifyRegister(const uint8_t spi_reg, const uint8_t mask, const uint8_t data) {

  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_BIT_MODIFY);
  SPI_transfer(spi_reg);
  SPI_transfer(mask);
  SPI_transfer(data);
  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
}

/*********************************************************************************************************
* Function name:           mcp2515_getStatus
* uint8_t mcp2515_getStatus(void)
* Descriptions:          
*********************************************************************************************************/
uint8_t MCP2515::mcp2515_getStatus(void) {

  uint8_t res;
  MCP2515_SPI->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  MCP2515_CS_SELECT();
  SPI_transfer(MCP2515_READ_STATUS);
  res = SPI_read();
  MCP2515_CS_UNSELECT();
  MCP2515_SPI->endTransaction();
  return res;
}

/*********************************************************************************************************
* Function name:           mcp2515_setMode
* mcp2515_setMode(const uint8_t MCP2515_Mode)
* Descriptions:            Set Mode (Control Mode) for MCP2515
*********************************************************************************************************/

uint8_t MCP2515::mcp2515_setMode(const uint8_t MCP2515_Mode) {

  mcp2515_modifyRegister(CANCTRL, MODE_MASK, MCP2515_Mode);

  unsigned long endTime = millis() + 10;
  bool modeMatch = false;
  while (millis() < endTime) {
    uint8_t newMode = mcp2515_readRegister(CANSTAT);
    newMode &= CANSTAT_OPMODE;

    modeMatch = newMode == MCP2515_Mode;

    if (modeMatch) {
      break;
    }
  }
  return MCP2515_Mode;
}

//Set to Config Mode
uint8_t MCP2515::setConfigMode() {
  return mcp2515_setMode(MODE_CONFIG);
}

//Set to Listen Only Mode
uint8_t MCP2515::setListenOnlyMode() {
  return mcp2515_setMode(MODE_LISTENONLY);
}

//Set to Loopback Mode
uint8_t MCP2515::setLoopbackMode() {
  return mcp2515_setMode(MODE_LOOPBACK);
}

//Set to Sleep Mode
uint8_t MCP2515::setSleepMode() {
  return mcp2515_setMode(MODE_SLEEP);
}

//Set to Normal Mode
uint8_t MCP2515::setNormalMode() {
  return mcp2515_setMode(MODE_NORMAL);
}

/*********************************************************************************************************
* Function name:           mcp2515_rate
* mcp2515_rate(const uint8_t CANSpeed, const uint8_t CANClock)
* Descriptions:            Config Rate (Speed) and Clock
*********************************************************************************************************/

uint8_t MCP2515::mcp2515_rate(const uint8_t CANSpeed, const uint8_t CANClock) {

  uint8_t set, cfg1, cfg2, cfg3;
  /*  
      CAN BAUD RATE:
      CAN_125KBPS   1
      CAN_250KBPS   2 //ECU Simulator
      CAN_500KBPS   3 // CAN 2.0, MCP2515 Speed
      CAN_1000KBPS  4
      */
  set = 1;
  switch (CANClock) {
    case (MCP2515_8MHZ):  // MCP2515 Clock
      switch (CANSpeed) {
        case (CAN_125KBPS):
          cfg1 = MCP2515_8MHz_125kBPS_CFG1;
          cfg2 = MCP2515_8MHz_125kBPS_CFG2;
          cfg3 = MCP2515_8MHz_125kBPS_CFG3;
          break;

        case (CAN_250KBPS):
          cfg1 = MCP2515_8MHz_250kBPS_CFG1;
          cfg2 = MCP2515_8MHz_250kBPS_CFG2;
          cfg3 = MCP2515_8MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):
          cfg1 = MCP2515_8MHz_500kBPS_CFG1;
          cfg2 = MCP2515_8MHz_500kBPS_CFG2;
          cfg3 = MCP2515_8MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):
          cfg1 = MCP2515_8MHz_1000kBPS_CFG1;
          cfg2 = MCP2515_8MHz_1000kBPS_CFG2;
          cfg3 = MCP2515_8MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;
          break;
      }
      break;

    case (MCP2515_16MHZ):
      switch (CANSpeed) {
        case (CAN_125KBPS):
          cfg1 = MCP2515_16MHz_125kBPS_CFG1;
          cfg2 = MCP2515_8MHz_125kBPS_CFG2;
          cfg3 = MCP2515_8MHz_125kBPS_CFG3;
          break;

        case (CAN_250KBPS):
          cfg1 = MCP2515_16MHz_250kBPS_CFG1;
          cfg2 = MCP2515_16MHz_250kBPS_CFG2;
          cfg3 = MCP2515_16MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):
          cfg1 = MCP2515_16MHz_500kBPS_CFG1;
          cfg2 = MCP2515_16MHz_500kBPS_CFG2;
          cfg3 = MCP2515_16MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS):
          cfg1 = MCP2515_16MHz_1000kBPS_CFG1;
          cfg2 = MCP2515_16MHz_1000kBPS_CFG2;
          cfg3 = MCP2515_16MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          return MCP2515_FAIL;
          break;
      }
      break;

    default:
      set = 0;
      return MCP2515_FAIL;
      break;
  }
  if (set) {
    mcp2515_setRegister(CNF1, cfg1);
    mcp2515_setRegister(CNF2, cfg2);
    mcp2515_setRegister(CNF3, cfg3);
    return MCP2515_OK;
  } else {
    return MCP2515_FAIL;
  }
}

/*********************************************************************************************************
* Function name:           mcp2515_def_id
* mcp2515_def_id(uint8_t reg, const uint8_t ext, const uint32_t ID)
* Descriptions:            Define CAN ID
*********************************************************************************************************/

void MCP2515::mcp2515_def_id(uint8_t reg, const uint8_t ext, const uint32_t ID) {
  uint16_t CAN_ID = (uint16_t)(ID & 0x0FFFF);
  uint8_t buffer[4];

  if (ext == 1) {  //ext = Extended CAN ID Frame
    buffer[MCP2515_EID0] = (uint8_t)(CAN_ID & 0xFF);
    buffer[MCP2515_EID8] = (uint8_t)(CAN_ID >> 8);
    CAN_ID = (uint16_t)(ID >> 16);
    buffer[MCP2515_SIDL] = (uint8_t)(CAN_ID & 0x03);
    buffer[MCP2515_SIDL] += (uint8_t)((CAN_ID & 0x1C) << 3);
    buffer[MCP2515_SIDL] |= MCP2515_TXB_EXIDE_M;
    buffer[MCP2515_SIDH] = (uint8_t)(CAN_ID >> 5);
  } else {  //std = Standard CAN ID Frame
    buffer[MCP2515_SIDH] = (uint8_t)(CAN_ID >> 3);
    buffer[MCP2515_SIDL] = (uint8_t)((CAN_ID & 0x07) << 5);
    buffer[MCP2515_EID0] = 0;
    buffer[MCP2515_EID8] = 0;
  }
  mcp2515_setRegisterS(reg, buffer, 4);
}

/*********************************************************************************************************
* Function name:           mcp2515_def_maskfilter
* mcp2515_def_maskfilter(const uint8_t reg, const uint8_t ext, const uint32_t ID)
* Descriptions:            Define Mask and Filter
*********************************************************************************************************/

void MCP2515::mcp2515_def_maskfilter(const uint8_t reg, const uint8_t ext, const uint32_t ID) {

  uint16_t CAN_ID;
  CAN_ID = (uint16_t)(ID & 0x0FFFF);
  uint8_t buffer[4];

  if (ext == 1) {
    buffer[MCP2515_EID0] = (uint8_t)(CAN_ID & 0xFF);
    buffer[MCP2515_EID8] = (uint8_t)(CAN_ID >> 8);
    CAN_ID = (uint16_t)(ID >> 16);
    buffer[MCP2515_SIDL] = (uint8_t)(CAN_ID & 0x03);
    buffer[MCP2515_SIDL] += (uint8_t)((CAN_ID & 0x1C) << 3);
    buffer[MCP2515_SIDL] |= MCP2515_TXB_EXIDE_M;
    buffer[MCP2515_SIDH] = (uint8_t)(CAN_ID >> 5);
  } else {
    buffer[MCP2515_EID0] = (uint8_t)(CAN_ID & 0xFF);
    buffer[MCP2515_EID8] = (uint8_t)(CAN_ID >> 8);
    CAN_ID = (uint16_t)(ID >> 16);
    buffer[MCP2515_SIDL] = (uint8_t)((CAN_ID & 0x07) << 5);
    buffer[MCP2515_SIDH] = (uint8_t)(CAN_ID >> 3);
  }

  mcp2515_setRegisterS(reg, buffer, 4);
}

/*********************************************************************************************************
* Function name:           mcp2515_read_id
* mcp2515_read_id(const uint8_t reg, uint8_t* ext, uint32_t* ID)
* Descriptions:            Read CAN ID
*********************************************************************************************************/

void MCP2515::mcp2515_read_id(const uint8_t reg, uint8_t *ext, uint32_t *ID) {
  uint8_t buffer[4];

  *ext = 0;
  *ID = 0;

  mcp2515_readRegisterS(reg, buffer, 4);

  *ID = (buffer[MCP2515_SIDH] << 3) + (buffer[MCP2515_SIDL] >> 5);

  if ((buffer[MCP2515_SIDL] & MCP2515_TXB_EXIDE_M) == MCP2515_TXB_EXIDE_M) {  //extended ID
    *ID = (*ID << 2) + (buffer[MCP2515_SIDL] & 0x03);
    *ID = (*ID << 8) + buffer[MCP2515_EID8];
    *ID = (*ID << 8) + buffer[MCP2515_EID0];
    *ext = 1;
  }
}

/*********************************************************************************************************
* Function name:           mcp2515_writeCANMessage
* mcp2515_writeCANMessage(const uint8_t buffer_SIDH_reg)
* Descriptions:            Write CAN Message
*********************************************************************************************************/

void MCP2515::mcp2515_writeCANMessage(const uint8_t buffer_SIDH_reg) {
  uint8_t reg;
  reg = buffer_SIDH_reg;
  mcp2515_setRegisterS(reg + 5, m_nData, m_nDLC);  // write Data Bytes

  if (m_nRTR == 1)  // if RTR set bit in byte
    m_nDLC |= MCP2515_RTR_MASK;

  mcp2515_setRegister((reg + 4), m_nDLC);  // write RTR and DLC
  mcp2515_def_id(reg, m_nExtFlag, m_nID);  // write CAN ID
}

/*********************************************************************************************************
* Function name:           mcp2515_readCANMessage
* mcp2515_readCANMessage( const uint8_t buffer_SIDH_reg)
* Descriptions:            Read CAN Message
*********************************************************************************************************/
void MCP2515::mcp2515_readCANMessage(const uint8_t buffer_SIDH_reg) {
  uint8_t reg, ctrl;

  reg = buffer_SIDH_reg;

  mcp2515_read_id(reg, &m_nExtFlag, &m_nID);

  ctrl = mcp2515_readRegister(reg - 1);
  m_nDLC = mcp2515_readRegister(reg + 4);

  if (ctrl & 0x08)  //0x08 = RXBnCTRL for RTR
    m_nRTR = 1;
  else
    m_nRTR = 0;

  m_nDLC &= MCP2515_DLC_MASK;
  mcp2515_readRegisterS(reg + 5, &(m_nData[0]), m_nDLC);
}

/*********************************************************************************************************
* Function name:           mcp2515_getTXBuffer
* mcp2515_initBMF(void)
* Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/

uint8_t MCP2515::mcp2515_getTXBuffer(uint8_t *txbuff) {
  uint8_t res, i, ctrlval;
  uint8_t ctrlreg[MCP2515_TXBUFFER] = { TXB0CTRL, TXB1CTRL, TXB2CTRL };

  res = MCP2515_TXBUSY;
  *txbuff = 0x00;

  for (i = 0; i < MCP2515_TXBUFFER; i++) {
    ctrlval = mcp2515_readRegister(ctrlreg[i]);
    if ((ctrlval & MCP2515_TXB_TXREQ) == 0) {
      *txbuff = ctrlreg[i] + 1;
      res = MCP2515_OK;
      return res;
    }
  }
  return res;
}

/*********************************************************************************************************
* Function name:           mcp2515_initBMF
* mcp2515_initBMF(void)
* Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/

void MCP2515::mcp2515_initBMF(void) {
  uint8_t i, a1, a2, a3;

  uint8_t std = 0;
  uint8_t ext = 1;

  uint32_t valMask = 0x00;
  uint32_t valFilter = 0x00;

  mcp2515_def_maskfilter(RXM0SIDH, ext, valMask);  //Both Mask 0
  mcp2515_def_maskfilter(RXM1SIDH, ext, valMask);


  mcp2515_def_maskfilter(RXF0SIDH, ext, valFilter);
  mcp2515_def_maskfilter(RXF1SIDH, std, valFilter);
  mcp2515_def_maskfilter(RXF2SIDH, ext, valFilter);
  mcp2515_def_maskfilter(RXF3SIDH, std, valFilter);
  mcp2515_def_maskfilter(RXF4SIDH, ext, valFilter);
  mcp2515_def_maskfilter(RXF5SIDH, std, valFilter);


  a1 = TXB0CTRL;
  a2 = TXB1CTRL;
  a3 = TXB2CTRL;

  for (i = 0; i < 14; i++) { /* in-buffer loop               */
    mcp2515_setRegister(a1, 0);
    mcp2515_setRegister(a2, 0);
    mcp2515_setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  mcp2515_setRegister(RXB0CTRL, 0);
  mcp2515_setRegister(RXB1CTRL, 0);
}

/*********************************************************************************************************
* Function name:           mcp2515_init
* mcp2515_init(void)
* Descriptions:            Initialize Controller (MCP2515)
*********************************************************************************************************/

uint8_t MCP2515::mcp2515_init(const uint8_t CANIDMode, const uint8_t CANSpeed, const uint8_t CANClock) {

  mcp2515_reset();

  if (setConfigMode() == MODE_CONFIG) {
    Serial.println("Set Config Mode Success!");

    mcp2515_rate(CANSpeed, CANClock);

    Serial.println("Init BMF and ID Mode");
    mcp2515_initBMF();

    mcp2515_setRegister(CANINTE, RX0IF | RX1IF);

    mcp2515_setRegister(BFPCTRL, MCP2515_BxBFS_MASK | MCP2515_BxBFE_MASK);

    mcp2515_setRegister(TXRTSCTRL, 0x00);

    switch (CANIDMode) {
      case (MCP2515_ANY):
        mcp2515_modifyRegister(RXB0CTRL,
                               MCP2515_RXB_RX_MASK | MCP2515_RXB_BUKT_MASK,
                               MCP2515_RXB_RX_ANY | MCP2515_RXB_BUKT_MASK);
        mcp2515_modifyRegister(RXB1CTRL, MCP2515_RXB_RX_MASK,
                               MCP2515_RXB_RX_ANY);
        break;

      case (MCP2515_STDEXT):
        mcp2515_modifyRegister(RXB0CTRL,
                               MCP2515_RXB_RX_MASK | MCP2515_RXB_BUKT_MASK,
                               MCP2515_RXB_RX_STDEXT | MCP2515_RXB_BUKT_MASK);
        mcp2515_modifyRegister(RXB1CTRL, MCP2515_RXB_RX_MASK,
                               MCP2515_RXB_RX_STDEXT);
        break;

      default:

#if MCP2515_DEBUG
        Serial.println("Setting ID Mode Failure");
#endif
        return MCP2515_FAIL;
        break;
    }

    if (setLoopbackMode() == MODE_LOOPBACK) {
      Serial.println("Set Loop Back Mode Success!");
    } else {
      Serial.println("Set Loop Back Mode Failed!");
    }
    return MCP2515_OK;
  } else {
    Serial.println("Set Config Mode Failed!");
    return MCP2515_FAIL;
  }
}

/*********************************************************************************************************
Public
*********************************************************************************************************/

/*********************************************************************************************************
* Function      : MCP2515_CS
* mcp2515_CS(uint8_t ChipSelect);
* Description   : Declare CAN Class and Chip Select (SPI) pin.
 *********************************************************************************************************/

MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK, SPIClass *_SPI) {
  MCP2515_CS = _CS;
  SPI_Clock = _SPI_CLOCK;

  MCP2515_CS_UNSELECT();
  pinMode(MCP2515_CS, OUTPUT);
  if (_SPI != nullptr) {
    MCP2515_SPI = _SPI;
  } else {
    MCP2515_SPI = &SPI;
    MCP2515_SPI;
  }
}

/*********************************************************************************************************
* Function      : begin
* uint8_t begin(uint8_t IDmode, uint8_t speed, uint8_t clock);
* Description   : Declare Contoller initialization parameters (mode, speed, clock)
 *********************************************************************************************************/

uint8_t MCP2515::begin(uint8_t IDmode, uint8_t speed, uint8_t clock) {
  uint8_t res;

  MCP2515_SPI->begin();
  res = mcp2515_init(IDmode, speed, clock);
  if (res == MCP2515_FAIL) {
    return CAN_FAILINIT;
  }
  return CAN_OK;
}

/*********************************************************************************************************
* Function      : def_Mask
uint8_t def_Mask(uint8_t num, uint8_t ext, uint32_t valData)
* Description   : Initializer for Mask
 *********************************************************************************************************/

uint8_t MCP2515::def_Mask(uint8_t num, uint8_t ext, uint32_t valData) {

  setConfigMode();

  Serial.println("Set Config Mode Success!");

  Serial.println("Initializing Mask...");

  uint8_t buffer[4];
  mcp2515_def_id(buffer[0], ext, valData);

  uint8_t reg;
  switch (num) {
    case 0:
      reg = RXM0SIDH;
      break;

    case 1:
      reg = RXM1SIDH;
      break;

    default:
      return MCP2515_FAIL;
  }
  mcp2515_setRegisterS(reg, buffer, 4);
  Serial.println("Initializing Mask Success!");

  return MCP2515_OK;
}

/*********************************************************************************************************
* Function      : def_Filter
* uint8_t def_Filter(uint8_t num, uint8_t ext, uint32_t valData)
* Description   : Initializer for Filter
 *********************************************************************************************************/
uint8_t MCP2515::def_Filter(uint8_t num, uint8_t ext, uint32_t valData) {

  setConfigMode();
  Serial.println("Set Config Mode Success!");

  Serial.println("Initializing Filter...");

  uint8_t reg;
  switch (num) {
    case 0:
      reg = RXF0SIDH;
      break;

    case 1:
      reg = RXF1SIDH;
      break;

    case 2:
      reg = RXF2SIDH;
      break;

    case 3:
      reg = RXF3SIDH;
      break;

    case 4:
      reg = RXF4SIDH;
      break;

    case 5:
      reg = RXF5SIDH;
      break;

    default:
      return MCP2515_FAIL;
  }
  uint8_t buffer[4];
  mcp2515_def_id(buffer[0], ext, valData);
  mcp2515_setRegisterS(reg, buffer, 4);
  Serial.println("Initializing Filter Success!");

  return MCP2515_OK;
}

/*********************************************************************************************************
* Function      : defCANMessage
* defCANMessage(uint32_t ID, uint8_t RTR, uint8_t ext, uint8_t length, uint8_t *Data)
* Description   : Define CAN Message: DLC, ID, Data, RTR
 *********************************************************************************************************/

uint8_t MCP2515::defCANMessage(uint32_t ID, uint8_t RTR, uint8_t ext, uint8_t length, uint8_t *Data) {
  int i = 0;
  m_nID = ID;
  m_nRTR = RTR;
  m_nExtFlag = ext;
  m_nDLC = length;
  for (i = 0; i < MAX_MESSAGE; i++)
    m_nData[i] = *(Data + i);

  return MCP2515_OK;
}

/*********************************************************************************************************
* Function      : transmitMessage()
* transmitMessage()
* Description   : Transmit CAN Message to TX Free Buffer
 *********************************************************************************************************/

uint8_t MCP2515::transmitMessage() {
  uint8_t res, res_alt, txbuff;
  uint32_t timeout, temp;

  temp = micros();

  do {
    res = mcp2515_getTXBuffer(&txbuff);
    timeout = micros() - temp;
  } while (res == MCP2515_TXBUSY && (timeout < timeoutval));

  if (timeout >= timeoutval) {
    return CAN_TXTIMEOUT;
  }
  timeout = 0;
  mcp2515_writeCANMessage(txbuff);
  mcp2515_modifyRegister(txbuff - 1, MCP2515_TXB_TXREQ, MCP2515_TXB_TXREQ);

  temp = micros();

  do {
    res_alt = mcp2515_readRegister(txbuff - 1);
    res_alt = res_alt & 0x08;  //MCP2515_TXB_TXREQ
    timeout = micros() - temp;
  } while (res_alt && (timeout < timeoutval));

  if (timeout >= timeoutval) {
    return CAN_MSGTIMEOUT;
  }
  return CAN_OK;
}

/*********************************************************************************************************
* Function      : sendMessage()
* sendMessage(uint32_t I, uint8_t length, uint8 *buffer)
* Description   : Check Frame for Extended ID/RTR, then send the Message
 *********************************************************************************************************/

uint8_t MCP2515::sendMessage(uint32_t ID, uint8_t ext, uint8_t length, uint8_t *buffer) {
  uint8_t res;
  uint8_t RTR = 0;
  /*
     if((ID & 0x80000000) == 0x80000000) //Check Extended ID
        ext = 1;
    if((ID & 0x40000000) == 0x40000000) //Check RTR Frame
        RTR = 1;
 
    */
  defCANMessage(ID, RTR, ext, length, buffer);
  res = transmitMessage();

  return res;
}

/*********************************************************************************************************
* Function      : sendMessage()
* sendMessage(uint32_t I, uint8_t length, uint8 *buffer)
* Description   : Check Frame for Extended ID/RTR, then send the Message
 *********************************************************************************************************/

uint8_t MCP2515::sendMessage(uint32_t ID, uint8_t length, uint8_t *buffer) {
  uint8_t res;
  uint8_t ext = 0;
  uint8_t RTR = 0;

  if ((ID & 0x80000000) == 0x80000000)  //Check Extended ID
    ext = 1;
  if ((ID & 0x40000000) == 0x40000000)  //Check RTR Frame
    RTR = 1;


  defCANMessage(ID, RTR, ext, length, buffer);
  res = transmitMessage();

  return res;
}

/*********************************************************************************************************
* Function      : receiveMessage()
* receiveMessage()
* Description   : Receive Message Transmission
 *********************************************************************************************************/

uint8_t MCP2515::receiveMessage() {
  uint8_t status, res;

  status = mcp2515_getStatus();

  if (status & RX0IF_STATUS) {
    mcp2515_readCANMessage(MCP2515_RXBUF_0);
    mcp2515_modifyRegister(CANINTF, RX0IF, 0);
    res = CAN_OK;
  }

  else if (status & RX1IF_STATUS) {
    mcp2515_readCANMessage(MCP2515_RXBUF_1);
    mcp2515_modifyRegister(CANINTF, RX1IF, 0);
    res = CAN_OK;
  } else {
    res = CAN_NOMSG;
  }

  return res;
}

/*********************************************************************************************************
* Function      : getMessage()
* getMessage()
* Description   : Get Message
 *********************************************************************************************************/

uint8_t MCP2515::getMessage(uint32_t *ID, uint8_t *ext, uint8_t *length, uint8_t buffer[]) {
  if (receiveMessage() == CAN_NOMSG) {
    return CAN_NOMSG;
  }

  *ID = m_nID;
  *length = m_nDLC;
  *ext = m_nExtFlag;

  for (int i = 0; i < m_nDLC; i++)
    buffer[i] = m_nData[i];

  return CAN_OK;
}

/*********************************************************************************************************
* Function      : getMessage()
* getMessage()
* Description   : Get Message
 *********************************************************************************************************/

uint8_t MCP2515::getMessage(uint32_t *ID, uint8_t *length, uint8_t buffer[]) {
  if (receiveMessage() == CAN_NOMSG) {
    return CAN_NOMSG;
  }

  if (m_nExtFlag)
    m_nID |= 0x80000000;  // Check if Extended Frame

  if (m_nRTR)
    m_nID |= 0x40000000;  //Check if RTR Frame

  *ID = m_nID;
  *length = m_nDLC;

  for (int i = 0; i < m_nDLC; i++)
    buffer[i] = m_nData[i];

  return CAN_OK;
}

/*********************************************************************************************************
* Function      : checkMessage()
* checkMessage()
* Description   : check Message
 *********************************************************************************************************/

uint8_t MCP2515::checkMessage(void) {
  uint8_t res;
  res = mcp2515_getStatus();
  if (res & RXIF_MASK_STATUS)
    return CAN_MSGAVAILABLE;
  else
    return CAN_NOMSG;
}
