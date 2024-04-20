/*

mcp2515.h - Header File for MCP2515 Library

Zulfan Rachmadi
2024
*/

#ifndef _MCP2515_H_
#define _MCP2515_H_

#include <Arduino.h>
#include <SPI.h>

#include "mcp2515_def.h"  //MCP2515 Register Definition File

#define MAX_MESSAGE 8
static const uint32_t DEFAULT_SPI_CLOCK = 10000000;

class MCP2515 {
public:  //used externally, functions,

  MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK = DEFAULT_SPI_CLOCK, SPIClass *_SPI = nullptr);

  uint8_t begin(uint8_t IDmode, uint8_t speed, uint8_t clock);
  uint8_t def_Mask(uint8_t num, uint32_t valData);
  uint8_t def_Filter(uint8_t num, uint32_t valData);
  uint8_t def_Mask(uint8_t num, uint8_t ext, uint32_t valData);
  uint8_t def_Filter(uint8_t num, uint8_t ext, uint32_t valData);

  uint8_t sendMessage(uint32_t ID, uint8_t ext, uint8_t length, uint8_t *buffer);
  uint8_t sendMessage(uint32_t ID, uint8_t length, uint8_t *buffer);
  uint8_t getMessage(uint32_t *ID, uint8_t *ext, uint8_t *length, uint8_t buffer[]);
  uint8_t getMessage(uint32_t *ID, uint8_t *length, uint8_t buffer[]);
  uint8_t checkMessage(void);

  uint8_t setConfigMode();
  uint8_t setListenOnlyMode();
  uint8_t setSleepMode();
  uint8_t setLoopbackMode();
  uint8_t setNormalMode();

  void mcp2515_reset(void);
  uint8_t mcp2515_rate(const uint8_t CANSpeed, const uint8_t CANClock);

private:  //used internally, functions,


  uint8_t defCANMessage(uint32_t ID, uint8_t RTR, uint8_t ext, uint8_t length, uint8_t *Data);
  uint8_t transmitMessage();
  uint8_t receiveMessage();

  uint8_t mcp2515_readRegister(const uint8_t spi_reg);
  void mcp2515_readRegisterS(const uint8_t spi_reg, uint8_t values[], const uint8_t n);
  void mcp2515_setRegister(const uint8_t spi_reg, const uint8_t value);
  void mcp2515_setRegisterS(const uint8_t spi_reg, uint8_t values[], const uint8_t n);
  void mcp2515_modifyRegister(const uint8_t spi_reg, const uint8_t mask, const uint8_t data);

  uint8_t mcp2515_getStatus(void);


  uint8_t mcp2515_setMode(const uint8_t MCP2515_Mode);

  void mcp2515_def_id(uint8_t reg, const uint8_t ext, const uint32_t ID);
  void mcp2515_def_maskfilter(const uint8_t reg, const uint8_t ext, const uint32_t ID);

  void mcp2515_read_id(const uint8_t reg, uint8_t *ext, uint32_t *ID);

  void mcp2515_writeCANMessage(const uint8_t buffer_SIDH_reg);
  void mcp2515_readCANMessage(const uint8_t buffer_SIDH_reg);

  uint8_t mcp2515_getTXBuffer(uint8_t *txbuff);

  void mcp2515_initBMF(void);
  uint8_t mcp2515_init(const uint8_t CANIDMode, const uint8_t CANSpeed, const uint8_t CANClock);

private:  //used internally, define variable

  uint8_t MCP2515_CS;
  uint32_t SPI_Clock;
  SPIClass *MCP2515_SPI;

  uint8_t MCP2515_Mode;

  uint32_t m_nID;
  uint8_t m_nDLC;
  uint8_t m_nData[MAX_MESSAGE];
  uint8_t m_nRTR;
  uint8_t m_nExtFlag;
};

#endif  // MCP2515_H