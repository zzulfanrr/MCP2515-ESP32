/*

mcp2515_def.h - Library for MCP2515 Register Definition

Source (Datasheet):
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf (2021)

Zulfan Rachmadi
2024
*/

#ifndef _MCP215DEF_H_
#define _MCP215DEF_H_

#include <Arduino.h>
#include <SPI.h>

//Define Time Out
#define timeoutval 2500

/*********************************************************************************************************/
// Define CAN ID Format
#define MCP2515_SIDH  0
#define MCP2515_SIDL  1
#define MCP2515_EID8  2
#define MCP2515_EID0  3

#define MCP2515_TXB_EXIDE_M     0x08                                        /* In TXBnSIDL                  */
#define MCP2515_DLC_MASK        0x0F                                        /* 4 LSBits                     */
#define MCP2515_RTR_MASK        0x40                                        /* (1<<6) Bit 6                 */

#define MCP2515_RXB_RX_ANY      0x60
#define MCP2515_RXB_RX_EXT      0x40
#define MCP2515_RXB_RX_STD      0x20
#define MCP2515_RXB_RX_STDEXT   0x00
#define MCP2515_RXB_RX_MASK     0x60
#define MCP2515_RXB_BUKT_MASK   (1<<2)

// Define CAN ID Format

#define MCP2515_STDEXT   0                                                  
#define MCP2515_STD      1                                                 
#define MCP2515_EXT      2                                          
#define MCP2515_ANY      3                                      


/*********************************************************************************************************/
// MESSAGE RECEPTION (4.0) 

#define MCP2515_TXB_TXBUFE    0x80
#define MCP2515_TXB_ABTF     0x40
#define MCP2515_TXB_MLOA      0x20
#define MCP2515_TXB_TXERR     0x10
#define MCP2515_TXB_TXREQ     0x08
#define MCP2515_TXB_TXIE      0x04
#define MCP2515_TXB_TXP10     0x03

#define MCP2515_BxBFS_MASK    0x30
#define MCP2515_BxBFE_MASK    0x0C
#define MCP2515_BxBFM_MASK    0x03

#define MCP2515_BxRTS_MASK    0x38
#define MCP2515_BxRTSM_MASK   0x07

#define RXIF_MASK_STATUS   (0x03)
#define RX0IF_STATUS       (1<<0)
#define RX1IF_STATUS       (1<<1)

/*********************************************************************************************************
MCP2515 Register Addresses
 *********************************************************************************************************/

/*********************************************************************************************************/
// MCP2515 Buffer

// TX Buffer 0
#define TXB0CTRL        0x30
#define TXB0SIDH        0x31
#define TXB0SIDL        0x32
#define TXB0EID8        0x33
#define TXB0EID0        0x34
#define TXB0DLC         0x35

#define TXB0D0          0x36
#define TXB0D1          0x37
#define TXB0D2          0x38
#define TXB0D3          0x39
#define TXB0D4          0x3A
#define TXB0D5          0x3B
#define TXB0D6          0x3C
#define TXB0D7          0x3D
                         
// TX Buffer 1
#define TXB1CTRL        0x40
#define TXB1SIDH        0x41
#define TXB1SIDL        0x42
#define TXB1EID8        0x43
#define TXB1EID0        0x44
#define TXB1DLC         0x45

#define TXB1D0          0x46
#define TXB1D1          0x47
#define TXB1D2          0x48
#define TXB1D3          0x49
#define TXB1D4          0x4A
#define TXB1D5          0x4B
#define TXB1D6          0x4C
#define TXB1D7          0x4D

// TX Buffer 2
#define TXB2CTRL        0x50
#define TXB2SIDH        0x51
#define TXB2SIDL        0x52
#define TXB2EID8        0x53
#define TXB2EID0        0x54
#define TXB2DLC         0x55

#define TXB2D0          0x56
#define TXB2D1          0x57
#define TXB2D2          0x58
#define TXB2D3          0x59
#define TXB2D4          0x5A
#define TXB2D5          0x5B
#define TXB2D6          0x5C
#define TXB2D7          0x5D
                         
// RX Buffer 0
#define RXB0CTRL        0x60
#define RXB0SIDH        0x61
#define RXB0SIDL        0x62
#define RXB0EID8        0x63
#define RXB0EID0        0x64
#define RXB0DLC         0x65

#define RXB0BUKT        0x04

#define RXB0D0          0x66
#define RXB0D1          0x67
#define RXB0D2          0x68
#define RXB0D3          0x69
#define RXB0D4          0x6A
#define RXB0D5          0x6B
#define RXB0D6          0x6C
#define RXB0D7          0x6D
                         
// RX Buffer 1
#define RXB1CTRL        0x70
#define RXB1SIDH        0x71
#define RXB1SIDL        0x72
#define RXB1EID8        0x73
#define RXB1EID0        0x74
#define RXB1DLC         0x75

#define RXB1D0          0x76
#define RXB1D1          0x77
#define RXB1D2          0x78
#define RXB1D3          0x79
#define RXB1D4          0x7A
#define RXB1D5          0x7B
#define RXB1D6          0x7C
#define RXB1D7          0x7D

/*********************************************************************************************************/
//MCP2515 RX Mask

//RX Mask 0
#define MASK0	  	0x20
#define RXM0SIDH	0x20
#define RXM0SIDL	0x21
#define RXM0EID8	0x22
#define RXM0EID0	0x23

//RX Mask 1
#define MASK1	  	0x24
#define RXM1SIDH	0x24
#define RXM1SIDL	0x25
#define RXM1EID8	0x26
#define RXM1EID0	0x27

/*********************************************************************************************************/

// Buffer Bit Masks
#define RXB0            0x00
#define RXB1            0x02
#define TXB0            0x01
#define TXB1            0x02
#define TXB2            0x04
#define TXB_ALL			TXB0 | TXB1 | TXB2


/*********************************************************************************************************/
// MCP2515 RX Filter

//RX Filter 0
#define FILTER0		0x00
#define RXF0SIDH	0x00
#define RXF0SIDL	0x01
#define RXF0EID8	0x02
#define RXF0EID0	0x03

//RX Filter 1
#define FILTER1		0x04
#define RXF1SIDH	0x04
#define RXF1SIDL	0x05
#define RXF1EID8	0x06
#define RXF1EID0	0x07

//RX Filter 2
#define FILTER2		0x08
#define RXF2SIDH	0x08
#define RXF2SIDL	0x09
#define RXF2EID8	0x0A
#define RXF2EID0	0x0B

//RX Filter 3
#define FILTER3		0x10
#define RXF3SIDH	0x10
#define RXF3SIDL	0x11
#define RXF3EID8	0x12
#define RXF3EID0	0x13

//RX Filter 4
#define FILTER4		0x14
#define RXF4SIDH	0x14
#define RXF4SIDL	0x15
#define RXF4EID8	0x16
#define RXF4EID0	0x17

//RX Filter 5
#define FILTER5		0x18
#define RXF5SIDH	0x18
#define RXF5SIDL	0x19
#define RXF5EID8	0x1A
#define RXF5EID0	0x1B

/*********************************************************************************************************
 *  RX Buffer 0 & 1
 *********************************************************************************************************/

#define MCP2515_RXBUF_0 (RXB0SIDH)
#define MCP2515_RXBUF_1 (RXB1SIDH)

/*********************************************************************************************************
 *  CAN Mode
 *********************************************************************************************************/

#define MODE_CONFIG		0x80
#define MODE_LISTENONLY		0x60
#define MODE_LOOPBACK	0x40
#define MODE_SLEEP		0x22
#define MODE_NORMAL		0x00

#define MODE_ONESHOT  0x08

#define MODE_MASK     0xE0 //Mode Mask defined also as Requesting Op Mode

/*********************************************************************************************************
 *  CAN Configuration
 *  CAN Error
 *  CAN Interrupt
 *********************************************************************************************************/

#define CANSTAT         0x0E // CAN Status 
#define CANCTRL         0x0F // CAN Control / Mode
#define BFPCTRL         0x0C // Bit Timing and Prescaler 

//Error Counter
#define TEC             0x1C // Transmit Error Counter
#define REC             0x1D // Receive Error Counter

//  Configuration Register : Control bit timing
#define CNF3            0x28
#define CNF2            0x29
#define CNF1            0x2A

#define CANINTE         0x2B  //CAN Interrupt Enable
#define CANINTF         0x2C  //CAN Interrupt Flag

#define EFLG            0x2D // Error Flag 

#define TXRTSCTRL       0x0D // Transmit Request and Transmission Status Control

/*********************************************************************************************************/
// CANINTF : CAN Interrupt Flag

#define RX0IF			0x01 // Receive Buffer 0 Flag
#define RX1IF			0x02 // Receive Buffer 1 Flag
#define TX0IF			0x04 // Transmit Buffer 0 Flag
#define TX1IF			0x08 // Transmit Buffer 1 Flag
#define TX2IF			0x10 // Transmit Buffer 2 Flag
#define ERRIF			0x20 // Error Flag (Bit/Frame)
#define WAKIF			0x40 // Wake Up Flag
#define MERRF			0x80 // Message Error Flag (Invalid Message Format)


/*********************************************************************************************************
 *  SPI MCP2515
 *********************************************************************************************************/

#define MCP2515_RESET           0xC0
#define MCP2515_READ            0x03
#define MCP2515_WRITE           0x02
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_BIT_MODIFY      0x05  
#define MCP2515_RX_STATUS       0xB0

//TX & RX Buffer
#define MCP2515_RXBUFFER  2
#define MCP2515_TXBUFFER  3

//Read RX Buffer
#define MCP2515_READ_RX0    0x90
#define MCP2515_READ_RX1    0x94

//Load TX Buffer
#define MCP2515_LOAD_TX0    0X40  
#define MCP2515_LOAD_TX1    0X42
#define MCP2515_LOAD_TX2    0X44  

// RTS : Request-to-Send
#define MCP2515_RTS_TX0     0x81
#define MCP2515_RTS_TX1     0x82
#define MCP2515_RTS_TX2     0x84
#define MCP2515_RTS_ALL     0x87

// MCP2515 Chip Select
#define MCP2515_CS_SELECT()    digitalWrite(MCP2515_CS, LOW)
#define MCP2515_CS_UNSELECT()  digitalWrite(MCP2515_CS, HIGH)

/*********************************************************************************************************
 *  CAN Rate and Speed
 *********************************************************************************************************/

// CAN BAUD RATE (Speed)
#define CAN_125kbps   1
#define CAN_250kbps   2 //ECU Simulator
#define CAN_500kbps   3
#define CAN_1000kbps  4

// CAN CLOCK

//Speed 8 MHz
#define MCP2515_8MHz_1000kbps_CFG1 (0x00)
#define MCP2515_8MHz_1000kbps_CFG2 (0xC0)
#define MCP2515_8MHz_1000kbps_CFG3 (0x80)

#define MCP2515_8MHz_500kbps_CFG1 (0x00)
#define MCP2515_8MHz_500kbps_CFG2 (0xD1)
#define MCP2515_8MHz_500kbps_CFG3 (0x81)

#define MCP2515_8MHz_250kbps_CFG1 (0x80)
#define MCP2515_8MHz_250kbps_CFG2 (0xE5)
#define MCP2515_8MHz_250kbps_CFG3 (0x83)

#define MCP2515_8MHz_125kbps_CFG1 (0x81)
#define MCP2515_8MHz_125kbps_CFG2 (0xE5)
#define MCP2515_8MHz_125kbps_CFG3 (0x83)

//Speed 16 MHz
#define MCP2515_16MHz_1000kbps_CFG1 (0x00)
#define MCP2515_16MHz_1000kbps_CFG2 (0xCA)
#define MCP2515_16MHz_1000kbps_CFG3 (0x81)

#define MCP2515_16MHz_500kbps_CFG1 (0x40)
#define MCP2515_16MHz_500kbps_CFG2 (0xE5)
#define MCP2515_16MHz_500kbps_CFG3 (0x83)

#define MCP2515_16MHz_250kbps_CFG1 (0x41)
#define MCP2515_16MHz_250kbps_CFG2 (0xE5)
#define MCP2515_16MHz_250kbps_CFG3 (0x83)

#define MCP2515_16MHz_125kbps_CFG1 (0x43)
#define MCP2515_16MHz_125kbps_CFG2 (0xE5)
#define MCP2515_16MHz_125kbps_CFG3 (0x83)

#define MCP2515_8MHZ 1
#define MCP2515_16MHZ 2
#define MCP2515_CLOCK 3

/*********************************************************************************************************
 *  CAN STATUS
 *********************************************************************************************************/

#define CANSTAT_OPMODE 0xE0;

/*********************************************************************************************************
 *  DEBUG AND ERROR
 *********************************************************************************************************/

// Debug Information MCP2515
#ifndef MCP2515_DEBUG
#define MCP2515_DEBUG 1
#endif

#define MCP2515_OK        0
#define MCP2515_FAIL      1
#define MCP2515_TXBUSY    2

//CAN Debug 

#define CAN_OK              0
#define CAN_FAIL            1
#define CAN_FAILINIT        2
#define CAN_FAILTX          3
#define CAN_ALLTXBUSY       4
#define CAN_MSGAVAILABLE    5
#define CAN_NOMSG           6
#define CAN_TXTIMEOUT       7
#define CAN_MSGTIMEOUT      8





#endif // MCP215DEF_H
