/*

mcp2515_def.h - Library for MCP2515 Register

Source (Datasheet):
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf (2021)

Zulfan Rachmadi
2024
*/

#ifndef MCP215DEF_H
#define MCP215DEF_H

#include <Arduino.h>
#include <SPI.h>

/*********************************************************************************************************
MCP2515 Register Addresses
 *********************************************************************************************************/

// MCP2515 Buffer

/ TX Buffer 0
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
 *  CAN Mode
 *********************************************************************************************************/

#define MODE_CONFIG		0x80
#define MODE_LISTEN		0x60
#define MODE_LOOPBACK	0x40
#define MODE_SLEEP		0x20
#define MODE_NORMAL		0x00

/*********************************************************************************************************
 *  CAN Configuration
 *********************************************************************************************************/

#define CANSTAT         0x0E
#define CANCTRL         0x0F
#define BFPCTRL         0x0C
#define TEC             0x1C
#define REC             0x1D
#define CNF3            0x28
#define CNF2            0x29
#define CNF1            0x2A
#define CANINTE         0x2B
#define CANINTF         0x2C
#define EFLG            0x2D
#define TXRTSCTRL       0x0D


/*********************************************************************************************************
 *  SPI
 *********************************************************************************************************/

#define CAN_RESET       0xC0
#define CAN_READ        0x03
#define CAN_WRITE       0x02
#define CAN_RTS         0x80
#define CAN_STATUS      0xA0
#define CAN_BIT_MODIFY  0x05  
#define CAN_RX_STATUS   0xB0
#define CAN_READ_BUFFER 0x90
#define CAN_LOAD_BUFFER 0X40  

#endif // MCP215DEF_H
