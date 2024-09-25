// #include "mcp2515.h"
// #include <SPI.h>

// #define MCP2515_INT 21
// #define INTERNAL_LED 2

// #define REQ_ID 0x98DB33F1
// // #define REQ_ID 0x7DF

// MCP2515 MCP2515(5);

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// unsigned long prevTX = 0;
// const unsigned int invlTX = 1000;

// void sendMode03Request();
// void processMode03Response();
// String convertDTC(uint16_t dtc);

// void setup() {
//   Serial.begin(115200);

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500KBPS, MCP2515_8MHZ) == CAN_OK) {
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     // Serial.println("Setup Failed");
//   }
//   delay(500);

//   if (MCP2515.setNormalMode() == MODE_NORMAL) {
//     // Serial.println("Set to Normal Mode successfully!");
//   } else {
//     // Serial.println("Failed to set Normal Mode");
//   }

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);
// }

// void loop() {
//   // Send Mode 03 request every interval
//   if ((millis() - prevTX) >= invlTX) {
//     prevTX = millis();
//     sendMode03Request();
//   }

//   // Receive CAN Response Message
//   if (!digitalRead(MCP2515_INT)) {
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);
    
//     // Print the full 8-byte message and its ID
//     Serial.print("ID: 0x");
//     Serial.print(RX_ID, HEX);
//     Serial.print(" DLC: ");
//     Serial.print(dlc);
//     Serial.print(" Data: ");
//     for (int i = 0; i < 8; i++) {
//       Serial.print("0x");
//       if (RX_Buff[i] < 0x10) {
//         Serial.print("0");
//       }
//       Serial.print(RX_Buff[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();

//     processMode03Response();
//   }
// }

// void sendMode03Request() {
//   byte mode03Request[8] = { 0x02, 0x03, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
//   byte sendStat = MCP2515.sendMessage(REQ_ID, 8, mode03Request);
// }

// void processMode03Response() {
//   if (RX_Buff[1] == 0x43) {
//     int numberOfDTCs = (RX_Buff[0] - 2) / 2; // each DTC is 2 bytes
//     for (int i = 0; i < numberOfDTCs; i++) {
//       int dtc = (RX_Buff[2 + i*2] << 8) | RX_Buff[3 + i*2];
//       String dtcString = convertDTC(dtc);
//       Serial.println("DTC: " + dtcString);
//     }
//   }
// }

// String convertDTC(uint16_t dtc) {
//   char dtcStr[6];
//   char firstChar;
  
//   switch ((dtc & 0xC000) >> 14) {
//     case 0: firstChar = 'P'; break;
//     case 1: firstChar = 'C'; break;
//     case 2: firstChar = 'B'; break;
//     case 3: firstChar = 'U'; break;
//   }

//   sprintf(dtcStr, "%c%04X", firstChar, (dtc & 0x3FFF));
//   return String(dtcStr);
// }
