// #include "mcp2515.h"
// #include <SPI.h>

// MCP2515 MCP2515(5);
// #define MCP2515_INT 21
// #define INTERNAL_LED 2

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// void setup() {
//   Serial.begin(115200);

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500kbps, MCP2515_8MHZ) == CAN_OK) {
//     Serial.println("Setup Success!");
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     Serial.println("Setup Failed");
//   }

//   if (MCP2515.setNormalMode() == MODE_NORMAL) {
//     Serial.println("Set to Normal Mode successfully!");
//   } else {
//     Serial.println("Failed to set Normal Mode");
//   }

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);
// }

// void loop() {
//   if (!digitalRead(MCP2515_INT)) {
//     // unsigned long receiveTime = micros();
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);

//     // digitalWrite(INTERNAL_LED, HIGH);

//     // Serial.print(receiveTime);
//     Serial.print("Received: ");
//     Serial.print("ID: 0x");
//     Serial.print(RX_ID, HEX);
//     Serial.print("  DLC: ");
//     Serial.print(dlc);
//     Serial.print("  Data: ");
    
//     for (int i = 0; i < dlc; i++) {
//       Serial.print("0x");
//       if (RX_Buff[i] < 0x10) Serial.print("0");
//       Serial.print(RX_Buff[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();

//     // digitalWrite(INTERNAL_LED, LOW);
//   }
// }