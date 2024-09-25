// #include "mcp2515.h"
// #include <SPI.h>

// MCP2515 MCP2515(5);
// #define MCP2515_INT 21
// #define INTERNAL_LED 2

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
//   static int messageCount = 1;
  
//   byte messages[][8] = {
//     {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
//     {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},
//     {0x53, 0x6B, 0x72, 0x69, 0x70, 0x73, 0x69, 0x00},
//     {0x44, 0x54, 0x45, 0x54, 0x49, 0x00, 0x00, 0x00}
//   };

//   uint32_t ids[] = {0x100, 0x101, 0x110, 0x111};

//   for (int i = 0; i < 4; i++) {
//     unsigned long sendTime = micros();
//     byte sendStat = MCP2515.sendMessage(ids[i], 8, messages[i]);
    
//     if (sendStat == CAN_OK) {
//       // Serial.print(sendTime);
//       Serial.print("Message ");
//       Serial.print(messageCount++);
//       Serial.println(" Sent Successfully!");
//     } else {
//       Serial.println("Error Sending Message");
//     }
    
//     // delay(500); // Wait for 500ms before sending the next message
//   }

//   delay(5000); // Wait for 5 seconds before sending the next set of messages
// }