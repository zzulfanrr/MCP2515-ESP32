#include "mcp2515.h"
#include <SPI.h>
MCP2515 MCP2515(5);
#define MCP2515_INT 21
#define INTERNAL_LED 2
// Define CAN ID (Extended)
#define LISTEN_ID 0x98DAF101
#define RES_ID 0x98DA01F1
#define REQ_ID 0x98DB33F1
uint32_t RX_ID;
unsigned char dlc;
unsigned char RX_Buff[8];
char messageString[128];
unsigned long prevTX = 0;
const unsigned int invlTX = 1000; //Interval before timeout

void formatCANMessage();

void setup() {
  Serial.begin(115200);
  pinMode(INTERNAL_LED, OUTPUT);
  
  if (MCP2515.begin(MCP2515_STDEXT, CAN_500KBPS, MCP2515_8MHZ) == CAN_OK) { 
    Serial.println("Setup Success!");
  } else {
    Serial.println("Setup Failed");
  }
  delay(500);
  
  if (MCP2515.setNormalMode() == MODE_NORMAL) {
    Serial.println("Set to Normal Mode successfully!");
  } else {
    Serial.println("Failed to set Normal Mode");
  }
  
  pinMode(MCP2515_INT, INPUT);
}

byte data[8] = { 0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

void loop() {
  if (!digitalRead(MCP2515_INT)) {
    MCP2515.getMessage(&RX_ID, &dlc, RX_Buff);
    formatCANMessage(); 
    digitalWrite(INTERNAL_LED, HIGH);
    delay(500);
    digitalWrite(INTERNAL_LED, LOW);
    delay(500);
  }
  
  if ((millis() - prevTX) >= invlTX) {
    prevTX = millis();
    byte sendStat = MCP2515.sendMessage(REQ_ID, 8, data);
    if (sendStat == CAN_OK) {
      Serial.println("Message Sent Successfully!");
    } else if (sendStat == CAN_TXTIMEOUT) {
      Serial.println("TX Timeout");
    } else if (sendStat == CAN_MSGTIMEOUT) {
      Serial.println("Message Timeout");
    } else {
      Serial.println("Error Sending Message");
    }
  }
}

void formatCANMessage() {
  if (dlc == 8) {
    byte byteUsed = RX_Buff[0];
    byte response = RX_Buff[1];
    byte parameterID = RX_Buff[2];
    int rpm = RX_Buff[3]; 

    
  }
  else {
    Serial.println("Unsupported CAN Message (Invalid Data Length Code)");
    Serial.println("Data:  ");
    for (byte i = 0; i < dlc; i++) {
      sprintf(messageString, " 0x%.2X", RX_Buff[i]);
      Serial.print(messageString);
    }
  }
}
