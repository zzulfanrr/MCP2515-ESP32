/*
#include <ArduinoJson.h>

// Other code...

void loop() {
  if (!digitalRead(MCP2515_INT)) {
    MCP2515.getMessage(&RX_ID, &dlc, RX_Buff);
    digitalWrite(INTERNAL_LED, HIGH);
    delay(500);
    digitalWrite(INTERNAL_LED, LOW);
    delay(500);

    // Create a JSON document
    StaticJsonDocument<200> doc;

    // Add values to the JSON document
    doc["Identifier"] = "0x" + String(RX_ID, HEX);
    doc["DLC"] = "0x" + String(dlc, HEX);

    JsonArray canData = doc.createNestedArray("CAN Data");
    for (byte i = 0; i < dlc; i++) {
      canData.add("0x" + String(RX_Buff[i], HEX));
    }

    // Extract Number of Bytes, Mode, PID, and Data (Hex)
    String numberOfBytesHex = "0x" + String(RX_Buff[0], HEX);
    String modeHex = "0x" + String(RX_Buff[1], HEX);
    String pidHex = "0x" + String(RX_Buff[2], HEX);

    doc["Number of Bytes"] = numberOfBytesHex;
    doc["Mode"] = modeHex;
    doc["PID"] = pidHex;

    // Extract Data (Hex)
    JsonArray dataHex = doc.createNestedArray("Data (Hex)");
    for (byte i = 3; i < (RX_Buff[0] + 3); i++) {
      dataHex.add("0x" + String(RX_Buff[i], HEX));
    }

    // Print the JSON document
    serializeJson(doc, Serial);
    Serial.println();
  }

  // Other code...
}
*/