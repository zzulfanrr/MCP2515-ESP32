// #include "mcp2515.h"
// #include <SPI.h>
// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>

// const char* ssid = "zzulfanrr iPhone";
// const char* password = "ByUbyTelkomsel";

// // const char* mqtt_server = "mqtt.hyperbase.in";
// // const int mqtt_port = 51883;
// // const char* mqtt_client_id = "CoolTermDevice";
// // const char* hyperbase_topic = "hyperbase-pg";

// // const char* carconnect_project_id = "018f8fad-ce7c-758f-b851-2f8c2b17cee5";  // ID Car Connect
// // const char* carconnect_token_id = "018f8fad-cebf-7cdb-befe-252516a459da";    // ID Token Sensor
// // const char* carconnect_token = "riyoGNmrfN3mCwJKLvnxJaNHYOBZ7C9iBH8I4LZp";
// // const char* car_collection_id = "018f8fad-cea1-735b-8891-1e8ed56334cc";  // ID Collection Cars
// // const char* obd_collection_id = "018f8fd5-776d-75fe-a6c8-4176a264c3b5";  // ID Collection OBD Data2
// // const char* car_id = "018f8fd6-09b0-78a4-aa2a-664128fe6aee";             // ID Car

// const char* mqtt_server = "103.76.129.93";
// const int mqtt_port = 31000;
// const char* mqtt_client_id = "CarConnect_1";
// const char* hyperbase_topic = "hyperbase-pg";

// const char* carconnect_project_id = "018e6934-7654-76de-a31c-af58e02d464d";  // ID Car Connect
// const char* carconnect_token_id = "018e693e-1823-7fd0-8eaf-91c88af6b8cc";    // ID Token Sensor
// const char* carconnect_token = "XSH2dDyiF8I5lt8duhso12Z2LbV4sHJCPfxmCmv1";
// const char* car_collection_id = "018e6937-16a9-727a-8f2d-512b97fa79ed";  // ID Collection Cars
// const char* obd_collection_id = "018e6934-d49a-76f6-b198-f54956cff23d";  // ID Collection OBD Data2
// const char* car_id = "018fbd47-f0eb-7a5c-b1de-0f66c8c11b9d";             // ID Car

// WiFiClient espClient;
// PubSubClient client(espClient);

// StaticJsonDocument<150> jsonUser;

// MCP2515 MCP2515(5);
// #define MCP2515_INT 21
// #define INTERNAL_LED 2

// #define LISTEN_ID 0x98DAF101
// #define RES_ID 0x98DA01F1
// // #define REQ_ID 0x7DF
// #define REQ_ID 0x98DB33F1

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// unsigned long prevTX = 0;
// const unsigned int invlTX = 1000;

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e);
// void carconnect();

// void setup() {
//   Serial.begin(115200);

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500KBPS, MCP2515_8MHZ) == CAN_OK) {
//     // Serial.println("Setup Success!");
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     // Serial.println("Setup Failed");
//   }
//   delay(500);

//   // if (MCP2515.setNormalMode() == MODE_NORMAL) {
//   //   // Serial.println("Set to Normal Mode successfully!");
//   // } else {
//   //   // Serial.println("Failed to set Normal Mode");
//   // }

//   if (MCP2515.setLoopbackMode() == MODE_LOOPBACK) {
//     Serial.println("Set to Loop Back Mode successfully!");
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     // Serial.println("Failed to set Loop Back Mode");
//   }

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);

//   WiFi.mode(WIFI_STA);

//   // Serial.print("Connecting to Wi-Fi");
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//     //   Serial.print(".");
//   }
//   Serial.println("Wi-Fi connected!");

//   client.setServer(mqtt_server, mqtt_port);
//   client.setBufferSize(1000);

//   jsonUser["collection_id"] = car_collection_id;
//   jsonUser["id"] = car_id;
// }

// byte dataSets[][8] = {
//   { 0x02, 0x01, 0x05, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //ECT
//   { 0x02, 0x01, 0x06, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //STFT
//   { 0x02, 0x01, 0x0B, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //MAP
//   { 0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //VSS
//   { 0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //RPM
//   { 0x02, 0x01, 0x0F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //IAT
//   { 0x02, 0x01, 0x11, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, //TPS
//   { 0x02, 0x01, 0x1F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA } //Run Time
// };
// int currentDataSet = 0;

// float engine_load = 0;
// int ect = 0;
// float stft = 0;
// int manifold_air_pressure = 0;
// int engine_rpm = 0;
// int throttle_position = 0;
// int vehicle_speed = 0;
// int iat = 0;
// int o2_sensor = 0;
// int o2_sensor2 = 0;
// int runtime = 0;
// float control_module_voltage = 0;

// enum PID_CODES : uint8_t {
//   PID_ENGINE_LOAD = 0x04,
//   PID_ENGINE_COOLANT_TEMPERATURE = 0x05,
//   PID_SHORT_TERM_FUEL_TRIM = 0x06,
//   PID_MANIFOLD_ABSOLUTE_PRESSURE = 0X0B,
//   PID_ENGINE_RPM = 0x0C,
//   PID_VEHICLE_SPEED = 0x0D,
//   PID_INTAKE_AIR_TEMPERATURE = 0X0F,
//   PID_THROTTLE_POSITION = 0x11,
//   PID_O2_SENSOR_2 = 0x15,
//   PID_RUN_TIME = 0x1F,
//   PID_CONTROL_MODULE_VOLTAGE = 0x42
// };

// uint8_t PID;
// uint8_t A;
// uint8_t B;
// uint8_t C;
// uint8_t D;
// uint8_t E;

// bool responseReceived[sizeof(dataSets) / sizeof(dataSets[0])] = { 0 };

// void reconnect() {
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     if (client.connect(mqtt_client_id, carconnect_token_id, carconnect_token)) {
//       Serial.println("connected");
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       // Wait 5 seconds before retrying
//       delay(5000);
//     }
//   }
// }

// void loop() {
//   if (!client.connected()) {  // Fixed the missing parenthesis and semicolon
//     reconnect();
//   }
//   client.loop();

//   //*************************************************************************************
//   //Send CAN Request Message
//   byte sendStat = MCP2515.sendMessage(REQ_ID, 8, dataSets[currentDataSet]);
//   currentDataSet = (currentDataSet + 1) % (sizeof(dataSets) / sizeof(dataSets[0]));

//   //*************************************************************************************
//   //Receive CAN Response Message
//   if (!digitalRead(MCP2515_INT)) {
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);

//     digitalWrite(INTERNAL_LED, HIGH);

//     Serial.print("Message Response:     ");
//     // char messageString[128];
//     // sprintf(messageString, "ID: 0x%.8lX  DLC: %1d  Data:", (RX_ID & 0x1FFFFFFF), dlc);
//     // Serial.print(messageString);
//     // for (byte i = 0; i < dlc; i++) {
//     //   sprintf(messageString, " 0x%.2X", RX_Buff[i]);
//     //   Serial.print(messageString);
//     // }
//     // Serial.print(",  ");

//     PID = RX_Buff[2];
//     A = RX_Buff[3];
//     B = RX_Buff[4];
//     C = RX_Buff[5];
//     D = RX_Buff[6];
//     E = RX_Buff[7];

//     decodePID(PID, A, B, C, D, E);
//     digitalWrite(INTERNAL_LED, LOW);
//   }
//   //*************************************************************************************
//   bool allResponsesReceived = true;
//   for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//     if (!responseReceived[i]) {
//       allResponsesReceived = false;
//       break;
//     }
//   }

//   if (allResponsesReceived) {
//     carconnect();  // Fixed the missing semicolon
//     memset(responseReceived, 0, sizeof(responseReceived));
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(250);
//     digitalWrite(INTERNAL_LED, LOW);
//     delay(250);
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(250);
//     digitalWrite(INTERNAL_LED, LOW);
//   }
// }

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
//   switch (static_cast<PID_CODES>(pid)) {
//     case PID_ENGINE_LOAD:
//       engine_load = (a / 2.55);
//        Serial.print("Manifold Absolute Pressure (kPa) = ");
//        Serial.println(manifold_air_pressure);
//       break;

//     case PID_MANIFOLD_ABSOLUTE_PRESSURE:
//       manifold_air_pressure = a;
//        Serial.print("Manifold Absolute Pressure (kPa) = ");
//        Serial.println(manifold_air_pressure);
//       break;

//     case PID_ENGINE_COOLANT_TEMPERATURE:
//       ect = (a - 40);
//        Serial.print("Engine Coolant Temperature (°C) = ");
//        Serial.println(ect);
//       break;

//     case PID_SHORT_TERM_FUEL_TRIM:
//       stft = ((a / 1.28) - 100);
//       Serial.print("STFT (%) = ");
//       Serial.println(stft);
//       break;

//     case PID_ENGINE_RPM:
//       engine_rpm = ((a * 256 + b) / 4);
//       Serial.print("Engine RPM (RPM) = ");
//       Serial.println(engine_rpm);
//       break;

//     case PID_VEHICLE_SPEED:
//       vehicle_speed = a;
//       Serial.print("Vehicle Speed (km/h) =");
//       Serial.println(vehicle_speed);
//       break;

//     case PID_INTAKE_AIR_TEMPERATURE:
//       iat = (a - 40);
//        Serial.print("Intake Air Pressure (°C) = ");
//        Serial.println(iat);
//       break;

//     case PID_THROTTLE_POSITION:
//       throttle_position = (a / 2.55);
//       Serial.print("Throttle Position (%) = ");
//       Serial.println(throttle_position);
//       break;

//     case PID_O2_SENSOR_2:
//       o2_sensor = (a / 100);
//       o2_sensor2 = (((100 / 128) * B) - 100);
//       Serial.print("Throttle Position (%) = ");
//       Serial.println(throttle_position);
//       break;

//     case PID_RUN_TIME:
//       runtime = (((256) * a) + b);
//       Serial.print("Run Time (s) = ");
//       Serial.println(runtime);
//       break;

//     case PID_CONTROL_MODULE_VOLTAGE:
//       control_module_voltage = (((256 * a) + b) / 1000.0);
//       Serial.print("Control Module Voltage (V) = ");
//       Serial.println(control_module_voltage);
//       break;

//     default:
//       break;
//   }

//   for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//     if (dataSets[i][2] == pid) {
//       responseReceived[i] = true;
//       break;
//     }
//   }
// }

// void carconnect() {
//   StaticJsonDocument<550> jsonSensor;
//   jsonSensor["car_id"] = car_id;
//   jsonSensor["catalyst_temperature"] = 0;
//   jsonSensor["engine_coolant_temperature"] = ect;
//   jsonSensor["engine_rpm"] = engine_rpm;
//   jsonSensor["fuel_system_status"] = "None";
//   jsonSensor["idle_time"] = 0;
//   jsonSensor["intake_air_temperature"] = iat;
//   jsonSensor["intake_manifold_pressure"] = manifold_air_pressure;
//   jsonSensor["long_term_fuel_trim"] = 0;
//   jsonSensor["mass_air_flow"] = 0;
//   jsonSensor["run_time"] = runtime;
//   jsonSensor["short_term_fuel_trim"] = stft;
//   jsonSensor["throttle_position"] = throttle_position;
//   jsonSensor["timestamp"] = 0;
//   jsonSensor["vehicle_speed"] = vehicle_speed;

//   StaticJsonDocument<800> jsonData;
//   jsonData["project_id"] = carconnect_project_id;
//   jsonData["token_id"] = carconnect_token_id;
//   jsonData["user"] = jsonUser;
//   jsonData["collection_id"] = obd_collection_id;
//   jsonData["data"] = jsonSensor;

//   String data = "";
//   serializeJson(jsonData, data);

//   //client.publish(hyperbase_topic, data.c_str());
//   if (client.publish(hyperbase_topic, data.c_str())) {
//     Serial.println("Data sent successfully");
//   } else {
//     Serial.println("Failed to send data");
//   }
// }
