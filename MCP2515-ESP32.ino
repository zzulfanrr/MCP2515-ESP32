// #include "mcp2515.h"
// #include <SPI.h>
// #include "ThingSpeak.h"
// #include <TinyGPS++.h>


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
// void logPID();

// void setup() {
//   Serial.begin(115200);

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500kbps, MCP2515_8MHZ) == CAN_OK) {
//     // Serial.println("Setup Success!");
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

//   if (MCP2515.setLoopbackMode() == MODE_LOOPBACK) {
//     // Serial.println("Set to Loop Back Mode successfully!");
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     // Serial.println("Failed to set Loop Back Mode");
//   }

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);


//   // String csvcol = "#,engine_load,ect,stft,manifold_air_pressure,engine_rpm,vehicle_speed,iat,throttle_position,latitude,longitude";
//   String csvcol = "#,manifold_air_pressure,engine_rpm,vehicle_speed,iat";  //
//   Serial.println(csvcol);
// }

// // byte data[8] = { 0x02, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },

// byte dataSets[][8] = {
//   { 0x02, 0x01, 0x42, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },



//   { 0x02, 0x01, 0x04, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x05, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   // { 0x02, 0x01, 0x06, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x0B, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x0F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   { 0x02, 0x01, 0x11, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   // { 0x02, 0x01, 0x15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
//   // { 0x02, 0x01, 0x1F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
  
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

// void loop() {

//   //*************************************************************************************
//   //Send CAN Request Message

//     // Send CAN Request Message
//     byte sendStat = MCP2515.sendMessage(REQ_ID, 8, dataSets[currentDataSet]);

//     if (sendStat == CAN_OK) {
//       Serial.println("Request Message Sent Successfully!");
//     } else if (sendStat == CAN_TXTIMEOUT) {
//       Serial.println("Time Out Sending Request Message");
//     } else {
//       Serial.println("Error Sending Request Message");
//     }

//     currentDataSet = (currentDataSet + 1) % (sizeof(dataSets) / sizeof(dataSets[0]));
  

//   //*************************************************************************************
//   //Receive CAN Response Message
//   if (!digitalRead(MCP2515_INT)) {
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);

//     Serial.print("ID: 0x");
//     Serial.print(RX_ID, HEX);
//     Serial.print("       DLC: ");
//     Serial.print(dlc);
//     Serial.println(" Data:");

//     // Tampilkan data yang diterima
//     Serial.print("Received Data: ");
//     for (int i = 0; i < 8; i++) {
//       Serial.print("0x");
//       Serial.print(RX_Buff[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();

//     digitalWrite(INTERNAL_LED, HIGH);

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

//     static int count = 1;
//     // String csvlog = String(count++) + "," + String(manifold_air_pressure) + "," + String(engine_rpm) + "," + String(vehicle_speed) + "," + String(iat);

//     // Serial.println(csvlog);

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
//       Serial.print("Engine Load (%) = ");
//       Serial.println(engine_load);
//       break;

//     case PID_MANIFOLD_ABSOLUTE_PRESSURE:
//       manifold_air_pressure = a;
//       Serial.print("Manifold Absolute (Air) Pressure (kPa) = ");
//       Serial.println(manifold_air_pressure);
//       break;

//     case PID_ENGINE_COOLANT_TEMPERATURE:
//       ect = (a - 40);
//       Serial.print("Engine Coolant Temperature (°C) = ");
//       Serial.println(ect);
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
//       Serial.print("Intake Air Pressure (°C) = ");
//       Serial.println(iat);
//       break;

//     case PID_THROTTLE_POSITION:
//       throttle_position = (a / 2.55);
//       Serial.print("Throttle Position (%) = ");
//       Serial.println(throttle_position);
//       break;

//     case PID_O2_SENSOR_2:
//       // o2_sensor = (a / 100);
//       // o2_sensor2 = (((100 / 128) * B) - 100);
//       // Serial.print("Throttle Position (%) = ");
//       // Serial.println(throttle_position);
//       break;

//     // case PID_RUN_TIME:
//       runtime = (((256) * a) + b);
//       // Serial.print("Run Time (s) = ");
//       // Serial.println(runtime);
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