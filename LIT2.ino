// #include "mcp2515.h"
// #include <SPI.h>
// #include "ThingSpeak.h"
// #include <TinyGPS++.h>

// MCP2515 MCP2515(5);
// #define MCP2515_INT 21
// #define INTERNAL_LED 2

// #define LISTEN_ID 0x98DAF101
// #define RES_ID 0x98DA01F1
// #define REQ_ID 0x98DB33F1

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// unsigned long prevTX = 0;
// const unsigned int invlTX = 1000;

// bool CarOn = false;
// bool EngineOn = false;
// bool isFirstRequest = true;

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e);
// void logPID();

// void setup() {
//   Serial.begin(115200);

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500kbps, MCP2515_8MHZ) == CAN_OK) {
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     Serial.println("Setup Failed");
//   }
//   delay(500);

//   if (MCP2515.setNormalMode() == MODE_NORMAL) {
//     // Serial.println("Set to Normal Mode successfully!");
//   } else {
//     Serial.println("Failed to set Normal Mode");
//   }

//   MCP2515.mcp2515_setSleepWakeup(1);

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);

//   String csvcol = "#,manifold_absolute_pressure,enginee_speed,vehicle_speed,intake_air_temperature";
//   Serial.println(csvcol);
// }

// byte dataSets[][8] = {
//   { 0x02, 0x01, 0x42, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Control Module Voltage
//   { 0x02, 0x01, 0x0B, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Manifold Absolute Pressure
//   { 0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Engine Speed
//   { 0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Vehicle Speed
//   { 0x02, 0x01, 0x0F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Intake Air Temperature
// };

// int currentDataSet = 0;

// float engine_load = 0;
// int ect = 0;
// float stft = 0;
// int manifold_absolute_pressure = 0;
// int enginee_speed = 0;
// int throttle_position = 0;
// int vehicle_speed = 0;
// int intake_air_temperature = 0;
// int o2_sensor = 0;
// int o2_sensor2 = 0;
// int runtime = 0;
// float control_module_voltage = 0;

// enum PID_CODES : uint8_t {
//   PID_ENGINE_LOAD = 0x04,
//   PID_ENGINE_COOLANT_TEMPERATURE = 0x05,
//   PID_SHORT_TERM_FUEL_TRIM = 0x06,
//   PID_MANIFOLD_ABSOLUTE_PRESSURE = 0X0B,
//   PID_enginee_speed = 0x0C,
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
//   unsigned long currentMillis = millis();

//   // Kirim pesan pertama jika belum dikirim
//   if (isFirstRequest) {
//     byte sendStat = MCP2515.sendMessage(REQ_ID, 8, dataSets[0]);
//     if (sendStat == CAN_OK) {
//       Serial.println("Request Message Sent Successfully!");
//       isFirstRequest = false;  // Tandai bahwa permintaan pertama telah dikirim
//     } else if (sendStat == CAN_TXTIMEOUT) {
//       Serial.println("Time Out Sending Request Message");
//     } else {
//       Serial.println("Error Sending Request Message");
//     }
//     prevTX = currentMillis;  // Update waktu terakhir pengiriman
//   }

//   // Tunggu respons dari CMV
//   if (!digitalRead(MCP2515_INT)) {
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);
//     if (RX_ID == 0x98DAF10E) {
//       PID = RX_Buff[2];
//       A = RX_Buff[3];
//       B = RX_Buff[4];
//       C = RX_Buff[5];
//       D = RX_Buff[6];
//       E = RX_Buff[7];
//       decodePID(PID, A, B, C, D, E);
//     }
//   }

//   if (CarOn && (currentMillis - prevTX >= invlTX)) {
//     prevTX = currentMillis;

//     for (int i = 1; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//       byte sendStat = MCP2515.sendMessage(REQ_ID, 8, dataSets[i]);
//       if (sendStat == CAN_OK) {
//         Serial.println("Request Message Sent Successfully!");
//       } else if (sendStat == CAN_TXTIMEOUT) {
//         Serial.println("Time Out Sending Request Message");
//       } else {
//         Serial.println("Error Sending Request Message");
//       }

//       delay(1000);  // Tunggu sebentar sebelum mengirim pesan berikutnya
//     }
//     //Receive CAN Response Message
//     if (!digitalRead(MCP2515_INT)) {
//       MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);
//       if (RX_ID == 0x98DAF10E) {
//         // ID & DLC
//         Serial.print("ID: 0x");
//         Serial.print(RX_ID, HEX);
//         Serial.print("       DLC: ");
//         Serial.print(dlc);
//         Serial.println(" Data:");
//         // DLC
//         Serial.print("Received Data: ");
//         for (int i = 0; i < 8; i++) {
//           Serial.print("0x");
//           Serial.print(RX_Buff[i], HEX);
//           Serial.print(" ");
//         }
//         Serial.println();

//         PID = RX_Buff[2];
//         A = RX_Buff[3];
//         B = RX_Buff[4];
//         C = RX_Buff[5];
//         D = RX_Buff[6];
//         E = RX_Buff[7];
//         // Decode
//         decodePID(PID, A, B, C, D, E);
//       }
//     }

//     // Cek jika semua respons telah diterima
//     bool allResponsesReceived = true;
//     for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//       if (!responseReceived[i]) {
//         allResponsesReceived = false;
//         break;
//       }
//     }

//     if (allResponsesReceived) {
//       static int count = 1;
//       String csvlog = String(count++) + "," + String(manifold_absolute_pressure) + "," + String(enginee_speed) + "," + String(vehicle_speed) + "," + String(intake_air_temperature);
//       Serial.println(csvlog);
//       memset(responseReceived, 0, sizeof(responseReceived));
//       digitalWrite(INTERNAL_LED, HIGH);
//       delay(250);
//       digitalWrite(INTERNAL_LED, LOW);
//       delay(250);
//       digitalWrite(INTERNAL_LED, HIGH);
//       delay(250);
//       digitalWrite(INTERNAL_LED, LOW);

//       // Setelah log, siap untuk permintaan berikutnya
//       isFirstRequest = true;
//     }
//   } else {
//     digitalWrite(INTERNAL_LED, LOW);
//     if (MCP2515.setSleepMode() == MODE_SLEEP) {
//       Serial.println("MCP2515 Mode Sleep!");
//     } else {
//       Serial.println("MCP2515 Failed Mode Sleep!");
//     }
//     delay(1000);  // Wait to ensure MCP2515 is in sleep mode

//     // Configure ESP32 to wake up after 5 seconds
//     esp_sleep_enable_timer_wakeup(5000000);  // 5 seconds in microseconds

//     // Optionally enable wakeup on MCP2515 interrupt (if connected to GPIO)
//     esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 0);  // Wake up if MCP2515 INT goes low

//     Serial.println("ESP32 Deep Sleep: 5 seconds...");

//     // Go to deep sleep
//     esp_deep_sleep_start();
//   }
// }

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
//   switch (static_cast<PID_CODES>(pid)) {
//     case PID_MANIFOLD_ABSOLUTE_PRESSURE:
//       manifold_absolute_pressure = a;
//       Serial.print("Manifold Absolute (Air) Pressure (kPa) = ");
//       Serial.println(manifold_absolute_pressure);
//       break;

//     case PID_enginee_speed:
//       enginee_speed = ((a * 256 + b) / 4);
//       Serial.print("Engine Speed (RPM) = ");
//       Serial.println(enginee_speed);

//       if (enginee_speed == 0) {
//         CarOn = true;
//         EngineOn = false;
//         Serial.println("Car is OFF [RPM Flag]");
//       }

//       break;

//     case PID_VEHICLE_SPEED:
//       vehicle_speed = a;
//       Serial.print("Vehicle Speed (km/h) = ");
//       Serial.println(vehicle_speed);
//       break;

//     case PID_INTAKE_AIR_TEMPERATURE:
//       intake_air_temperature = (a - 40);
//       Serial.print("Intake Air Pressure (°C) = ");
//       Serial.println(intake_air_temperature);
//       break;

//     case PID_CONTROL_MODULE_VOLTAGE:
//       control_module_voltage = (((256 * a) + b) / 1000.0);
//       Serial.print("Control Module Voltage (V) = ");
//       Serial.println(control_module_voltage);

//       if (control_module_voltage > 12.0) {
//         CarOn = true;
//         Serial.println("Car is ON");

//         if (control_module_voltage > 13.0) {
//           EngineOn = true;
//           Serial.println("Engine is ON");
//         } else {
//           EngineOn = false;
//           Serial.println("Engine is OFF");
//         }
//       } else {
//         CarOn = false;
//         EngineOn = false;
//         Serial.println("Car & Engine is OFF");
//       }

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
