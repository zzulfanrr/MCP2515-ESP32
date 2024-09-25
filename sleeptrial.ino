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

// // Enum untuk state machine
// enum CarState {
//   CAR_OFF,
//   CAR_ACC_ON,
//   CAR_ENGINE_ON
// };

// CarState currentState = CAR_OFF;

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// unsigned long prevTX = 0;
// const unsigned int invlTX = 1000;

// // Variabel untuk debouncing
// unsigned long lastStateChangeTime = 0;
// const unsigned long debounceDelay = 1000;  // 1 detik debounce

// // Variabel untuk logging
// unsigned long lastLogTime = 0;
// const unsigned long logInterval = 2000;  // Log setiap 5 detik

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e);
// void logPID();
// void updateCarState();
// void handleCarState();
// void enterSleepMode();

// void setup() {
//   Serial.begin(115200);

//   esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

//   if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
//     Serial.println("Wakeup caused by external signal using RTC_IO");
//     // Lakukan inisialisasi khusus jika bangun karena sinyal eksternal
//   } else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
//     Serial.println("Wakeup caused by timer");
//     // Lakukan inisialisasi khusus jika bangun karena timer
//   } else {
//     Serial.println("Wakeup was not caused by deep sleep");
//     // Ini adalah boot normal, lakukan inisialisasi penuh
//   }

//   if (MCP2515.config(MCP2515_STDEXT, CAN_500kbps, MCP2515_8MHZ) == CAN_OK) {
//     digitalWrite(INTERNAL_LED, HIGH);
//     delay(500);
//     digitalWrite(INTERNAL_LED, LOW);
//   } else {
//     Serial.println("Setup Failed");
//   }

//   if (MCP2515.setNormalMode() != MODE_NORMAL) {
//     Serial.println("Failed to set Normal Mode");
//   }

//   MCP2515.mcp2515_setSleepWakeup(1);

//   pinMode(INTERNAL_LED, OUTPUT);
//   pinMode(MCP2515_INT, INPUT);

//   Serial.println("#,timestamp,carstate,MAP,RPM,VSS,IAT,CMV");
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

//   // Update car state
//   updateCarState();

//   // Handle current state
//   handleCarState();

//   // Periodic logging
//   if (currentMillis - lastLogTime >= logInterval) {
//     logPID();
//     lastLogTime = currentMillis;
//   }

//   // Check for CAN messages
//   if (!digitalRead(MCP2515_INT)) {
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);
//     // if (RX_ID == 0x98DAF10E) {
//     // ID& DLC
//     // Serial.print("ID: 0x");
//     // Serial.print(RX_ID, HEX);
//     // Serial.print("       DLC: ");
//     // Serial.print(dlc);
//     // Serial.println(" Data:");
//     // // DLC
//     // Serial.print("Received Data: ");
//     // for (int i = 0; i < 8; i++) {
//     //   Serial.print("0x");
//     //   Serial.print(RX_Buff[i], HEX);
//     //   Serial.print(" ");
//     // }
//     // Serial.println();
//     decodePID(RX_Buff[2], RX_Buff[3], RX_Buff[4], RX_Buff[5], RX_Buff[6], RX_Buff[7]);
//     // }
//   }
// }

// void updateCarState() {
//   static bool firstRun = true;
//   unsigned long currentMillis = millis();
//   CarState newState = currentState;

//   if (firstRun) {
//     // Segera periksa status mobil setelah wake up
//     sendOBDRequests();
//     firstRun = false;
//   }

//   // Jika tidak ada respons OBD, tegangan tidak akan diperbarui
//   // Tetap asumsikan state berdasarkan tegangan yang terakhir diterima
//   if (control_module_voltage > 13.0) {
//     newState = CAR_ENGINE_ON;
//   } else if (control_module_voltage > 12.0) {
//     newState = CAR_ACC_ON;
//   } else {
//     newState = CAR_OFF;
//   }

//   // Debouncing untuk menghindari perubahan state yang cepat
//   if (newState != currentState && (currentMillis - lastStateChangeTime > debounceDelay)) {
//     currentState = newState;
//     lastStateChangeTime = currentMillis;
//     Serial.print("Car state changed to: ");
//     Serial.println(currentState);

//     if (currentState == CAR_OFF) {
//       enterSleepMode();
//     }
//   }
// }

// void handleCarState() {
//   static unsigned long lastWakeTime = 0;
//   unsigned long currentTime = millis();

//   switch (currentState) {
//     case CAR_OFF:
//       if (currentTime - lastWakeTime > 10000) {  // Tunggu 10 detik sebelum sleep lagi
//         enterSleepMode();
//       }
//       break;
//     case CAR_ACC_ON:
//     case CAR_ENGINE_ON:
//       if (currentTime - prevTX >= invlTX) {
//         sendOBDRequests();
//       }
//       break;
//   }

//   if (currentTime < lastWakeTime) {
//     // millis() telah reset, update lastWakeTime
//     lastWakeTime = currentTime;
//   }
// }

// void sendOBDRequests() {
//   bool receivedResponse = false;              // Variabel untuk mengecek apakah ada respons
//   unsigned long requestStartTime = millis();  // Mulai menghitung waktu

//   for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//     byte sendStat = MCP2515.sendMessage(REQ_ID, 8, dataSets[i]);

//     // if (sendStat != CAN_OK) {
//     //   Serial.println("Error sending OBD request");
//     //   return;
//     // }

//     unsigned long waitTime = millis();
//     while (millis() - waitTime < 500) {  // Tunggu sampai 500ms untuk menerima respons
//       if (!digitalRead(MCP2515_INT)) {
//         MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);
//         // if (RX_ID == 0x98DAF10E) {
//         decodePID(RX_Buff[2], RX_Buff[3], RX_Buff[4], RX_Buff[5], RX_Buff[6], RX_Buff[7]);
//         receivedResponse = true;  // Menandai bahwa ada respons yang diterima
//         // }
//       }
//     }
//     delay(100);  // Delay kecil antara request
//   }

//   prevTX = millis();  // Update waktu terakhir transmisi

//   // Jika tidak ada respons, asumsikan mobil mati
//   if (!receivedResponse) {
//     Serial.println("No OBD response received, assuming car is off.");
//     currentState = CAR_OFF;
//     enterSleepMode();  // Masuk sleep mode jika tidak ada respons
//   }
// }


// void enterSleepMode() {
//   if (MCP2515.setSleepMode() == MODE_SLEEP) {
//     Serial.println("MCP2515 entering sleep mode");
//   } else {
//     Serial.println("Failed to set MCP2515 to sleep mode");
//   }

//   // Configure ESP32 to wake up after 5 seconds
//   esp_sleep_enable_timer_wakeup(5000000);        // 5 seconds in microseconds
//   esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, 0);  // Wake up if MCP2515 INT goes low

//   Serial.println("Entering deep sleep for 5 seconds");
//   esp_deep_sleep_start();
// }

// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
//   switch (static_cast<PID_CODES>(pid)) {
//     case PID_MANIFOLD_ABSOLUTE_PRESSURE:
//       manifold_absolute_pressure = a;
//       break;
//     case PID_enginee_speed:
//       enginee_speed = ((a * 256 + b) / 4); 
//       break;
//     case PID_VEHICLE_SPEED:
//       vehicle_speed = a;
//       break;
//     case PID_INTAKE_AIR_TEMPERATURE:
//       intake_air_temperature = (a - 40);
//       break;
//     case PID_CONTROL_MODULE_VOLTAGE:
//       control_module_voltage = (((256 * a) + b) / 1000.0);
//       break;
//     default:
//       break;
//   }
// }

// void logPID() {
//   static int count = 1;
//   String logMessage = String(count++) + "," + String(millis()) + "," + String(currentState) + "," + String(manifold_absolute_pressure) + "," + String(enginee_speed) + "," + String(vehicle_speed) + "," + String(intake_air_temperature) + "," + String(control_module_voltage);
//   Serial.println(logMessage);
// }