// #include "mcp2515.h"
// #include <SPI.h>

// MCP2515 MCP2515(5);
// #define MCP2515_INT 21

// uint32_t RX_ID;
// unsigned char dlc;
// unsigned char RX_Buff[8];

// // Define variables to store decoded values
// float engine_load = 0;
// int ect = 0;
// float stft = 0;
// int manifold_air_pressure = 0;
// int engine_rpm = 0;
// int throttle_position = 0;
// int vehicle_speed = 0;
// int iat = 0;
// float control_module_voltage = 0;

// enum PID_CODES : uint8_t {
//   PID_ENGINE_LOAD = 0x04,
//   PID_ENGINE_COOLANT_TEMPERATURE = 0x05,
//   PID_SHORT_TERM_FUEL_TRIM = 0x06,
//   PID_MANIFOLD_ABSOLUTE_PRESSURE = 0x0B,
//   PID_ENGINE_RPM = 0x0C,
//   PID_VEHICLE_SPEED = 0x0D,
//   PID_INTAKE_AIR_TEMPERATURE = 0x0F,
//   PID_THROTTLE_POSITION = 0x11,
//   PID_CONTROL_MODULE_VOLTAGE = 0x42
// };

// // Function to decode the PID data
// void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e);

// void setup() {
//   Serial.begin(115200);

//   // Configure MCP2515 CAN controller
//   if (MCP2515.config(MCP2515_STDEXT, CAN_500kbps, MCP2515_8MHZ) == CAN_OK) {
//     Serial.println("CAN setup successful!");
//   } else {
//     Serial.println("CAN setup failed!");
//   }

//   // Set the MCP2515 to Normal Mode
//   if (MCP2515.setNormalMode() != MODE_NORMAL) {
//     Serial.println("Failed to set Normal Mode");
//   }

//   // Uncomment the following lines to enable Loopback Mode
//   /*
//   if (MCP2515.setLoopbackMode() == MODE_LOOPBACK) {
//     Serial.println("Loopback Mode set successfully!");
//   } else {
//     Serial.println("Failed to set Loopback Mode");
//   }
//   */

//   pinMode(MCP2515_INT, INPUT);
// }

// void loop() {
//   // Check if a CAN message has been received
//   if (!digitalRead(MCP2515_INT)) {
//     // Receive the message
//     MCP2515.receiveMessage(&RX_ID, &dlc, RX_Buff);

//     Serial.print("Received ID: 0x");
//     Serial.print(RX_ID, HEX);
//     Serial.print(" DLC: ");
//     Serial.print(dlc);
//     Serial.print(" Data: ");
    
//     for (int i = 0; i < 8; i++) {
//       Serial.print("0x");
//       Serial.print(RX_Buff[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();

//     // Extract and decode the message payload
//     uint8_t PID = RX_Buff[2];
//     uint8_t A = RX_Buff[3];
//     uint8_t B = RX_Buff[4];
//     uint8_t C = RX_Buff[5];
//     uint8_t D = RX_Buff[6];
//     uint8_t E = RX_Buff[7];
//     decodePID(PID, A, B, C, D, E);
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
//       Serial.print("Vehicle Speed (km/h) = ");
//       Serial.println(vehicle_speed);
//       break;

//     case PID_INTAKE_AIR_TEMPERATURE:
//       iat = (a - 40);
//       Serial.print("Intake Air Temperature (°C) = ");
//       Serial.println(iat);
//       break;

//     case PID_THROTTLE_POSITION:
//       throttle_position = (a / 2.55);
//       Serial.print("Throttle Position (%) = ");
//       Serial.println(throttle_position);
//       break;

//     case PID_CONTROL_MODULE_VOLTAGE:
//       control_module_voltage = (((256 * a) + b) / 1000.0);
//       Serial.print("Control Module Voltage (V) = ");
//       Serial.println(control_module_voltage);
//       break;

//     default:
//       Serial.println("Unknown PID");
//       break;
//   }
// }
