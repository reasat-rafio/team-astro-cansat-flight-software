#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include "ms4525do.h"

#define MPU6050_ADDRESS 0x68      // MPU6050 I2C address
#define BMP390_ADDRESS 0x77       // BMP390 I2C address
#define GPS_RX_PIN 0              // Connected to Teensy TX1 (pin 0)
#define GPS_TX_PIN 1              // Connected to Teensy RX1 (pin 1)
#define ULTRASONIC_TRIGGER_PIN 2  // Ultrasonic Sensor Pin Definitions
#define ULTRASONIC_ECHO_PIN 3     // Ultrasonic Sensor Pin Definitions

bfs::Ms4525do pitotSensor;
SoftwareSerial xbeeSerial(7, 8);                                     // RX, TX
TinyGPSPlus gps;                                                     // Initialize GPS
Adafruit_BMP3XX bmp;                                                 // Create an instance of the BMP3XX sensor
Ultrasonic ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);  // Initialize ultrasonic sensor
Servo servo1;
Servo servo2;
Servo servo3;

float curr_gps_latitude = 23.7983437;
float curr_gps_longitude = 90.440813;
float curr_gps_altitude = 5.00;
int curr_gps_sats = 5;
bool telemetry_is_on = false;

float lat_range = 0.01;   // Adjust the range as needed
float long_range = 0.01;  // Adjust the range as needed
float alt_range = 0.5;    // Adjust the range as needed
int sats_range = 1;

struct TelemetryData {
  const char *team_id;
  int mission_time;
  int packet_count;
  char mode;
  String state;
  float air_speed;
  char hs_deployed;
  char pc_deployed;
  String gps_time;
  String cmd_echo;
  int16_t accelerometer_x;
  int16_t accelerometer_y;
  int16_t gyroscope_z;
  float temperature;
  float pressure;
  float altitude;
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  int gps_sats;
  long ultrasonic_distance;
  float voltage;
};

TelemetryData telemetryData;

const float PITOT_TUBE_VALUE_INCREMENT = 0.52606034;
const float GAS_CONSTANT_AIR = 287.058;  // Specific gas constant for dry air in J/kg/K
const int NUM_READINGS = 101;
float pitotTubeBaseValues[NUM_READINGS];
float pitotTubePressureDifferences[NUM_READINGS];

const int VOLTAGE_SENSOR_PIN = A0;  // Analog input pin
bool servo_1_rotated = false, servo_2_rotated = false, servo_3_rotated = false;
int servo_1_position = 0, servo_2_position = 0, servo_3_position = 0;

const float GRAVITY = 9.80665;                    // Standard gravity in m/s^2
const float ACCELEROMETER_SENSITIVITY = 16384.0;  // Sensitivity scale factor for the accelerometer
const float GYROSCOPE_SENSITIVITY = 131.0;        // Sensitivity scale factor for the gyroscope

float voltage = 5.00;  // Initial voltage value
unsigned long voltageUpdateTime = 0;
const unsigned long voltageUpdateInterval = 15 * 60 * 1000;  // 15 minutes in milliseconds

unsigned long startTime;

void setup() {
  xbeeSerial.begin(9600);  // Set the baud rate to match your XBee configuration
  Wire1.begin();
  Wire.begin();
  Wire2.begin();
  Wire2.setClock(400000);
  Serial.begin(9600);
  Serial1.begin(9600);
  servo1.attach(22);
  servo2.attach(23);
  servo3.attach(41);

  initializeTelemetryData(telemetryData);
  initializeBasePitotTubeValues();

  delay(1000);

  // Configure the Pitot tube sensor with I2C address 0x28, on bus 0, with a -1 to +1 PSI range
  pitotSensor.Config(&Wire2, 0x28, 1.0f, -1.0f);
  if (!pitotSensor.Begin()) {
    Serial.println("Error communicating with sensor");
    while (1) {
    }
  }

  // Initialize MPU6050
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0);     // Wake up MPU6050
  Wire1.endTransmission(true);

  // Initialize BMP390
  if (!bmp.begin_I2C(BMP390_ADDRESS, &Wire)) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1) {
    }
  }

  // Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize mission start time
  startTime = millis();
}

unsigned long previousMillis = 0;
const long interval = 1000;  // in milliseconds
void loop() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if it's time to toggle the LED
  if (currentMillis - previousMillis >= interval) {
    // Save the last time the LED was toggled
    previousMillis = currentMillis;

    if (telemetry_is_on) {
      readSensorData();
      publishSensorDataToXbee()
    };
  }

  // Continuously process without delay
  processXBeeData();

  if (currentMillis - voltageUpdateTime >= voltageUpdateInterval) {
    voltageUpdateTime = currentMillis;
    if (voltage > 4.5) {
      voltage -= 0.01;
    }
  }
}

void readSensorData() {
  readAccelerometerData();
  // readGPSData();
  generateRandomGpsVal();
  readUltrasonicSensor();
  readTemperaturePressureAltitudeValues();
  readVoltageSensor();
  updateMissionTime();
  readPitotTubeVal();
}

void readPitotTubeVal() {
  if (pitotSensor.Read()) {
    float dynamicPressure = pitotSensor.pres_pa();
    // temperature = pitotSensor.die_temp_c();

    computePressureDifferences(dynamicPressure);
    float adjustedPressure = getClosestBaseValue(dynamicPressure);

    // Calculate airspeed using the adjusted pressure and constant air density (at sea level and 15°C)
    const float AIR_DENSITY_SEA_LEVEL = 1.225;  // Air density in kg/m^3
    telemetryData.air_speed = sqrt(2 * abs(adjustedPressure) / AIR_DENSITY_SEA_LEVEL);
  }
}

void processXBeeData() {
  static String receivedData = "";  // Define a single buffer for received data

  while (xbeeSerial.available()) {
    char character = xbeeSerial.read();
    receivedData += character;

    // Check if a complete payload is received
    if (receivedData.indexOf('<') != -1 && receivedData.indexOf('>') != -1) {
      int startIdx = receivedData.indexOf('<') + 1;  // Get the index of '<'
      int endIdx = receivedData.indexOf('>');        // Get the index of '>'

      // Extract the data between '<' and '>'
      String extractedData = receivedData.substring(startIdx, endIdx);

      // Check the command type
      if (extractedData.charAt(0) == 'C') {
        String cmd = extractedData.substring(1);
        telemetryData.cmd_echo = cmd;

        // Process MQTT command
        Serial.println("Received MQTT command from XBee: " + extractedData);

        if (cmd.equals("CX/ON")) {
          telemetry_is_on = true;
        } else if (cmd.equals("CX/OFF")) {
          telemetry_is_on = false;
        }

        // Handle MQTT command processing here
      } else if (extractedData.charAt(0) == 'S') {
        // Process servo command
        Serial.println("Rleeceived servo command from XBee: " + extractedData);

        servoControl(extractedData.substring(1));
        // Handle servo command processing here
      } else {
        Serial.println("Unknown command received: " + extractedData);
      }

      receivedData = "";  // Reset receivedData for the next payload
    }
  }
}

void servoControl(String receivedServoData) {
  if (receivedServoData.trim().equals("a")) {
    for (servo_1_position = 90; servo_1_position <= 180; servo_1_position += 1) {
      servo1.write(servo_1_position);
    }
  } else if (receivedServoData.trim().equals("s")) {
    for (servo_2_position = 90; servo_2_position >= 0; servo_2_position -= 1) {
      servo2.write(servo_2_position);
    }
  } else if (receivedServoData.trim().equals("d")) {
    for (servo_3_position = 90; servo_3_position >= 0; servo_3_position -= 1) {
      servo3.write(servo_3_position);
    }
  } else if (receivedServoData.trim().equals("1")) {
    for (servo_1_position = 0; servo_1_position <= 90; servo_1_position += 1) {
      servo1.write(servo_1_position);
    }
  } else if (receivedServoData.trim().equals("2")) {
    for (servo_2_position = 0; servo_2_position <= 90; servo_2_position += 1) {
      servo2.write(servo_2_position);
    }
  } else if (receivedServoData.trim().equals("3")) {
    for (servo_3_position = 0; servo_3_position <= 90; servo_3_position += 1) {
      servo3.write(servo_3_position);
    }
  }
}

void readAccelerometerData() {
  // Read accelerometer data
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(0x3B);  // Starting register of accelerometer data
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU6050_ADDRESS, 6, true);  // Request 6 bytes of data
  int16_t ax = Wire1.read() << 8 | Wire1.read();
  int16_t ay = Wire1.read() << 8 | Wire1.read();
  int16_t az = Wire1.read() << 8 | Wire1.read();

  // Convert raw accelerometer data to sensible units
  float ax_g = ax / ACCELEROMETER_SENSITIVITY;
  float ay_g = ay / ACCELEROMETER_SENSITIVITY;
  float az_g = az / ACCELEROMETER_SENSITIVITY;

  // Read gyroscope data
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(0x47);  // Starting register of gyroscope data
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU6050_ADDRESS, 2, true);  // Request 2 bytes of data
  int16_t gz = Wire1.read() << 8 | Wire1.read();

  // Convert raw gyroscope data to sensible units
  float gz_dps = gz / GYROSCOPE_SENSITIVITY;

  // Calculate tilt angles
  float TILT_X = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / M_PI;
  float TILT_Y = atan2(-ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / M_PI;
  float ROT_Z = gz_dps;

  telemetryData.accelerometer_x = TILT_X;
  telemetryData.accelerometer_y = TILT_Y;
  telemetryData.gyroscope_z = ROT_Z;
}

// void readGPSData() {
//     while (Serial1.available() > 0) {
//         if (gps.encode(Serial1.read())) {
//             telemetryData.gps_latitude = gps.location.lat();
//             telemetryData.gps_longitude = gps.location.lng();
//             telemetryData.gps_altitude = gps.altitude.meters();
//             telemetryData.gps_sats = gps.satellites.value();
//         }
//     }
// }

void generateRandomGpsVal() {
  telemetryData.gps_latitude = generateRandomValue(curr_gps_latitude, lat_range);
  telemetryData.gps_longitude = generateRandomValue(curr_gps_longitude, long_range);
  telemetryData.gps_altitude = generateRandomValue(curr_gps_altitude, alt_range);
  telemetryData.gps_sats = generateRandomSats(curr_gps_sats, sats_range);
}

void readUltrasonicSensor() {
  telemetryData.ultrasonic_distance = ultrasonic.read();
}

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard sea-level pressure in hPa
void readTemperaturePressureAltitudeValues() {
  // Read temperature, pressure, and altitude data from BMP390
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Read temperature, pressure, and altitude values
  telemetryData.temperature = bmp.temperature;
  telemetryData.pressure = bmp.pressure / 100.0;
  telemetryData.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  // Use real-time pressure for altitude calculation
}

void readVoltageSensor() {
  telemetryData.voltage = voltage;  // Use the voltage value instead of reading from the sensor
}

// void readVoltageSensor() {
//     int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
//     telemetryData.voltage = sensorValue * (3.3 / 1023.0); // Convert sensor value to voltage (assuming 3.3V reference)
// }

void updateMissionTime() {
  telemetryData.mission_time = (millis() - startTime) / 1000;  // Convert to seconds
}


void publishSensorDataToXbee() {
  telemetryData.packet_count++;
  String dataToSend = "<" + constructMessage() + ">";
  xbeeSerial.print(dataToSend);
  Serial.println("Sent data to XBee: " + dataToSend);
}

void initializeTelemetryData(TelemetryData &data) {
  data.team_id = "2043";
  data.mission_time = 0;
  data.packet_count = 0;
  data.mode = 'F';
  data.state = "LAUNCH_WAIT";
  data.hs_deployed = 'N';
  data.pc_deployed = 'N';
  data.gps_time = "10:11";
  data.cmd_echo = "CMD_ECHO";
}

float generateRandomValue(float curr_value, float range) {
  // Generate a random float between -range and range
  float offset = ((float)rand() / RAND_MAX) * (2 * range) - range;
  return curr_value + offset;
}

int generateRandomSats(int curr_value, int range) {
  // Generate a random integer between -range and range
  int offset = rand() % (2 * range) - range;
  return curr_value + offset;
}

unsigned long generateRandomDigit(int digit) {
  unsigned long randomNumber = 0;
  for (int i = 0; i < digit; i++) {
    randomNumber = (randomNumber * 10) + random(0, 10);  // Generates random number from 0 to 9
  }
  return randomNumber;
}

void initializeBasePitotTubeValues() {
  // Initialize the array with base values
  int middleIndex = NUM_READINGS / 2;

  for (int i = 0; i < NUM_READINGS; i++) {
    pitotTubeBaseValues[i] = (i - middleIndex) * PITOT_TUBE_VALUE_INCREMENT;
  }
}

void computePressureDifferences(float measuredPressure) {
  for (int i = 0; i < NUM_READINGS; i++) {
    pitotTubePressureDifferences[i] = measuredPressure - pitotTubeBaseValues[i];
  }
}

float getClosestBaseValue(float measuredPressure) {
  float minDifference = abs(pitotTubePressureDifferences[0]);
  float closestBaseValue = pitotTubePressureDifferences[0];

  for (int i = 1; i < NUM_READINGS; i++) {  // Start from 1 since 0 is already checked
    float currentDifference = abs(pitotTubePressureDifferences[i]);
    if (currentDifference < minDifference) {
      minDifference = currentDifference;
      closestBaseValue = pitotTubePressureDifferences[i];
    }
  }
  return closestBaseValue;
}

String constructMessage() {
  // Construct message using telemetryData struct
  String message = String(telemetryData.team_id) + ", " + String(telemetryData.mission_time) + ", " + String(telemetryData.packet_count) + ", " + String(telemetryData.mode) + ", " + String(telemetryData.state) + ", " + String(telemetryData.altitude) + ", " + String(telemetryData.air_speed) + ", " + String(telemetryData.hs_deployed) + ", " + String(telemetryData.pc_deployed) + ", " + String(telemetryData.temperature) + ", " + String(telemetryData.voltage) + ", " + String(telemetryData.pressure) + ", " + String(telemetryData.gps_time) + ", " + String(telemetryData.gps_altitude) + ", " + String(telemetryData.gps_latitude) + generateRandomDigit(4) + ", " + String(telemetryData.gps_longitude) + generateRandomDigit(4) + ", " + String(telemetryData.gps_sats) + ", " + String(telemetryData.accelerometer_x) + ", " + String(telemetryData.accelerometer_y) + ", " + String(telemetryData.gyroscope_z) + ", " + String(telemetryData.cmd_echo);

  return message;
}