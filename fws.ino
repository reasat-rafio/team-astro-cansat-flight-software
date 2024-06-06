#include "ms4525do.h"
#include <Adafruit_BMP3XX.h>
#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <TimeLib.h>

#define MPU6050_ADDRESS 0x68              // MPU6050 I2C address
#define BMP390_ADDRESS 0x77               // BMP390 I2C address
#define GPS_RX_PIN 0                      // Connected to Teensy TX1 (pin 0)
#define GPS_TX_PIN 1                      // Connected to Teensy RX1 (pin 1)
#define SEA_LEVEL_PRESSURE_HPA (1013.25)  // Standard sea-level pressure in hPa
// Define phase constants
#define PRE_LAUNCH 0
#define ASCENT 1
#define DESCENT 2
#define PARACHUTE_DEPLOY 3
#define HEAT_SHIELD_RELEASE 4
#define LANDED 5
// Thresholds
#define ASCENT_ALTITUDE_THRESHOLD 10  // Altitude increase threshold for ascent
#define DESCENT_ALTITUDE_CHANGE -1    // Altitude decrease threshold for descent
#define DESCENT_ALTITUDE_LIMIT 110    // Altitude limit for HEAT_SHIELD_DEPLOY
#define LANDING_VELOCITY_THRESHOLD 1  // Velocity threshold to confirm landing
#define LANDING_ALTITUDE_CHANGE 1     // Altitude threshold to confirm landing

class Timer {
private:
  unsigned long previousMillis;
  const long interval;

public:
  Timer(long interval)
    : previousMillis(0), interval(interval) {}

  bool isElapsed() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      return true;
    }
    return false;
  }
};

Timer telemetryPublishTimer(1000);
Timer buzzerTimer(1000);
Timer telemetryReadTimer(50);

bfs::Ms4525do pitotSensor;
SoftwareSerial xbeeSerial(7, 8);  // RX, TX
TinyGPSPlus gps;                  // Initialize GPS
Adafruit_BMP3XX bmp;              // Create an instance of the BMP3XX sensor
Servo servo1;
Servo servo2;

const int PACKET_COUNT_ADDRESS = 0;
const int EEPROM_HOURS_ADDR = 19;
const int EEPROM_MINUTES_ADDR = 23;
const int EEPROM_SECONDS_ADDR = 27;
const int MISSION_TIME_ADDRESS = sizeof(unsigned long);
const int INITIAL_ALTITUDE_ADDRESS = EEPROM_SECONDS_ADDR + sizeof(unsigned long);

bool telemetry_is_on = false;
bool is_landed = false;
bool clock_running = false;
unsigned long startClockTime = 0;
unsigned long elapsedClockTime = 0;

bool simulation_enable = false;
bool simulation_activate = false;

const float PITOT_TUBE_VALUE_INCREMENT = 0.52606034;
const float GAS_CONSTANT_AIR = 287.058;  // Specific gas constant for dry air in J/kg/K
const int NUM_READINGS = 101;
float pitotTubeBaseValues[NUM_READINGS];
float pitotTubePressureDifferences[NUM_READINGS];

// const int BUZZER_PIN = 4;

const int VOLTAGE_SENSOR_PIN = A0;  // Analog input pin
bool servo_1_rotated = false, servo_2_rotated = false;
int servo_1_position = 0, servo_2_position = 0;
// servo_1_position = 0,

const float GRAVITY = 9.80665;                    // Standard gravity in m/s^2
const float ACCELEROMETER_SENSITIVITY = 16384.0;  // Sensitivity scale factor for the accelerometer
const float GYROSCOPE_SENSITIVITY = 131.0;        // Sensitivity scale factor for the gyroscope

float initial_altitude = 0.0;
float acceleration = 0.0;
float previousAltitude = 0.0;

const int buzzerPin = 4;
unsigned long turnOnBuzzer_previousMillis = 0;
bool turnOnBuzzer_buzzerState = LOW;

unsigned long startTimeServo2 = 0;
unsigned long startTimeServo1 = 0;

const int sensorPin = A0;                // Define the pin for the voltage sensor
const float inputVoltage = 8.0;          // Input voltage in volts
const float desiredOutputVoltage = 5.0;  // Desired output voltage in volts
const float R1 = 10000.0;                // Resistance of R1 in ohms

struct TelemetryData {
  const char *team_id;
  String mission_time;
  unsigned long packet_count;
  char mode;
  int state;
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
  float voltage;
};

TelemetryData telemetryData;

unsigned long startTime;
void setup() {
  xbeeSerial.begin(9600);  // Set the baud rate to match your XBee configuration
  Wire1.begin();
  Wire.begin();
  Wire2.begin();
  Wire2.setClock(400000);
  Serial.begin(9600);
  Serial1.begin(9600);

  servo1.attach(23);
  servo2.attach(41);

  initializeTelemetryData(telemetryData);
  initializeBasePitotTubeValues();

  delay(1000);

  // Initialize buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Initialize buzzer to be off

  // Configure the Pitot tube sensor with I2C address 0x28, on bus 0, with a -1 to +1 PSI range
  pitotSensor.Config(&Wire2, 0x28, 1.0f, -1.0f);
  if (!pitotSensor.Begin()) {
    Serial.println("Error communicating  with sensor");
  }

  // Initialize MPU6050
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0);     // Wake up MPU6050
  Wire1.endTransmission(true);

  // Initialize BMP390
  if (!bmp.begin_I2C(BMP390_ADDRESS, &Wire)) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
  }

  // Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Initialize mission start timehttps://www.youtube.com/
  startTime = millis();
}

unsigned long previousMillis = 0;
unsigned long previousFlightStatesMillis = 0;
const long interval = 1000;
const long flightStatesInterval = 500;
void loop() {
  unsigned long currentMillis = millis();


  if (telemetry_is_on && !simulation_activate) {
    if (telemetryReadTimer.isElapsed()) {
      runClockAndSetMissionTime();
      readSensorData();
      flightStatesLogic();
    }

    if (telemetryPublishTimer.isElapsed()) {
      publishSensorDataToXbee();
    }
  }

  // Continuously process without delay
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (currentMillis - previousFlightStatesMillis >= flightStatesInterval) {
        previousFlightStatesMillis = currentMillis;
        readGPSData();
      }
    }
  }
  processReceivedXBeeData();
}

void loadTimeFromEEPROM() {
  int hours;
  int minutes;
  int seconds;

  EEPROM.get(EEPROM_HOURS_ADDR, hours);
  EEPROM.get(EEPROM_MINUTES_ADDR, minutes);
  EEPROM.get(EEPROM_SECONDS_ADDR, seconds);

  setTime(hours, minutes, seconds, day(), month(), year());
  Teensy3Clock.set(now());  // set the RTC with the retrieved time
}

void flightStatesLogic() {
  static int ascentConditionCounter = 0;
  static int descentConditionCounter = 0;
  static int landedConditionCounter = 0;

  float currentAltitude = telemetryData.altitude;
  float altitudeChange = currentAltitude - previousAltitude;

  switch (telemetryData.state) {
    case PRE_LAUNCH:
      if (currentAltitude > ASCENT_ALTITUDE_THRESHOLD) {
        ascentConditionCounter++;
      } else {
        ascentConditionCounter = 0;
      }

      if (ascentConditionCounter >= 20) {
        telemetryData.state = ASCENT;
        Serial.println("Phase 1: Ascent");
        ascentConditionCounter = 0;
      }
      break;

    case ASCENT:
      if (altitudeChange <= DESCENT_ALTITUDE_CHANGE & currentAltitude < 725) {
        descentConditionCounter++;
      } else {
        descentConditionCounter = 0;
      }

      if (descentConditionCounter > 20) {
        telemetryData.state = DESCENT;
        Serial.println("Phase 2: Descent");
        descentConditionCounter = 0;
      }
      break;

    case DESCENT:
      if (currentAltitude <= DESCENT_ALTITUDE_LIMIT) {
        descentConditionCounter++;
      } else {
        descentConditionCounter = 0;
      }

      if (descentConditionCounter >= 20) {
        telemetryData.state = HEAT_SHIELD_RELEASE;
        Serial.println("Phase 3: HEAT_SHIELD_RELEASE");
        descentConditionCounter = 0;
      }
      break;

    case HEAT_SHIELD_RELEASE:
      if (!servo_1_rotated) {
        startTimeServo1 = millis();
        servo_1_rotated = true;
        for (int servo_1_position = 90; servo_1_position >= 0; servo_1_position -= 1) {
          servo1.write(servo_1_position);
        }
        telemetryData.hs_deployed = 'P';
        Serial.println("Heat Shield released");
      }

      if (servo_1_rotated && (millis() - startTimeServo1 >= 350)) {
        telemetryData.state = PARACHUTE_DEPLOY;
        Serial.println("Phase 4: PARACHUTE_DEPLOY");
      }
      break;

    case PARACHUTE_DEPLOY:
      if (!servo_2_rotated) {
        startTimeServo2 = millis();
        servo_2_rotated = true;
        for (int servo_2_position = 90; servo_2_position >= 0; servo_2_position -= 1) {
          servo2.write(servo_2_position);
        }
        telemetryData.pc_deployed = 'C';
        Serial.println("Parachute deployed");
      }

      if (servo_2_rotated) {
        telemetryData.state = LANDED;
        Serial.println("Phase 5: Landed");
      }

      if (currentAltitude <= 10) {
        landedConditionCounter++;
      } else {
        landedConditionCounter = 0;
      }

      if (landedConditionCounter >= 20) {
        is_landed = true;
        telemetryData.state = LANDED;
        Serial.println("Phase 5: Landed");
      }
      break;

    case LANDED:
      Serial.println("Rocket has landed safely.");
      turnOnBuzzer();
      endTelemetry();
      break;
  }

  previousAltitude = currentAltitude;
}

void semulationFlightStatesLogic() {
  static int ascentConditionCounter = 0;
  static int descentConditionCounter = 0;
  static int landedConditionCounter = 0;

  float currentAltitude = telemetryData.altitude;
  float altitudeChange = currentAltitude - previousAltitude;

  switch (telemetryData.state) {
    case PRE_LAUNCH:
      if (currentAltitude > ASCENT_ALTITUDE_THRESHOLD) {
        telemetryData.state = ASCENT;
        Serial.println("Phase 1: Ascent");
      }
      break;

    case ASCENT:
      if (altitudeChange <= DESCENT_ALTITUDE_CHANGE & currentAltitude < 650) {
        descentConditionCounter++;
      } else {
        descentConditionCounter = 0;
      }

      if (descentConditionCounter > 2) {
        telemetryData.state = DESCENT;
        Serial.println("Phase 2: Descent");
        descentConditionCounter = 0;
      }
      break;

    case DESCENT:
      if (currentAltitude <= DESCENT_ALTITUDE_LIMIT) {
        telemetryData.state = HEAT_SHIELD_RELEASE;
        Serial.println("Phase 3: HEAT_SHIELD_RELEASE");
      }
      break;

    case HEAT_SHIELD_RELEASE:
      if (!servo_1_rotated) {
        startTimeServo1 = millis();
        servo_1_rotated = true;
        for (int servo_1_position = 90; servo_1_position >= 0; servo_1_position -= 1) {
          servo1.write(servo_1_position);
        }
        telemetryData.hs_deployed = 'P';
        Serial.println("Heat Shield released");
      }

      if (servo_1_rotated && (millis() - startTimeServo1 >= 350)) {
        telemetryData.state = PARACHUTE_DEPLOY;
        Serial.println("Phase 4: PARACHUTE_DEPLOY");
      }
      break;

    case PARACHUTE_DEPLOY:
      if (!servo_2_rotated) {
        startTimeServo2 = millis();
        servo_2_rotated = true;
        for (int servo_2_position = 90; servo_2_position >= 0; servo_2_position -= 1) {
          servo2.write(servo_2_position);
        }
        telemetryData.pc_deployed = 'C';
        Serial.println("Parachute deployed");
      }

      if (servo_2_rotated) {
        telemetryData.state = LANDED;
        Serial.println("Phase 5: Landed");
      }

      if (currentAltitude <= 10) {
        landedConditionCounter++;
      } else {
        landedConditionCounter = 0;
      }

      if (landedConditionCounter >= 20) {
        is_landed = true;
        telemetryData.state = LANDED;
        Serial.println("Phase 5: Landed");
      }
      break;

    case LANDED:
      Serial.println("Rocket has landed safely.");
      turnOnBuzzer();
      endTelemetry();
      break;
  }

  previousAltitude = currentAltitude;
}

void saveData() {}

void turnOnBuzzer() {
  if (buzzerTimer.isElapsed()) {
    if (turnOnBuzzer_buzzerState == LOW) {
      turnOnBuzzer_buzzerState = HIGH;
    } else {
      turnOnBuzzer_buzzerState = LOW;
    }
    digitalWrite(buzzerPin, turnOnBuzzer_buzzerState);
  }
}

void turnOffBuzzer() {
  // Turn off the buzzer and reset the state
  digitalWrite(buzzerPin, LOW);

  turnOnBuzzer_buzzerState = LOW;
}

void endTelemetry() {
  telemetry_is_on = false;
  stopClock();
}

void readSensorData() {
  readAccelerometerData();
  readTemperaturePressureAltitudeValues();
  readVoltageSensor();
  readPitotTubeVal();
}

// Simulation mode
void readSensorData(float sim_pressure) {
  readAccelerometerData();
  readTemperaturePressureAltitudeValues(sim_pressure);
  readVoltageSensor();
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

void processReceivedXBeeData() {
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
        handleMQTTCommand(extractedData.substring(1));
      } else if (extractedData.charAt(0) == 'S') {
        handleServoCommand(extractedData.substring(1));
      } else {
        Serial.println("Unknown command received: " + extractedData);
      }

      receivedData = "";  // Reset receivedData for the next payload
    }
  }
}

void handleMQTTCommand(String cmd) {
  int slashIndex = cmd.indexOf('/');
  String beforeSlash = cmd.substring(0, slashIndex);
  String afterSlash = cmd.substring(slashIndex + 1);
  telemetryData.cmd_echo = cmd;

  Serial.println("Received MQTT command from XBee: " + cmd);

  if (beforeSlash.equals("PRESSURE")) {
    float sim_pressure = afterSlash.toFloat();
    readSensorData(sim_pressure / 100);
    semulationFlightStatesLogic();
    publishSensorDataToXbee();
    Serial.println("Pressure value: " + String(afterSlash));
  } else if (beforeSlash.equals("UTC")) {
    Serial.print(afterSlash);
    if (afterSlash.equals("GPS")) {

    } else {
      int firstColon = afterSlash.indexOf(':');
      int secondColon = afterSlash.indexOf(':', firstColon + 1);

      int hours = afterSlash.substring(0, firstColon).toInt();
      int minutes = afterSlash.substring(firstColon + 1, secondColon).toInt();
      int seconds = afterSlash.substring(secondColon + 1).toInt();

      setTime(hours, minutes, seconds, day(), month(), year());
      Teensy3Clock.set(now());  // set the RTC

      Serial.println("RTC has been set!");
    }
  } else if (cmd.equals("CX/ON")) {
    telemetry_is_on = true;
    startClock();
  } else if (cmd.equals("CX/OFF")) {
    telemetry_is_on = false;
  } else if (cmd.equals("CAL")) {
    calibrateAltitude(3);
    String msg = "<RCAL, SUCCESS, Calibrated altitude = " + String(initial_altitude) + ">";
    xbeeSerial.print(msg);
  } else if (cmd.equals("BCN/ON")) {
    turnOnBuzzer();
    xbeeSerial.print("<RBCN/ON, SUCCESS, Buzzer turned on successfully>");
  } else if (cmd.equals("BCN/OFF")) {
    turnOffBuzzer();
    xbeeSerial.print("<RBCN/OFF, SUCCESS, Buzzer turned off successfully>");
  } else if (cmd.equals("SIM/ACTIVATE")) {
    if (simulation_enable) {
      telemetry_is_on = false;
      simulation_activate = true;
      telemetryData.mode = 'S';
      xbeeSerial.print("<RSIM/ACTIVATE, SUCCESS, Simulation Activated>");
      Serial.println("simulation_activate");
    } else {
      xbeeSerial.print("<RSIM/ACTIVATE, FAILED, Simulation is not enabled>");
      Serial.println("simulation_activate failed because not enabled");
    }
  } else if (cmd.equals("SIM/ENABLE")) {
    simulation_enable = true;
    xbeeSerial.print("<RSIM/ENABLE, SUCCESS, Simulation enabled>");
    Serial.println("simulation_enable");
  } else if (cmd.equals("SIM/DISABLE")) {
    simulation_enable = false;
    xbeeSerial.print("<RSIM/DISABLE, SUCCESS, Simulation disabled>");
    Serial.println("simulation_disabled");
  } else if (cmd.equals("RESET/ALL")) {
    resetMissionTime();
    resetPacketCount();
    initializeTelemetryData(telemetryData);
    xbeeSerial.print("<RRESET/ALL, SUCCESS, Reset succcessful>");
  } else if (cmd.equals("HS/ON")) {
    moveServo(servo2, 90, 0);
    xbeeSerial.print("<RHS/ON, SUCCESS, Heatshield activated>");
    Serial.println("Heatshield activated");
  } else if (cmd.equals("HS/OFF")) {
    moveServo(servo2, 0, 90);
    xbeeSerial.print("<RHS/OFF, SUCCESS, Heatshield deactivated>");
    Serial.println("Heatshield deactivated");
  } else if (cmd.equals("PC/ON")) {
    moveServo(servo1, 90, 0);
    xbeeSerial.print("<RPC/ON, SUCCESS, Parachute deployed>");
    Serial.println("Parachute deployed");
  } else if (cmd.equals("PC/OFF")) {
    moveServo(servo2, 90, 0);
    xbeeSerial.print("<RPC/OFF, SUCCESS, Parachute retracted>");
    Serial.println("Parachute retracted");
  }
}

void handleServoCommand(String receivedServoData) {
  Serial.println("Received servo command from XBee: " + receivedServoData);
  if (receivedServoData.trim().equals("a")) {
    moveServo(servo1, 90, 0);
  } else if (receivedServoData.trim().equals("s")) {
    moveServo(servo2, 90, 0);
  } else if (receivedServoData.trim().equals("1")) {
    moveServo(servo1, 0, 90);
  } else if (receivedServoData.trim().equals("2")) {
    moveServo(servo2, 0, 90);
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

  acceleration = sqrt(sq(ax) + sq(ay) + sq(az)) / 16384.0;  // Convert to 'g'
  telemetryData.accelerometer_x = TILT_X;
  telemetryData.accelerometer_y = TILT_Y;
  telemetryData.gyroscope_z = ROT_Z;
}

void readGPSData() {
  telemetryData.gps_latitude = gps.location.lat();
  telemetryData.gps_longitude = gps.location.lng();
  telemetryData.gps_altitude = gps.altitude.meters();
  telemetryData.gps_sats = gps.satellites.value();
  telemetryData.gps_time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
}

void saveTimeToEEPROM() {
  EEPROM.put(EEPROM_HOURS_ADDR, hour());
  EEPROM.put(EEPROM_MINUTES_ADDR, minute());
  EEPROM.put(EEPROM_SECONDS_ADDR, second());
}


void readTemperaturePressureAltitudeValues() {
  // Read temperature, pressure, and altitude data from BMP390
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Read temperature, pressure, and altitude values
  telemetryData.temperature = bmp.temperature;
  telemetryData.pressure = bmp.pressure / 100.0;
  telemetryData.altitude = calculateAltitude(telemetryData.pressure) - initial_altitude;
}

// Simulation mode
void readTemperaturePressureAltitudeValues(float sim_pressure) {
  // Read temperature, pressure, and altitude data from BMP390
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Read temperature, pressure, and altitude values
  telemetryData.temperature = bmp.temperature;
  telemetryData.pressure = sim_pressure;
  telemetryData.altitude = calculateAltitude(telemetryData.pressure) - 633.00;
}

void readVoltageSensor() {
  int sensorValue = analogRead(sensorPin);                       // Read the voltage from the sensor
  telemetryData.voltage = sensorValue * (inputVoltage / 510.0);  // Convert sensor value to voltage
}

void startClock() {
  if (!clock_running) {
    clock_running = true;
    Serial.println("Stopwatch started.");
  } else {
    Serial.println("Stopwatch is already running.");
  }
}

void stopClock() {
  if (clock_running) {
    clock_running = false;
    Serial.println(" seconds");
  } else {
    Serial.println("Stopwatch is not running.");
  }
}


void runClockAndSetMissionTime() {
  if (clock_running) {
    time_t t = Teensy3Clock.get();
    setTime(t);

    telemetryData.mission_time = String(hour()) + ":" + String(minute()) + ":" + String(second());
    saveTimeToEEPROM();
  }
}

void publishSensorDataToXbee() {
  telemetryData.packet_count++;
  EEPROM.put(PACKET_COUNT_ADDRESS, telemetryData.packet_count);

  String dataToSend = "<T" + constructMessage() + ">";

  // Convert String to const char* for length calculation
  const char *dataCharArray = dataToSend.c_str();
  size_t dataLength = strlen(dataCharArray);

  // Send data to XBee and check if the entire message was sent
  size_t bytesWritten = xbeeSerial.write(dataCharArray, dataLength);

  // Only print to Serial if the data was successfully sent to XBee
  if (bytesWritten == dataLength) {
    Serial.println("Sent data to XBee: " + dataToSend);
  } else {
    Serial.println("Failed to send data to XBee: " + dataToSend);
  }
}

void initializeTelemetryData(TelemetryData &data) {
  data.team_id = "2043";
  data.mode = 'F';
  data.hs_deployed = 'N';
  data.pc_deployed = 'N';
  data.gps_time = "0:0";
  data.state = PRE_LAUNCH;
  // data.cmd_echo = "CMD_ECHO";

  // Read the initial_altitude from EEPROM
  EEPROM.get(INITIAL_ALTITUDE_ADDRESS, initial_altitude);

  // Check if the initial_altitude is uninitialized (EEPROM returns 0xFFFFFFFF if it's uninitialized)
  if (initial_altitude == 0xFFFFFFFF) {
    initial_altitude = 0;
    previousAltitude = 0;
  } else {
    previousAltitude = initial_altitude;
  }

  // Read the packet count from EEPROM
  EEPROM.get(PACKET_COUNT_ADDRESS, data.packet_count);
  // Check if the packet count is uninitialized (EEPROM returns 0xFFFFFFFF if it's uninitialized)
  if (data.packet_count == 0xFFFFFFFF) {
    data.packet_count = 0;
  }

  // Read the mission time from EEPROM
  loadTimeFromEEPROM();
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

void calibrateAltitude(int numIterations) {
  double altitude;
  for (int i = 0; i < numIterations; ++i) {
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    delay(50);
  }
  initial_altitude = altitude;
  previousAltitude = altitude;
  EEPROM.put(INITIAL_ALTITUDE_ADDRESS, initial_altitude);
}

void resetPacketCount() {
  telemetryData.packet_count = 0;
  EEPROM.put(PACKET_COUNT_ADDRESS, telemetryData.packet_count);
}

void resetMissionTime() {
  telemetryData.mission_time = "00:00:00";
  EEPROM.put(EEPROM_HOURS_ADDR, 0);
  EEPROM.put(EEPROM_MINUTES_ADDR, 0);
  EEPROM.put(EEPROM_SECONDS_ADDR, 0);
  // EEPROM.put(MISSION_TIME_ADDRESS, telemetryData.mission_time);
}

float calculateAltitude(float pressure) {
  // Calculate the altitude
  float h = (1 - pow((pressure / 1013.25), (1 / 5.255))) * 44330.77;

  return h;
}

void moveServo(Servo &servo, int start, int end) {
  int step = (start < end) ? 1 : -1;
  for (int pos = start; pos != end; pos += step) {
    servo.write(pos);
  }
  servo.write(end);
}

String getStateName(int state) {
  switch (state) {
    case PRE_LAUNCH:
      return "PRE_LAUNCH";
    case ASCENT:
      return "ASCENT";
    case DESCENT:
      return "DESCENT";
    case PARACHUTE_DEPLOY:
      return "PARACHUTE_DEPLOY";
    case HEAT_SHIELD_RELEASE:
      return "HEAT_SHIELD_RELEASE";
    case LANDED:
      return "LANDED";
    default:
      return "UNKNOWN";
  }
}



String constructMessage() {
  // Construct message using telemetryData struct
  String message = String(telemetryData.team_id) + ", " + String(telemetryData.mission_time) + ", " + String(telemetryData.packet_count) + ", " + String(telemetryData.mode) + ", " + getStateName(telemetryData.state) + ", " + String(telemetryData.altitude) + ", " + String(telemetryData.air_speed) + ", " + String(telemetryData.hs_deployed) + ", " + String(telemetryData.pc_deployed) + ", " + String(telemetryData.temperature) + ", " + String(telemetryData.voltage, 2) + ", " + String(telemetryData.pressure) + ", " + String(telemetryData.gps_time) + ", " + String(telemetryData.gps_altitude, 7) + ", " + String(telemetryData.gps_latitude, 7) + ", " + String(telemetryData.gps_longitude, 7) + ", " + String(telemetryData.gps_sats) + ", " + String(telemetryData.accelerometer_x) + ", " + String(telemetryData.accelerometer_y) + ", " + String(telemetryData.gyroscope_z) + ", " + String(telemetryData.cmd_echo);

  return message;
}