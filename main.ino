#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Ultrasonic.h>
#include <Wire.h>

#define MPU6050_ADDRESS 0x68     // MPU6050 I2C address
#define GPS_RX_PIN 0             // Connected to Teensy TX1 (pin 0)
#define GPS_TX_PIN 1             // Connected to Teensy RX1 (pin 1)
#define ULTRASONIC_TRIGGER_PIN 2 // Ultrasonic Sensor Pin Definitions
#define ULTRASONIC_ECHO_PIN 3    // Ultrasonic Sensor Pin Definitions

SoftwareSerial xbeeSerial(7, 8);                                    // RX, TX
TinyGPSPlus gps;                                                    // Initialize GPS
Adafruit_BMP3XX bmp;                                                // Create an instance of the BMP3XX sensor
Ultrasonic ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN); // Initialize ultrasonic sensor
Servo servo1;
Servo servo2;
Servo servo3;

const char team_id[] = "2043";
int pos = 0;
int16_t accelerometer_x, accelerometer_y, gyroscope_z;
float temperature, pressure, altitude;
float gps_latitude, gps_longitude, gps_altitude;
int gps_sats = 0;
long ultrasonic_distance;
float voltage;
int packet_count = 0;
char mode = 'F';
String state = "LAUNCH_WAIT";
float air_speed = 10.21;
char hs_deployed = 'N';
char pc_deployed = 'N';
int mission_time = 0;
String gps_time = "10:11";
String cmd_echo = "CMD_ECHO";

const int VOLTAGE_SENSOR_PIN = A0; // Analog input pin
bool servo_1_rotated = false, servo_2_rotated = false, servo_3_rotated = false;
int servo_1_position = 0, servo_2_position = 0, servo_3_position = 0;

const int telemetry_payload_size = 180; // Define the payload size
const int servo_payload_size = 16;
const int command_payload_size = 32;

void setup() {
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
    Wire1.begin();
    Wire.begin();
    Serial.begin(9600);
    Serial1.begin(9600);
    servo1.attach(22);
    servo2.attach(23);
    servo3.attach(41);

    delay(1000);

    while (!Serial)
        ;

    if (!bmp.begin_I2C()) { // Use begin_I2C() for I2C communication
        Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
        while (1)
            ;
    }

    // Initialize MPU6050
    Wire1.beginTransmission(MPU6050_ADDRESS);
    Wire1.write(0x6B); // PWR_MGMT_1 register
    Wire1.write(0);    // Wake up MPU6050
    Wire1.endTransmission(true);
}

unsigned long lastSensorDataTime = 0;          // Stores the timestamp for the last sensor data transmission
const unsigned long sensorDataInterval = 1000; // Delay between sensor data transmissions (1 second)

void loop() {
    // Check if it's time to read and publish sensor data
    if (millis() - lastSensorDataTime >= sensorDataInterval) {
        readSensorData();
        publishSensorDataToXbee();
        lastSensorDataTime = millis(); // Update timestamp
    }

    // Continuously process without delay
    processXBeeServoCommands();
    processXBeeMqttCmdCommands();
}

void readSensorData() {
    readAccelerometerData();
    readGPSData();
    readUltrasonicSensor();
    readTemperaturePressureAltitudeValues();
    readVoltageSensor();
}

static String receivedMqttCMDData = "";
void processXBeeMqttCmdCommands() {
    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedMqttCMDData += character;
        if (receivedMqttCMDData.length() == command_payload_size) { // Check if a complete payload is received
            Serial.println("Received data from XBee: ");
            Serial.print(receivedMqttCMDData.trim());
            receivedMqttCMDData = "";
        }
    }
}

static String receivedServoData = "";
void processXBeeServoCommands() {
    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedServoData += character;
        if (receivedServoData.length() == servo_payload_size) { // Check if a complete payload is received
            Serial.println(receivedServoData);

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

            Serial.println("Received data from XBee: " + receivedServoData);
            receivedServoData = ""; // Reset receivedData for the next payload
        }
    }
}

void readAccelerometerData() {
    // Read accelerometer data
    Wire1.beginTransmission(MPU6050_ADDRESS);
    Wire1.write(0x3B); // Starting register of accelerometer data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDRESS, 6, true); // Request 6 bytes of data
    accelerometer_x = Wire1.read() << 8 | Wire1.read();
    accelerometer_y = Wire1.read() << 8 | Wire1.read();
    // We don't need accelerometer_z for this example

    // Read gyroscope data
    Wire1.beginTransmission(MPU6050_ADDRESS);
    Wire1.write(0x47); // Starting register of gyroscope data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDRESS, 2, true); // Request 2 bytes of data
    gyroscope_z = Wire1.read() << 8 | Wire1.read();
}

void readGPSData() {
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            gps_latitude = gps.location.lat();
            gps_longitude = gps.location.lng();
            gps_altitude = gps.altitude.meters();
        }
    }
}

void readUltrasonicSensor() {
    ultrasonic_distance = ultrasonic.read();
}

void readTemperaturePressureAltitudeValues() {
    // Read temperature, pressure, and altitude values
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25); // Pass your local sea level pressure for altitude calculation
}

void readVoltageSensor() {
    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
    voltage = sensorValue * (3.3 / 1023.0); // Convert sensor value to voltage (assuming 3.3V reference)
}

void publishSensorDataToXbee() {
    String dataToSend = constructMessage();
    if (dataToSend.length() <= telemetry_payload_size) {
        xbeeSerial.print(dataToSend);
        for (int i = dataToSend.length(); i < telemetry_payload_size; i++) {
            xbeeSerial.print(" "); // Fill the remaining payload with spaces
        }
        Serial.println("Sent data to XBee: " + dataToSend); // Print the sent data
    } else {
        Serial.println("Data exceeds payload size. Sending aborted.");
    }
}

String constructMessage() {
    String message = String(team_id) + ", " +
                     String(mission_time) + ", " +
                     String(packet_count) + ", " +
                     String(mode) + ", " +
                     String(state) + ", " +
                     String(altitude) + ", " +
                     String(air_speed) + ", " +
                     String(hs_deployed) + ", " +
                     String(pc_deployed) + ", " +
                     String(temperature) + ", " +
                     String(voltage) + ", " +
                     String(pressure) + ", " +
                     String(gps_time) + ", " +
                     String(gps_altitude) + ", " +
                     String(gps_latitude) + ", " +
                     String(gps_longitude) + ", " +
                     String(gps_sats) + ", " +
                     String(accelerometer_x) + ", " +
                     String(accelerometer_y) + ", " +
                     String(gyroscope_z) + ", " +
                     String(cmd_echo);

    return message;
}