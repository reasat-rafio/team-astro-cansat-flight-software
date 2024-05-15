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

    initializeTelemetryData(telemetryData);

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

unsigned long previousMillis = 0;
const long interval = 1000; // in milliseconds

void loop() {
    // Get the current time
    unsigned long currentMillis = millis();

    // Check if it's time to toggle the LED
    if (currentMillis - previousMillis >= interval) {
        // Save the last time the LED was toggled
        previousMillis = currentMillis;

        readSensorData();
        publishSensorDataToXbee();
    }

    // Continuously process without delay
    processXBeeCommands();
}

void readSensorData() {
    readAccelerometerData();
    readGPSData();
    readUltrasonicSensor();
    readTemperaturePressureAltitudeValues();
    readVoltageSensor();
}

void processXBeeCommands() {
    static String receivedData = ""; // Define a single buffer for received data

    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedData += character;

        // Check if a complete payload is received
        if (receivedData.length() == command_payload_size || receivedData.length() == servo_payload_size) {
            if (receivedData.length() == command_payload_size) {
                // Process MQTT command
                Serial.println("Received MQTT command from XBee: " + receivedData.trim());
                // Handle MQTT command processing here
            } else if (receivedData.length() == servo_payload_size) {
                // Process servo command
                Serial.println("Received servo command from XBee: " + receivedData.trim());

                servoControl(receivedData);
            }

            receivedData = ""; // Reset receivedData for the next payload
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
    Wire1.write(0x3B); // Starting register of accelerometer data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDRESS, 6, true); // Request 6 bytes of data
    telemetryData.accelerometer_x = Wire1.read() << 8 | Wire1.read();
    telemetryData.accelerometer_y = Wire1.read() << 8 | Wire1.read();
    // We don't need accelerometer_z for this example

    // Read gyroscope data
    Wire1.beginTransmission(MPU6050_ADDRESS);
    Wire1.write(0x47); // Starting register of gyroscope data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDRESS, 2, true); // Request 2 bytes of data
    telemetryData.gyroscope_z = Wire1.read() << 8 | Wire1.read();
}

void readGPSData() {
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            telemetryData.gps_latitude = gps.location.lat();
            telemetryData.gps_longitude = gps.location.lng();
            telemetryData.gps_altitude = gps.altitude.meters();
            telemetryData.gps_sats = gps.satellites.value();
        }
    }
}

void readUltrasonicSensor() {
    telemetryData.ultrasonic_distance = ultrasonic.read();
}

void readTemperaturePressureAltitudeValues() {
    // Read temperature, pressure, and altitude values
    telemetryData.temperature = bmp.readTemperature();
    telemetryData.pressure = bmp.readPressure();
    telemetryData.altitude = bmp.readAltitude(1013.25); // Pass your local sea level pressure for altitude calculation
}

void readVoltageSensor() {
    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
    telemetryData.voltage = sensorValue * (3.3 / 1023.0); // Convert sensor value to voltage (assuming 3.3V reference)
}

void publishSensorDataToXbee() {
    // Construct message using telemetryData struct
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

void initializeTelemetryData(TelemetryData &data) {
    data.team_id = "2043";
    data.mission_time = 0;
    data.packet_count = 0;
    data.mode = 'F';
    data.state = "LAUNCH_WAIT";
    data.air_speed = 10.21;
    data.hs_deployed = 'N';
    data.pc_deployed = 'N';
    data.gps_time = "10:11";
    data.cmd_echo = "CMD_ECHO";
}

String constructMessage() {
    // Construct message using telemetryData struct
    String message = String(telemetryData.team_id) + ", " +
                     String(telemetryData.mission_time) + ", " +
                     String(telemetryData.packet_count) + ", " +
                     String(telemetryData.mode) + ", " +
                     String(telemetryData.state) + ", " +
                     String(telemetryData.altitude) + ", " +
                     String(telemetryData.air_speed) + ", " +
                     String(telemetryData.hs_deployed) + ", " +
                     String(telemetryData.pc_deployed) + ", " +
                     String(telemetryData.temperature) + ", " +
                     String(telemetryData.voltage) + ", " +
                     String(telemetryData.pressure) + ", " +
                     String(telemetryData.gps_time) + ", " +
                     String(telemetryData.gps_altitude) + ", " +
                     String(telemetryData.gps_latitude) + ", " +
                     String(telemetryData.gps_longitude) + ", " +
                     String(telemetryData.gps_sats) + ", " +
                     String(telemetryData.accelerometer_x) + ", " +
                     String(telemetryData.accelerometer_y) + ", " +
                     String(telemetryData.gyroscope_z) + ", " +
                     String(telemetryData.cmd_echo);

    return message;
}
