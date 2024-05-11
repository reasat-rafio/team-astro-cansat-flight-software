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

int pos = 0;
int16_t accelerometer_x, accelerometer_y, gyroscope_z;
float temperature, pressure, altitude;
float gps_latitude, gps_longitude, gps_altitude;
long ultrasonic_distance;
float voltage;

const int VOLTAGE_SENSOR_PIN = A0; // Analog input pin
bool servo_1_rotated = false;
bool servo_2_rotated = false;
bool servo_3_rotated = false;

const int payloadSize = 256; // Define the payload size

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

void loop() {
    readSensorData();
    publishToXbee();
    delay(1000); // Delay before sending the next data
}

void readSensorData() {
    readAccelerometerData();
    readGPSData();
    readUltrasonicSensor();
    readTemperaturePressureAltitudeValues();
    readVoltageSensor();
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

void publishToXbee() {
    String dataToSend = constructMessage();
    if (dataToSend.length() <= payloadSize) {
        xbeeSerial.print(dataToSend);
        for (int i = dataToSend.length(); i < payloadSize; i++) {
            xbeeSerial.print(" "); // Fill the remaining payload with spaces
        }
        Serial.println("Sent data to XBee: " + dataToSend); // Print the sent data
    } else {
        Serial.println("Data exceeds payload size. Sending aborted.");
    }
}

String constructMessage() {
    String message = "";
    message += "Acceleration X: ";
    message += accelerometer_x;
    message += " | Y: ";
    message += accelerometer_y;
    message += " | Gyroscope Z: ";
    message += gyroscope_z;
    message += " | Latitude: ";
    message += gps_latitude;
    message += " | Longitude: ";
    message += gps_longitude;
    message += " | Altitude: ";
    message += gps_altitude;
    message += " | Distance: ";
    message += ultrasonic_distance;
    message += " | Temperature: ";
    message += temperature;
    message += " | Pressure: ";
    message += pressure;
    message += " | Altitude: ";
    message += altitude;
    message += " | Voltage: ";
    message += voltage;
    return message;
}
