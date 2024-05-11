#include <TinyGPS++.h>

// Neo M8N GPS Sensor Pin Definitions
#define GPS_RX_PIN 0  // Connected to Teensy TX1 (pin 0)
#define GPS_TX_PIN 1  // Connected to Teensy RX1 (pin 1)

// Initialize GPS
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  Serial1.begin(9600); // Initialize serial communication for GPS module
}

void loop() {
  // Read GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: ");
      Serial.println(gps.altitude.meters());
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    }
  }
}
