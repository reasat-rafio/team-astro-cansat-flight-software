 
#include <Ultrasonic.h>

// Ultrasonic Sensor Pin Definitions
#define ULTRASONIC_TRIGGER_PIN 2
#define ULTRASONIC_ECHO_PIN 3

// Initialize ultrasonic sensor
Ultrasonic ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
}

void loop() {
  // Read ultrasonic sensor
  long distance = ultrasonic.read();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000); // Wait for a second before taking another reading
}
