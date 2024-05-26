#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(0, 1); // RX, TX - Replace with the pins connected to your XBee module
const int payloadSize = 16;      // Define the payload size

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
}

void loop() {
    static String receivedData = "";

    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedData += character;
        if (receivedData.length() == payloadSize) { // Check if a complete payload is received
            Serial.println("Received data from XBee: " + receivedData);
            receivedData = ""; // Reset receivedData for the next payload
        }
    }
}