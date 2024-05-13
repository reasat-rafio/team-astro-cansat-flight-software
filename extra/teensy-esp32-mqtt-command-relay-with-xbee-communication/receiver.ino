#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(7, 8); // RX, TX

const int payloadSize = 256; // Define the payload size

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
            Serial.println(receivedData);
            Serial.println("Received data from XBee: " + receivedData);
            receivedData = "";
        }
    }
}
