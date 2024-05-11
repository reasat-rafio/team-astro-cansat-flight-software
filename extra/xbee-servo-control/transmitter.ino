#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(16, 17); // RX, TX - Replace with the pins connected to your XBee module

const int payloadSize = 16; // Define the payload size

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
}

void loop() {
    if (Serial.available()) {                             // Check if there is data available from Serial monitor
        String dataToSend = Serial.readStringUntil('\n'); // Read data from Serial monitor
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
}