#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(7, 8); // RX, TX
const int payloadSize = 16;      // Define the payload size

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
}

void loop() {
    String dataToSend = "Hello, XBee!"; // Data to send
    if (dataToSend.length() <= payloadSize) {
        xbeeSerial.print(dataToSend);
        for (int i = dataToSend.length(); i < payloadSize; i++) {
            xbeeSerial.print(" "); // Fill the remaining payload with spaces
        }
        Serial.println("Sent data to XBee: " + dataToSend); // Print the sent data
    } else {
        Serial.println("Data exceeds payload size. Sending aborted.");
    }
    delay(1000); // Delay before sending the next data
}