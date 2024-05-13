
#include <Servo.h>
#include <SoftwareSerial.h>

Servo servo1;
Servo servo2;
Servo servo3;

SoftwareSerial xbeeSerial(7, 8); // RX, TX

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
const int payloadSize = 16; // Define the payload size

void setup() {
    servo1.attach(22);
    servo2.attach(23);
    servo3.attach(41);
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

            if (receivedData.trim().equals("a")) {
                for (pos1 = 90; pos1 <= 180; pos1 += 1) {
                    servo1.write(pos1);
                }
            } else if (receivedData.trim().equals("s")) {
                for (pos2 = 90; pos2 >= 0; pos2 -= 1) {
                    servo2.write(pos2);
                }
            } else if (receivedData.trim().equals("d")) {
                for (pos3 = 90; pos3 >= 0; pos3 -= 1) {
                    servo3.write(pos3);
                }
            } else if (receivedData.trim().equals("1")) {
                for (pos1 = 0; pos1 <= 90; pos1 += 1) {
                    servo1.write(pos1);
                }
            } else if (receivedData.trim().equals("2")) {
                for (pos2 = 0; pos2 <= 90; pos2 += 1) {
                    servo2.write(pos2);
                }
            } else if (receivedData.trim().equals("3")) {
                for (pos3 = 0; pos3 <= 90; pos3 += 1) {
                    servo3.write(pos3);
                }
            }

            Serial.println("Received data from XBee: " + receivedData);
            receivedData = ""; // Reset receivedData for the next payload
        }
    }

    if (Serial.available()) {                             // Check if there is data available from Serial monitor
        String dataToSend = Serial.readStringUntil('\n'); // Read data from Serial monitor
        xbeeSerial.print(dataToSend);
        Serial.println("Sent data to XBee: " + dataToSend); // Print the sent data
    }
}
