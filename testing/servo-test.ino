#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

void setup() {
    servo1.attach(22);
    servo2.attach(23);
    servo3.attach(41);
}

void loop() {
    // Rotate servo1 to 90 degrees
    for (pos1 = 0; pos1 <= 90; pos1 += 1) {
        servo1.write(pos1);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds

    // Rotate servo1 back to 0 degrees
    for (pos1 = 90; pos1 >= 0; pos1 -= 1) {
        servo1.write(pos1);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds

    // Rotate servo2 to 90 degrees
    for (pos2 = 0; pos2 <= 90; pos2 += 1) {
        servo2.write(pos2);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds

    // Rotate servo2 back to 0 degrees
    for (pos2 = 90; pos2 >= 0; pos2 -= 1) {
        servo2.write(pos2);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds

    // Rotate servo3 to 90 degrees
    for (pos3 = 0; pos3 <= 90; pos3 += 1) {
        servo3.write(pos3);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds

    // Rotate servo3 back to 0 degrees
    for (pos3 = 90; pos3 >= 0; pos3 -= 1) {
        servo3.write(pos3);
        delay(15); // Adjust the delay if needed for your servo
    }
    delay(5000); // Wait for 5 seconds
}