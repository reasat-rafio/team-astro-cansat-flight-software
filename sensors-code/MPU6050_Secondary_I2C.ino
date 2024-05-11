 
#include <Wire.h>

#define MPU6050_ADDRESS 0x68 // MPU6050 I2C address

int16_t accelerometer_x, accelerometer_y, gyroscope_z;

void setup() {
  Wire1.begin(); // Initialize Wire1 for I2C communication

  Serial.begin(9600);
  delay(1000);

  // Initialize MPU6050
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(0x6B); // PWR_MGMT_1 register
  Wire1.write(0);    // Wake up MPU6050
  Wire1.endTransmission(true);
}

void loop() {
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

  // Print the values
  Serial.print("Acceleration X: ");
  Serial.print(accelerometer_x);
  Serial.print(" | Y: ");
  Serial.print(accelerometer_y);
  Serial.print(" | Gyroscope Z: ");
  Serial.println(gyroscope_z);

  delay(1000); // Delay for readability
}
