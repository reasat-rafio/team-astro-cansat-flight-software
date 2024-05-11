#include <Wire.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp; // Create an instance of the BMP3XX sensor

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!bmp.begin_I2C()) { // Use begin_I2C() for I2C communication
    Serial.println("Could not find a valid BMP3XX sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read temperature, pressure, and altitude values
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25); // Pass your local sea level pressure for altitude calculation

  // Print the values to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  // Delay before taking the next reading
  delay(1000); // Adjust as needed
}
