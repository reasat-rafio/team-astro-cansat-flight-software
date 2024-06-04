const int sensorPin = A0; // Define the pin for the voltage sensor
const float inputVoltage = 8.0; // Input voltage in volts
const float desiredOutputVoltage = 5.0; // Desired output voltage in volts
const float R1 = 10000.0; // Resistance of R1 in ohms

void setup() {
  Serial.begin(9600); // Start serial communication
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read the voltage from the sensor
  float voltageOut = sensorValue * (inputVoltage / 510.0); // Convert sensor value to voltage
  float R2 = (voltageOut * R1) / (inputVoltage - voltageOut); // Calculate R2 using the measured output voltage
  
  Serial.print("Output Voltage: ");
  Serial.print(voltageOut, 2); // Print output voltage with 2 decimal places
  Serial.println(" V");

  Serial.print("Estimated R2: ");
  Serial.print(R2, 2); // Print estimated R2 value with 2 decimal places
  Serial.println(" ohms");

  delay(1000); // Wait for 1 second before reading again (adjust as needed)
}
