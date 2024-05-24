#include "ms4525do.h"
#include <Wire.h> // Include the Wire2 library

bfs::Ms4525do pitotSensor;
float dynamicPressure, temperature, airspeed;
const float GAS_CONSTANT_AIR = 287.058; // Specific gas constant for dry air in J/kg/K

const int NUM_READINGS = 101;
float baseValues[NUM_READINGS];
float pressureDifferences[NUM_READINGS];
const float VALUE_INCREMENT = 0.52606034;

void setup() {
    // Initialize Serial for displaying data
    Serial.begin(9600);
    while (!Serial) {
    }
    Wire2.begin(); // Initialize Wire2
    Wire2.setClock(400000);

    // Configure the Pitot tube sensor with I2C address 0x28, on bus 0, with a -1 to +1 PSI range
    pitotSensor.Config(&Wire2, 0x28, 1.0f, -1.0f);

    initializeBaseValues();

    // Start communication with the pressure transducer
    if (!pitotSensor.Begin()) {
        Serial.println("Error communicating with sensor");
        while (1) {
        }
    }
}

void initializeBaseValues() {
    // Initialize the array with base values
    int middleIndex = NUM_READINGS / 2;

    for (int i = 0; i < NUM_READINGS; i++) {
        baseValues[i] = (i - middleIndex) * VALUE_INCREMENT;
    }
}

void computePressureDifferences(float measuredPressure) {
    for (int i = 0; i < NUM_READINGS; i++) {
        pressureDifferences[i] = measuredPressure - baseValues[i];
    }
}

float getClosestBaseValue(float measuredPressure) {
    float minDifference = abs(pressureDifferences[0]);
    float closestBaseValue = pressureDifferences[0];

    for (int i = 1; i < NUM_READINGS; i++) { // Start from 1 since 0 is already checked
        float currentDifference = abs(pressureDifferences[i]);
        if (currentDifference < minDifference) {
            minDifference = currentDifference;
            closestBaseValue = pressureDifferences[i];
        }
    }
    return closestBaseValue;
}

void loop() {
    // Read the sensor data
    if (pitotSensor.Read()) {
        dynamicPressure = pitotSensor.pres_pa();
        temperature = pitotSensor.die_temp_c();

        computePressureDifferences(dynamicPressure);
        float adjustedPressure = getClosestBaseValue(dynamicPressure);

        // Calculate airspeed using the adjusted pressure and constant air density (at sea level and 15°C)
        const float AIR_DENSITY_SEA_LEVEL = 1.225; // Air density in kg/m^3
        airspeed = sqrt(2 * abs(adjustedPressure) / AIR_DENSITY_SEA_LEVEL);

        // Display the data
        Serial.print("Dynamic Pressure: ");
        Serial.print(dynamicPressure, 8);
        Serial.print(" | Adjusted Pressure: ");
        Serial.print(adjustedPressure, 8);
        Serial.print(" | Airspeed: ");
        Serial.print(airspeed, 8);
        Serial.print(" | Temperature: ");
        Serial.print(temperature, 8);
        Serial.println();
    }
    delay(1000);
}
