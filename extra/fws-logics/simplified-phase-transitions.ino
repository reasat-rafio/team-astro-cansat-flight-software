#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Create an instance of the BMP280 sensor
Adafruit_BMP280 bmp;

// Define phase constants
#define PRE_LAUNCH 0
#define ASCENT 1
#define DESCENT 2
#define SEPARATION 3
#define PARACHUTE_DEPLOY 4
#define LANDED 5

// Thresholds
#define ASCENT_ALTITUDE_THRESHOLD 100 // Altitude increase threshold for ascent
#define DESCENT_ALTITUDE_CHANGE 3     // Altitude decrease threshold for descent
#define DESCENT_ALTITUDE_LIMIT 500    // Altitude limit for separation
#define SEPARATION_ALTITUDE_LIMIT 200 // Altitude limit for parachute deploy
#define LANDING_TIME_THRESHOLD 5      // Time threshold to confirm landing in seconds

// Variables
int state = PRE_LAUNCH;
unsigned long lastTime;
float previousAltitude;
bool isLanded = false;

void setup() {
    Serial.begin(9600);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1)
            ;
    }

    // Set up sensor readings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    lastTime = millis();
    previousAltitude = bmp.readAltitude(1013.25);
}

void loop() {
    unsigned long currentTime = millis();
    float currentAltitude = bmp.readAltitude(1013.25);
    float altitudeChange = currentAltitude - previousAltitude;

    switch (state) {
    case PRE_LAUNCH:
        if (altitudeChange > ASCENT_ALTITUDE_THRESHOLD && (currentTime - lastTime) <= 3000) {
            state = ASCENT;
            lastTime = currentTime;
        }
        break;

    case ASCENT:
        if (altitudeChange < -DESCENT_ALTITUDE_CHANGE && (currentTime - lastTime) <= 3000) {
            state = DESCENT;
            lastTime = currentTime;
        }
        break;

    case DESCENT:
        if (currentAltitude < DESCENT_ALTITUDE_LIMIT) {
            state = SEPARATION;
            lastTime = currentTime;
        }
        break;

    case SEPARATION:
        if (currentAltitude < SEPARATION_ALTITUDE_LIMIT) {
            state = PARACHUTE_DEPLOY;
            lastTime = currentTime;
        }
        break;

    case PARACHUTE_DEPLOY:
        if ((currentTime - lastTime) > 5000) {
            isLanded = true;
        }
        if (isLanded) {
            state = LANDED;
        }
        break;

    case LANDED:
        Serial.println("Rocket has landed safely.");
        // Execute landing procedures, e.g., save data, end telemetry
        saveData();
        turnOnBuzzer();
        endTelemetry();
        break;
    }

    previousAltitude = currentAltitude;
    delay(100); // Delay for sensor reading stabilization
}

void saveData() {
    // Implement data saving logic here
    Serial.println("Data saved to SD card.");
}

void turnOnBuzzer() {
    // Implement buzzer activation logic here
    Serial.println("Buzzer activated.");
}

void endTelemetry() {
    // Implement telemetry ending logic here
    Serial.println("Telemetry ended.");
}
