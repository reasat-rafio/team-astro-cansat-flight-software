#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <MPU6050.h>
#include <Wire.h>

// Create instances of the sensors
Adafruit_BMP3XX bmp;
MPU6050 mpu;

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
#define LANDING_VELOCITY_THRESHOLD 1  // Velocity threshold to confirm landing

// Variables
int phase = PRE_LAUNCH;
float previousAltitude;
bool isLanded = false;

void setup() {
    Serial.begin(9600);

    // Initialize BMP390
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while (1)
            ;
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1)
            ;
    }

    previousAltitude = bmp.readAltitude(1013.25); // Sea level pressure in hPa
}

void loop() {
    float currentAltitude = bmp.readAltitude(1013.25);
    float altitudeChange = currentAltitude - previousAltitude;

    // Read accelerometer values
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float acceleration = sqrt(sq(ax) + sq(ay) + sq(az)) / 16384.0; // Convert to 'g'

    switch (phase) {
    case PRE_LAUNCH:
        if (altitudeChange > ASCENT_ALTITUDE_THRESHOLD) {
            phase = ASCENT;
            Serial.println("Phase 1: Ascent");
        }
        break;

    case ASCENT:
        if (altitudeChange < -DESCENT_ALTITUDE_CHANGE) {
            phase = DESCENT;
            Serial.println("Phase 2: Descent");
        }
        break;

    case DESCENT:
        if (currentAltitude < DESCENT_ALTITUDE_LIMIT) {
            phase = SEPARATION;
            Serial.println("Phase 3: Separation");
        }
        break;

    case SEPARATION:
        if (currentAltitude < SEPARATION_ALTITUDE_LIMIT) {
            phase = PARACHUTE_DEPLOY;
            Serial.println("Phase 4: Parachute Deploy");
        }
        break;

    case PARACHUTE_DEPLOY:
        if (acceleration < LANDING_VELOCITY_THRESHOLD) {
            isLanded = true;
        }
        if (isLanded) {
            phase = LANDED;
            Serial.println("Phase 5: Landed");
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
