# Canast 2024 Flight Software (FWS)

This repository hosts the Flight Software (FWS) for the Canast 2024 project, designed to control and monitor a CanSat's systems throughout its mission. The software is tailored for the Teensy microcontroller platform, leveraging a suite of sensors and actuators to ensure mission success.

## Authors

- Al Reasat Rafio
- Anika Tabassum Orchi

## Features

- **Pressure and Temperature Sensing:** Utilizes the BMP390 sensor for high-accuracy atmospheric pressure and temperature readings.
- **Airspeed Measurement:** Incorporates the MS4525DO sensor for precise airspeed measurements.
- **GPS Tracking:** Employs a GPS module for real-time tracking and navigation, using the TinyGPS++ library for data parsing.
- **Flight Phase Detection:** Automatically identifies different flight phases (Pre-Launch, Ascent, Descent, Parachute Deploy, Heat Shield Release, Landed) using altitude and velocity data.
- **Data Logging:** Records critical flight data to EEPROM for post-mission analysis.
- **Actuator Control:** Manages servo motors for mechanical actions such as parachute deployment and heat shield release.

## Hardware Requirements

- **Teensy Microcontroller:** The core platform for the FWS.
- **BMP390 Sensor:** For atmospheric pressure and temperature measurements.
- **MS4525DO Sensor:** For airspeed determination.
- **GPS Module:** Connected via designated RX and TX pins.
- **Servo Motors:** For deployment mechanisms.
- **EEPROM:** Utilized for data logging purposes.

## Software Dependencies

- **Adafruit_BMP3XX Library:** For BMP390 sensor interfacing.
- **TinyGPS++ Library:** For GPS data parsing.
- **TimeLib Library:** For time management functions.
- **SoftwareSerial Library:** Enables serial communication on digital pins.
- **Wire Library:** For I2C communication with sensors.

## Installation

1. Install the Arduino IDE.
2. Download and install the required libraries through the Arduino Library Manager.
3. Clone this repository and open `fws.ino` in the Arduino IDE.
4. Configure the IDE for the Teensy board and select the correct port.
5. Compile and upload the firmware to the Teensy microcontroller.

## Configuration

- **Sensor Addresses:** Verify the I2C addresses for the BMP390 and other sensors match your setup.
- **GPS Pins:** Ensure the GPS module's RX and TX pins are correctly configured.
- **Sea Level Pressure:** Adjust the `SEA_LEVEL_PRESSURE_HPA` constant for accurate altitude calculations based on your local sea level pressure.

## Usage

Upon successful firmware upload, the CanSat will begin its mission, logging data and transitioning through flight phases based on sensor inputs. Monitor the serial output for real-time data and diagnostics.

## Contributing

We welcome contributions to the Canast 2024 FWS. Please fork the repository, commit your changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
