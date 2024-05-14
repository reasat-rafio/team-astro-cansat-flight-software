#include "ms4525do.h"

bfs::Ms4525do pres;
float Dynamic_Pressure, Temperature, airspeed ;
const float R = 287.058 ; //Specific gas constant for dry air= 287.058 j/kg/k

void setup() {
  /* Serial to display data */
  Serial.begin(9600);
  while(!Serial){}
  Wire.begin();
  Wire.setClock(400000);
  /*
  * I2C address of 0x28, on bus 0, with a -1 to +1 PSI range
  */
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  /* Starting communication with the pressure transducer */
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while(1){}
  }
}

void loop() {
  /* Read the sensor */

  if (pres.Read()) {

     Dynamic_Pressure = getBaseValue(pres.pres_pa());
   /*  // Calculate air density
     airDensity = atmosphericPressure / (R * temperatureKelvin); // atmospheric pressure(static pressure) from BMP sensor  */

     const float airDensity= 1.225 ; // Air density in kg/m^3 (at sea level and 15°C)
    // Calculate airspeed (using the calculated air density)
    airspeed = sqrt(2 * Dynamic_Pressure / airDensity);

    Temperature = pres.die_temp_c();

    /* Display the data */
    Serial.print("Dynamic_Pressure: ");
    Serial.print(Dynamic_Pressure, 6);
    Serial.print("\t\t");
    Serial.print("Air Speed: ");
    Serial.print(airspeed, 6);
    Serial.print("\t\t");
    Serial.print("Temperature: ");
    Serial.print(Temperature, 6);
    Serial.print("\n");
  }
  delay(1000);
}

float getBaseValue(float totalValue) {
    // Define the offset values
    float offset1 = -51.027618;
    float offset2 = -45.766921;
    float offset3 = -39.454174;
    float offset4 = -57.340366;

    // Calculate the difference between the total value and each offset
    float diff1 = totalValue - offset1;
    float diff2 = totalValue - offset2;
    float diff3 = totalValue - offset3;
    float diff4 = totalValue - offset4;

    // Find the closest offset value to the total value
    float minDiff = abs(diff1);
    float baseValue = diff1;
    if (abs(diff2) < minDiff) {
        minDiff = abs(diff2);
        baseValue = diff2;
    }
    if (abs(diff3) < minDiff) {
        minDiff = abs(diff3);
        baseValue = diff3;
    }
    if (abs(diff4) < minDiff) {
        minDiff = abs(diff4);
        baseValue = diff4;
    }

    return baseValue;
}

