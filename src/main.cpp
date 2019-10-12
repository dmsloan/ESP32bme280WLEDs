#include <Arduino.h>

/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  You have to recalculate the pressure according to the formula:
Prel = Pabs + h/8.3

Prel is relative pressure (recalculating to the sea level) [hPa]
Pabs is absolute pressure at the messurement place/site [hPa]
h is altitude (height above sea level) [m]

Elevation for IBE is about 270 m. (886 ft)
 ***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1015.2)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime = 10;

//delayTime = 10;

void printValues() {
    Serial.print(millis());
    Serial.print(" Temperature = ");
    Serial.print((bme.readTemperature()*9/5)+32); // convert to °F
    Serial.print("°F ");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0f); // the "f" after the 100.0 tell the complier the number is a float not a double
    Serial.print(" hPa ");

    Serial.print(bme.readPressure() / 3386.389f); // convert pa to inHG
    Serial.print(" inHG ");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.2808);
    Serial.print(" ft ");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.print("% ");

    Serial.println();
}
void flashLEDs(){
    for (int j = 0; j < 10; j++){
        for (int i=7; i <= 12; i++){
            digitalWrite(i, HIGH);
        }
        delay(delayTime*100);
        for (int i=7; i <= 12; i++){
            digitalWrite(i, LOW);
        }
        delay(delayTime*100);
    }

    for (int j=1; j <= 100; j++){
        for (int i=7; i <= 12; i++){
            digitalWrite(i, HIGH);
            delay(delayTime);
            digitalWrite(i, LOW);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("ESP32BME280WLEDs.ino"));
    Wire.begin(23,22); //(SDA, SCL)this is not needed because it is defined in the board description. change here if needed.

    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);

    bool status;
    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

    Serial.println();
}

void loop() { 
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme.takeForcedMeasurement(); // has no effect in normal mode. If you dont use this the sensor will never update measurement
    printValues();
    flashLEDs();
}