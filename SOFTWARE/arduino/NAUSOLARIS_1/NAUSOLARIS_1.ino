#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* This sketch was part of the development process of INFiniti one - it reads the temperature data
   of two DS18B20 sensors, along with the voltage of a battery and stores it in a SD card.
   This code is further built upon in the next examples.
   
   Tested for: SparkFun OpenLog module, empty 1GB SD card
   Written by Miķelis Putnieks on 27/05/2021 as part of project Nausolaris
*/

void read_voltage();
void read_temperature();
void write_data();

#define ONE_WIRE_BUS 2               // Temperature data wire is connected to the Arduino digital pin 2
OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass oneWire reference to DallasTemperature library

int analogVoltage = A7;
float voltage = 0.0;
float R1 = 100000; //voltage divider resistor 1
float R2 = 100000; //voltage divider resistor 2

float t1 = 0;
float t2 = 0;

int deviceCount = 0;  // a count of digital temperature sensors
int i = 0;            // a counter for the loop iterations

void setup(){
  sensors.begin(); // Start up the library
  Serial.begin(9600);
  while (!Serial){
    ; // wait for serial port to connect
  }
  // locate (temp. sensor) devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");
  
  pinMode(analogVoltage, INPUT);
}

void loop(){
  i++;
  read_voltage();
  read_temperature();
  delay(1000);
  write_data();
}

void read_temperature(){
  sensors.requestTemperatures(); // Call to issue a global temperature and requests to all devices on the bus
  t1 = sensors.getTempCByIndex(0); // "byIndex" - One can have more than one IC on the same bus. 0 refers to the first IC on the wire
  t2 = sensors.getTempCByIndex(1);
}

void read_voltage(){
  int value = analogRead(analogVoltage);
  voltage = (value * 5.0 / 1024.0)/(R2/(R1+R2)); // default conversion for Arduino analog input using 0-5V mapping in 1024 parts, added support for voltage divider conversion.
}

void write_data(){
  String dataString = "";
  dataString += String(i);
  dataString += ",";
  dataString += String(t1);
  dataString += ",";
  dataString += String(t2);
  dataString += ",";
  dataString += String(voltage);
  Serial.println(dataString);
}
