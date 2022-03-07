#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* This sketch was part of the development process of INFiniti one - it is the next stage
   of the sketch NAUSOLARIS_1 with GPS code from sketch simple_test_gps_decoder integrated
   
   Tested for GPS module: UBLOX MAX-M8c pico breakout with chip antenna
   Written by Miķelis Putnieks on 07/06/2021, as part of project Nausolaris
*/

void read_voltage();
void read_temperature();
void write_data();
void CheckGPS();
void ProcessGPSLine();
void ProcessGNGGACommand();

#define ONE_WIRE_BUS 2         // Temperature data wire is connected to the Arduino digital pin 2
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass oneWire reference to DallasTemperature library
SoftwareSerial ss(3, 4);       // 9600-baud serial GPS device hooked up on digital pins 3 and 4

// - - -    GPS variables       - - -
byte GPSBuffer[82];
byte GPSIndex = 0;
unsigned int GPS_Satellites = 0;
unsigned int GPS_Altitude = 0;
String GPS_Lat;
String GPS_Lon;
String GPS_Time;
bool newData = false;

// - - -   Voltage variables    - - -
int analogVoltage = A7;
float voltage = 0.0;
float R1 = 100000; //voltage reader/divider resistor 1
float R2 = 100000; //voltage reader/divider resistor 2

// - - -  Temperature variables - - -
float t1 = 0;
float t2 = 0;
int deviceCount = 0;  // a count of digital temperature sensors

// - - -    Other variables     - - -
int iter = 0;            // a counter for the loop iterations

void setup(){
  sensors.begin();      // Start up the sensor library
  Serial.begin(115200); // comms with the SD card
  ss.begin(9600);       // comms with the GPS module
  
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount(); // locate (temp. sensor) devices on the bus
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  
  pinMode(analogVoltage, INPUT);
}

void loop(){
  iter++;
  read_voltage();
  read_temperature();

  // -- For one second we parse GPS data and report some key values --
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;){
    CheckGPS();
  }

  // -- Was new data received? if yes do we have GNGGA satellite connection? yes? well then lets display the decoded data. --
  if (!newData){  // If no GNGGA message was found
    Serial.println("GPS GNGGA not received! Check wiring connections!");
  }
  /*
  else{
    Serial.println(" ");
    Serial.print("Satellites :");
    Serial.println(GPS_Satellites);
    if(GPS_Satellites >= 1){
      Serial.print("Altitude :");
      Serial.println(GPS_Altitude);
      Serial.print("Time :");
      Serial.println(GPS_Time);
      Serial.println("Coordinates :");
      Serial.print("Lattitude :");
      Serial.println(GPS_Lat);
      Serial.print("Longitude :");
      Serial.println(GPS_Lon);
    }
    Serial.println("- - - - - - - - - -");
    Serial.println(" ");
  }
  */
  delay(1000);  // A delay of a second after reading and parsing the gps data
  write_data();
}

void read_temperature(){
  sensors.requestTemperatures(); // Call to issue a global temperature and Requests to all devices on the bus
  t1 = sensors.getTempCByIndex(0); // "byIndex" - One can have more than one IC on the same bus. 0 refers to the first IC on the wire
  t2 = sensors.getTempCByIndex(1);
}

void read_voltage(){
  int value = analogRead(analogVoltage);
  voltage = (value * 5.0 / 1024.0)/(R2/(R1+R2)); // default conversion for Arduino analog input using 0-5V mapping in 1024 parts, added support for voltage divider conversion.
}

void write_data(){
  String dataString = "";
  dataString += String(iter);
  dataString += ","+String(t1);
  dataString += ","+String(t2);
  dataString += ","+String(voltage);
  dataString += ","+GPS_Lat;
  dataString += ","+GPS_Lon;
  dataString += ","+GPS_Time;
  dataString += ","+String(GPS_Satellites);
  dataString += ","+String(GPS_Altitude);
  Serial.println(dataString);
}

void CheckGPS(){
  int inByte;
  while (ss.available()){
    inByte = ss.read();
    //Serial.write(inByte); // uncomment this line if you want to see the GPS data flowing
 
    if ((inByte =='$') || (GPSIndex >= 80)){
      GPSIndex = 0;
    }
    if (inByte != '\r'){
      GPSBuffer[GPSIndex++] = inByte;
    }
    if (inByte == '\n'){
      ProcessGPSLine();
      GPSIndex = 0;
    }
  }
}

void ProcessGPSLine(){
  if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'N') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')){
    Serial.println("GNGGA Detected!");
    Serial.write(GPSBuffer, 82);
    ProcessGNGGACommand();
    newData = true;
  }
}

void ProcessGNGGACommand(){
  int i, j, k, IntegerPart;
  unsigned int Altitude = 0;
  bool trimmed = false;
  
  // $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  //                                               =====  <-- altitude in field 8

  IntegerPart = 1;
  GPS_Satellites = 0;
  GPS_Lat = String(" ");
  GPS_Lon = String(" ");
  GPS_Time = String(" ");
  
  for (i=7, j=0, k=0; (i<GPSIndex) && (j<9); i++){ // We start at 7 so we ignore the '$GNGGA,'
    if (GPSBuffer[i] == ','){
      j++;    // Segment index
      k=0;    // Index into target variable
      IntegerPart = 1;
      trimmed = false;
    }
    else{

      if (j == 0){  //Time
        k++;
        if ((char)GPSBuffer[i] == '.'){
          trimmed = true;
        }
        if (!trimmed){
          if(k%2 && k>1){
            GPS_Time += String(":");
          }
        GPS_Time += String((char)GPSBuffer[i]);
        }
      }
      
      else if (j == 1){  //Latitude
        GPS_Lat += String((char)GPSBuffer[i]);
      }

      else if (j == 2){  //Latitude N or S
        GPS_Lat += String("\"");
        GPS_Lat += String((char)GPSBuffer[i]);
      }
      
      else if (j == 3){  //Longitude
        GPS_Lon += String((char)GPSBuffer[i]);
      }

      else if (j == 4){  //Longitude E or W
        GPS_Lon += String("\"");
        GPS_Lon += String((char)GPSBuffer[i]);
      }
      
      else if (j == 6){  //Satellite Count
        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9')){
          GPS_Satellites = GPS_Satellites * 10;
          GPS_Satellites += (unsigned int)(GPSBuffer[i] - '0');
        }
      }
      
      else if (j == 8){  // Altitude
        if ((GPSBuffer[i] >= '0') && (GPSBuffer[i] <= '9') && IntegerPart){
          Altitude = Altitude * 10;
          Altitude += (unsigned int)(GPSBuffer[i] - '0');
        }
        else{
          IntegerPart = 0;
        }
      }
    }
    GPS_Altitude = Altitude;
  }
}

