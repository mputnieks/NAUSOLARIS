#include <Arduino.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* This sketch was part of the development process of INFiniti one - it is the next stage
   of the sketch NAUSOLARIS_2 combined with LoRa code from sketch Nausolaris_4 integrated
   
   Tested for GPS module: UBLOX MAX-M8c pico breakout with chip antenna
   Tested for LoRa module: HopeRF RFM95W-915S2 LoRa(TM) Transceiver
   Written by Miķelis Putnieks on 01/10/2021, as part of project Nausolaris
*/

void read_voltage();      // reads and stores the current battery voltage
void read_temperature();  // reads and stores the temperature t1 and t2 from two sensors, can be extended to more (Nausolaris CORE board allows for up to 4)
void write_data();        // a deprecated method that was used during testing to assemble the string to be sent and stored
void checkGPS();
void processGPSLine();
void processGNGGACommand();
void transmitData();
//void CreateTXLine();
void CreateTXLine_v2();
void test();
char Hex(char Character);

#define ONE_WIRE_BUS 2         // Temperature data wire is connected to the Arduino digital pin 2
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass oneWire reference to DallasTemperature library
SoftwareSerial ss(3, 4);       // 9600-baud serial GPS device hooked up on digital pins 3 and 4
SX1278 radio = new Module(7, 5, 8, 6);
/* SX1278 has the following connections:
  NSS pin:   D7
  DIO0 pin:  D5
  RESET pin: D8
  DIO1 pin:  D6
*/

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
String dataString = "";  // a string to be transmitted via LoRa, stored on SD card
const String callsign = "NSolaris-R"; // the amateur radio callsign to be added to the transmission
#define SENTENCE_LENGTH 100     // Maximum length of telemetry line to send
#define LORA_PREFIX "$$"        // Some older LoRa software does not accept a prefix of more than 2x "$"
char Sentence[SENTENCE_LENGTH];

void setup() {
  sensors.begin();      // Start up the sensor library
  Serial.begin(115200); // comms with the SD card
  ss.begin(9600);       // comms with the GPS module
  
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount(); // locate (temperature sensor) devices on the bus
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  
  pinMode(analogVoltage, INPUT);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  //int state = radio.begin();  //default settings
  int state = radio.begin(432.662, 62.5, 8, 8);
  //frequency: 432.662 MHz
  //bandwidth: 62.5 kHz
  //spread factor: 8
  //coding rate: 8

  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  iter++;
  read_voltage();
  read_temperature();

  // -- For one second we parse GPS data and report some key values --
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;){
    checkGPS();
  }
  // -- Was new data received? if yes do we have GNGGA satellite connection? yes? well then lets display the decoded data. --
  if (!newData){  // If no GNGGA message was found
    Serial.println("GPS GNGGA not received! Check wiring connections!");
  }

  delay(7900);  // A delay in ms after reading and parsing the gps data
  write_data();
  //CreateTXLine();
  CreateTXLine_v2();
  transmitData();
  delay(100);  // a delay in ms after transmitting
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
  dataString = "$$";
  dataString += callsign;
  dataString += ","+String(iter);
  dataString += ","+GPS_Time;
  dataString += ","+GPS_Lat;
  dataString += ","+GPS_Lon;
  dataString += ","+String(GPS_Altitude);
  dataString += ","+String(GPS_Satellites);
  dataString += ","+String(t1);
  dataString += ","+String(voltage);
  dataString += ","+String(t2);
  Serial.println(dataString);
}

void CreateTXLine_v2() {
   int Count, i, j, k;
   unsigned int CRC, xPolynomial;
   
   char m[dataString.length()-3];
   for(k = 0;k < dataString.length(); k++){
    m[k]=dataString[k];
   }
   //Serial.println(m);

   sprintf(Sentence,
      "%s",
      m
   );

   Count = strlen(Sentence);
   //Serial.println(Sentence);

   // Calc CRC
   CRC = 0xffff;           // Seed
   xPolynomial = 0x1021;
   
   //for (i = 0; i < Count; i++){   //uncomment when testing CRC with string "habitat"
   for (i = strlen(LORA_PREFIX); i < Count; i++){   // For speed, repeat calculation instead of looping for each bit
      CRC ^= (((unsigned int)Sentence[i]) << 8);
      for (j=0; j<8; j++){
        if (CRC & 0x8000)
            CRC = (CRC << 1) ^ 0x1021;
        else
            CRC <<= 1;
      }
   }

   Sentence[Count++] = '*';
   Sentence[Count++] = Hex((CRC >> 12) & 15);
   Sentence[Count++] = Hex((CRC >> 8) & 15);
   Sentence[Count++] = Hex((CRC >> 4) & 15);
   Sentence[Count++] = Hex(CRC & 15);
   Sentence[Count++] = '\n';  
   Sentence[Count++] = '\0';

   
   //the string "habitat" should checksum to 0x3EFB.
   //Serial.println(Sentence);
}

char Hex(char Character){
  char HexTable[] = "0123456789ABCDEF";
  return HexTable[Character];
}

void checkGPS(){
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
      processGPSLine();
      GPSIndex = 0;
    }
  }
}

void processGPSLine(){
  if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'N') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')){
    //Serial.println("GNGGA Detected!");
    //Serial.write(GPSBuffer, 82); //uncomment to see the raw GNGGA line
    processGNGGACommand();
    newData = true;
  }
}

void processGNGGACommand(){
  int i, j, k, IntegerPart;
  unsigned int Altitude = 0;
  bool trimmed = false;
  
  // $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47     (example data)
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

void transmitData(){
  //Serial.print(F("[SX1278] Transmitting packet ... "));
  // you can transmit C-string or Arduino string up to 256 characters long
  // NOTE: transmit() is a blocking method!
  //       See example SX127x_Transmit_Interrupt for details
  //       on non-blocking transmission method.
  int state = radio.transmit(Sentence);

  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    //Serial.println(F(" success!"));

    // print measured data rate
    //Serial.print(F("[SX1278] Datarate:\t"));
    //Serial.print(radio.getDataRate());
    //Serial.println(F(" bps"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}