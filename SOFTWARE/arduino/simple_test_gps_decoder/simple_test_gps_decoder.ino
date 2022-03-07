#include <SoftwareSerial.h>

/* This sample code demonstrates the detection and decoding of a GNGGA message.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on digital pins 3 and 4.

   Tested for GPS module: UBLOX MAX-M8c pico breakout with chip antenna
   Written by Miķelis Putnieks on 06/06/2021, as part of project Nausolaris
*/

SoftwareSerial ss(3, 4);

byte GPSBuffer[82];
byte GPSIndex = 0;
bool newData = false;
unsigned int GPS_Satellites = 0;
unsigned int GPS_Altitude = 0;
String GPS_Lat;
String GPS_Lon;
String GPS_Time;

void setup(){
  Serial.begin(115200);
  ss.begin(9600);
}

void loop(){
  
  
  // -- For one second we parse GPS data and report some key values --
  newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;){
    CheckGPS();
  }

  // -- Was new data received? if yes do we have GNGGA satellite connection? yes? well then lets display the decoded data. --
  if (!newData){  // If no GNGGA message was found
    Serial.println("GPS GNGGA not received! Check wiring connections!");
  }
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
}

void CheckGPS(){  // reads and stores the raw incoming data in a buffer
  int inByte;
  while (ss.available()){
    inByte = ss.read();
    Serial.write(inByte); // uncomment this line if you want to see the raw GPS data flowing
    if ((inByte =='$') || (GPSIndex >= 80)){ GPSIndex = 0; }  //start writing the buffer all over (new line)
    if (inByte != '\r'){ GPSBuffer[GPSIndex++] = inByte; }    //add the data to buffer
    if (inByte == '\n'){ ProcessGPSLine(); GPSIndex = 0; }    //process the last line (see if contains GNGGA message)
  }
}

void ProcessGPSLine(){
  if ((GPSBuffer[1] == 'G') && (GPSBuffer[2] == 'N') && (GPSBuffer[3] == 'G') && (GPSBuffer[4] == 'G') && (GPSBuffer[5] == 'A')){
    Serial.println("GNGGA Detected!");
    //Serial.write(GPSBuffer, 82);  // uncoment if you want to see the GNGGA message being decoded
    ProcessGNGGACommand();
    newData = true;
  }
}

void ProcessGNGGACommand(){
  int i, j, k, IntegerPart;
  unsigned int Altitude = 0;
  bool trimmed = false;
  
  // $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  //                                               =====  <-- altitude in field 8 (example)

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
