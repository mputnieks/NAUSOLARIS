#include <Arduino.h>
#include <NMEAGPS.h>

//======================================================================
// High Altitude Balloon Data Logger
// Written by Aaron Price
//Version 3.0
//Description:  This program sets the GPS in flight mode then collects 
//latitude, longitude, time, satellites locked, altitude, and wind speed.
//Update allows the collection of temperature data using TMP analog sensor on pin 0.
//It prints this data to the serial port or SD Card every 30 seconds.
//Either the onboard LED or the LED on D13 will blink every time data is recorded onto the SD card.
//Sketch uses NEOGPS library and is based off of the example sketch NMEAloc. 
//Learn how to build the system here: https://www.instructables.com/id/The-Easiest-Arduino-High-Altitude-Balloon-Data-Log/
//======================================================================

#if defined( UBRR1H ) | defined( ID_USART0 )
  // Default is to use Serial1 when available.  You could also
  // use NeoHWSerial, especially if you want to handle GPS characters
  // in an Interrupt Service Routine.
  //#include <NeoHWSerial.h>
#else  
  // Only one serial port is available
  #include <SoftwareSerial.h> // uses software serial
#endif
#include "GPSport.h"

#ifdef NeoHWSerial_h
  #define Serial NeoSerial
#else
  #define Serial Serial
#endif



//------------------------------------------------------------
// Check that the config files are set up properly

#if !defined( NMEAGPS_PARSE_RMC )
  #error You must uncomment NMEAGPS_PARSE_RMC in NMEAGPS_cfg.h!
#endif

#if !defined( GPS_FIX_TIME )
  #error You must uncomment GPS_FIX_TIME in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SPEED )
  #error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined( GPS_FIX_SATELLITES )
  #error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#ifndef GPS_FIX_ALTITUDE
  #error GPS_FIX_ALTITUDE must be defined in GPSfix_cfg.h!
#endif

//------------------------------------------------------------

static NMEAGPS  gps; // This parses the GPS characters

const int temperaturePin = 5;

int i = 0;

byte gps_set_sucess = 0 ;

float voltage, degreesC, degreesF;

void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gps_port.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  gps_port.println();
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (gps_port.available()) {
      b = gps_port.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
//----------------------------------------------------------------

static void printL( Print & outs, int32_t degE7 );
static void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg*10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)){
    outs.print( '0' );
    factor /= 10L;
  }
  
  // Print fractional degrees
  outs.print( degE7 );
}


static void doSomeWork();
static void doSomeWork( const gps_fix & fix )
{
  //  This is the best place to do your time-consuming work, right after
  //     the RMC sentence was received.  If you do anything in "loop()",
  //     you could cause GPS characters to be lost, and you will not
  //     get a good lat/lon.
  //  For this example, we just print the lat/lon.  If you print too much,
  //     this routine will not get back to "loop()" in time to process
  //the next set of GPS data.

// test vs other option
  //float voltage, degreesC, degreesF;
  //voltage = getVoltage(temperaturePin);
  //degreesC = (voltage - 0.5) * 100.0;
  //degreesF = degreesC * (9.0/5.0) + 32.0;
  //Serial.println(degreesF);

 if (i == 30) { // keeps a reading every 30 seconds. Change this to the time wanted between readings

 //concern tha this takes time therefore could throw off repetition
  // consistency
//  float voltage, degreesC, degreesF;
  voltage = getVoltage(temperaturePin);
  degreesC = (voltage - 0.5) * 100.0;
  degreesF = degreesC * (9.0/5.0) + 32.0;
  // Serial.print("Degrees C "); //uncomment if you want celsius
  // Serial.println(degreesC); //uncomment if you want celsius
  Serial.print("Degrees F "); //comment if you do not want fahrenheit
  Serial.println(degreesF); //comment if you do not want fahrenheit

  digitalWrite(LED_BUILTIN, HIGH);
  if (fix.valid.time) {
    // Set these values to the offset of your timezone from GMT
    static const int32_t         zone_hours   = -7L; // PST
    static const int32_t         zone_minutes =  0L; // usually zero
    static const NeoGPS::clock_t zone_offset  =
                      zone_hours   * NeoGPS::SECONDS_PER_HOUR +
                      zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

    NeoGPS::time_t localTime( fix.dateTime + zone_offset );

    Serial << localTime;

  }


  if (fix.valid.location) {

    
    if ( fix.dateTime.seconds < 10 )
      Serial.print( '0' );
    Serial.print( fix.dateTime.seconds );
    Serial.print( ',' );
    
    

    // Serial.print( fix.latitude(), 6 ); // floating-point display
    // Serial.print( fix.latitudeL() ); // integer display
    printL( Serial, fix.latitudeL() ); // prints int like a float
    Serial.print( ',' );
    // Serial.print( fix.longitude(), 6 ); // floating-point display
    // Serial.print( fix.longitudeL() );  // integer display
    printL( Serial, fix.longitudeL() ); // prints int like a float

    Serial.print( ',' );
    if (fix.valid.satellites)
      Serial.print( fix.satellites );
     

    Serial.print( " , " );
    Serial.print( fix.speed(), 6 );
    Serial.print( F(" kn = ") );
    Serial.print( fix.speed_mph(), 6 );
    Serial.print( F(" mph") );
    Serial.print( "      " );
    Serial.print( gps.fix().altitude_cm());
  
    
    
  } else {
    // No valid location data yet!
    Serial.print( '?' );
  }

  Serial.println();
  digitalWrite(LED_BUILTIN, LOW);

 i = 0;

 }else{

  i = i + 1;
  
  }// doSomeWork
} 

float getVoltage(int pin)
{
 return (analogRead(pin) * 0.004882814);
}
//------------------------------------

static void GPSloop();
static void GPSloop()
{
  while (gps.available( gps_port ))
    doSomeWork( gps.read() );

} // GPSloop
  
//--------------------------

void setup()
{
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

   // Start the UART for the GPS device
  gps_port.begin(9600);

Serial.println("Setting uBlox flight mode: "); 
 uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
  
  while (!Serial);
  
  Serial.print( F("NMEAloc.INO: started\n") );
  Serial.print( F("fix object size = ") );
  Serial.println( sizeof(gps.fix()) );
  Serial.print( F("NMEAGPS object size = ") );
  Serial.println( sizeof(gps) );
  Serial.println( F("Looking for GPS device on " USING_GPS_PORT) );
  Serial.print("High Altitude Weather Balloon Data Logger");
  Serial.println(" by Aaron Price");
  Serial.println();
  Serial.println(" Time               Latitude   Longitude     SAT   Wind Speed     Wind Speed    Altitude");      
  Serial.println("                      (deg)       (deg)              knotts          mph           cm");
  Serial.println("----------------------------------------------------------------------------------------------------------------");

  #ifdef NMEAGPS_NO_MERGING
    Serial.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif

  Serial.flush();

  
 
}

//--------------------------

void loop()
{

  GPSloop();

  // If the GPS has been sending data, then the "fix" structure may have
  //   valid data.  Remember, you must check the valid flags before you
  //   use any of the data inside "fix".  See "doSomeWork" for an example
  //   of checking whether any lat/lon data has been received yet.
}
