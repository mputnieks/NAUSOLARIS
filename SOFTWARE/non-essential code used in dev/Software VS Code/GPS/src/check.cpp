/*******************************************************************************************************
  Program Operation - This is a simple program to test a GPS. It reads characters from the GPS using
  software serial and sends them (echoes) to the IDE serial monitor. If your ever having problems with
  a GPS (or just think you are) use this program first.

  If you get no data displayed on the serial monitor, the most likely cause is that you have the receive
  data pin into the Arduino (RX) pin connected incorrectly.

  GPS baud rate set at 9600 baud, Serial monitor set at 115200 baud. If the data displayed on the serial
  terminal appears to be random text with odd symbols its very likely you have the GPS serial baud rate
  set incorrectly for the GPS.

  Note that not all pins on all Arduinos will work with software serial, see here;

  https://www.arduino.cc/en/Reference/softwareSerial

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// software serial port 1 for gps: RX = D3, TX = D4
SoftwareSerial GPS(3, 4);

void setup(){
  Serial.begin(9600);
  while (!Serial){
    ; // wait for hordware serial port to connect
  }

  GPS.begin(9600);
  while (!GPS){
    ; // wait for software serial gps port to connect
  }
  Serial.println("GPS_Echo Starting");
}

void loop(){
  while (GPS.available()){
    Serial.write(GPS.read());
  }
}