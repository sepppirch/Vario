// Display Library example for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2
#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Math.h>
#include <BME280I2C.h> //from BME280 by Tyler Glenn
#include <TinyGPS++.h>
#define GPSSerial Serial1
#include "BlueDot_BME280.h" 
#include <QMC5883LCompass.h>
// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
#include <Arduino.h>
#include "wiring_private.h"
 
Uart mySerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
 
// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()
{
mySerial.IrqHandler();
}
int i = 0;

float climb = -1.5;
 
int bearingM = 125;
int bearingG = 133;
int hours = 12;
int minutes = 55;
int gSpeed = 39;


const byte speakerPin=14;
unsigned long lastPeriodStart;
int onDuration;
int periodDuration = 1000;
int count = 0;
int sink = 0;
float offZ = 0.5;

QMC5883LCompass compass;
// need to add tilt compensation!!!
// have a look at this: https://www.best-microcontroller-projects.com/magnetometer-tilt-compensation.html
// https://blog.arduino.cc/2019/05/15/this-compass-reads-the-correct-heading-even-when-tilted/
//https://www.instructables.com/Quaternion-Compass/
//https://duino4projects.com/tilt-compensated-compass/
int calibrationData[3][2];
bool changed = false;
bool compassCalibDone = true;
int t = 0;
int c = 0;
int CompassBearing,lastCompassBearing;
float MagDeclination = 4.5;
uint32_t total_seconds = 0;

//String gSpeed = "NOGPS";
//String gBearing = "NOGPS";
float roll, pitch, heading;
Madgwick filter;
const float sensorRate = 104.00;
TinyGPSPlus gps;
int GPSBaud = 9600;
BlueDot_BME280 bme1;                                     //Object for Sensor 1
int lastSecond = 0;                               
int bme1Detected = 0;  
int temp = 0;
float lastAlt, Alt;
//float climb;
float multi = 20;
float climbs [64];
float gForce = 1;

const float DEG2RAD = PI / 180.0f;

float gpsBearing;
float  gpsSpeed;
int gpsHour, gpsMin,gpsAlt;

int latit = 4822144;
int longit = 1637766;
int gpsLastAge = 0;
bool gpsNewFix = false;

String gpsTime = "xxx";
String GPS_Speed = "xxx";

unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
long interval ;  

const int buttonPin = 21; 
int buttonState = 0;
const int buttonPin2 = 20; 
int button2State = 0;
int button2Count = 0;
int button2LastState = 0;


void setup()
{

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  //attachInterrupt(0, pin_ISR, CHANGE);
  
  Serial1.begin(GPSBaud);
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  //Wire.begin();
  // Reassign pins 5 and 6 to SERCOM alt
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
 
// Start my new hardware serial
  mySerial.begin(115200);
  delay(100);
  
  if(!IMU.begin())  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  filter.begin(sensorRate);
  compass.init();
  compass.setCalibration(-1355, 1540, -1697, 1135, -1363, 1131);



  bme1.parameter.communication = 0;
  bme1.parameter.I2CAddress = 0x76;
  bme1.parameter.sensorMode = 0b11;
  bme1.parameter.IIRfilter = 0b100;
  bme1.parameter.humidOversampling = 0b101;
  bme1.parameter.tempOversampling = 0b101;
  bme1.parameter.pressOversampling = 0b010;
  bme1.parameter.pressureSeaLevel = 1013.25;
  bme1.parameter.tempOutsideCelsius = 15;               //default value of 15°C
  bme1.parameter.tempOutsideFahrenheit = 59;

  
  if (bme1.init() != 0x60)
  {    
    Serial.println(F("Ops! First BME280 Sensor not found!"));
    bme1Detected = 0;
  }

  else
  {
    Serial.println(F("BME280 Sensor detected!"));
    bme1Detected = 1;
  }


  if ((bme1Detected == 0))
  {
    Serial.println("Barometer not detected" );

    while(1);
  }

//GPS.begin(9600);


//delay(1000);
}

void loop()
{
  
  while (mySerial.available() > 0) {
     if (mySerial.read() == '?') {
      
        delay(10);
        unsigned long currentMillis = millis();
        interval = currentMillis - previousMillis;
        previousMillis = currentMillis;
        //Serial.println(interval);
        Serial.println(button2Count);

        i++;
        i = i % 360;
        Alt = bme1.readAltitudeMeter();
        temp = bme1.readTempC();
        //Serial.println(gForce);
        climb = (Alt - lastAlt)*1000/interval;
        lastAlt = Alt;

        
        //latit = latit + cos(float(CompassBearing) * DEG2RAD) * 6;
        //longit = longit + sin(float(CompassBearing) * DEG2RAD) * 6;

        String sentence;
 
        if (gpsNewFix)
        {
        sentence = "$"+ String(climb) + " " + String(int(Alt))+ " " + String(CompassBearing) + " " + String(int(gpsBearing)) + " " + String(int(gpsSpeed)) + " " + String(gpsHour) + " " + String(gpsMin) + " " + String(int(temp)) + " " + String(int(latit))+ " " + String(int(longit)) + "!";
        }
        else
        {
        sentence = "&"+ String(climb) + " " + String(CompassBearing)  + "!";
        }
         //String sentece = "$"+ String(climb) + " " + String(int(Alt))+ " " + String(CompassBearing) + " " + String(i % 360) + " " + String(int(gps.speed.kmph())) + " " + String(gps.time.hour()) + " " + String(gps.time.minute()) + " " + String(int(temp)) +"!";
        Serial.println(sentence);
        
        
        //String sentece = "$0.45 2001 " + String(i) + " 213 14 57 36!";
        mySerial.println(sentence);

        displayGPSInfo(true);
        

     }
  }
  
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
     

  
  //unsigned long currentMillis = millis();

  /*
   * 
  if (Serial.available() > 0) {
    // get incoming byte:
    char inByte = Serial.read();
    if (inByte == 'c'){

      compassCalibDone = false;
      Serial.println("COMPASS CALIBRATION STARTED");
    }
    
  Serial.println(inByte);
}else{
*/

 // total_seconds = total_seconds + 10  ;
 
  //delay(50);
  //Serial.println( Alt);
  readCompassModule(false);
  readButtons();
  //play(climb);
  

 // }
}
void play (float in){
  
      if (millis()-lastPeriodStart>=periodDuration)
      {
      lastPeriodStart+=periodDuration;
      
      if (climb > 0) /*PLAY Climb*/
        {

          float dur = 2000 / ((climb + 1) * 2.0);
          if (dur > 20)
          {
            periodDuration = dur;
          }else
          {periodDuration = 20;}
          
        onDuration = periodDuration / 10;
 
        float freq = (climb + 1) * 70 + 90;
        Serial.println(String(climb) + " " + String(freq));
 
        tone(speakerPin,freq, onDuration + 20 );
        }
        else if(climb > -0.4 && climb < 0 ){ /*thermal sniffer*/
          
          periodDuration = 800 ;
          tone(speakerPin,260, 30 );
          
          }
        else if (climb < -2.5)
        { /*PLAY SINKTONE*/
          periodDuration = 700 ;

          sink++;
          sink = sink % 2;
          if(sink == 0){
             tone(speakerPin,70, 200 );
            }else{
             tone(speakerPin,150,200 );
            }

        }
  
     }
  
  }

void readButtons(){
  buttonState = digitalRead(buttonPin);
  button2State = digitalRead(buttonPin2);

  if(button2State == 0 && button2State != button2LastState){
    button2Count++;
    
  }
  button2LastState = button2State;
  String buttons = String(buttonState) + " " + String(button2State);
        
  //Serial.println(buttons); 
}


void calibrateCompass(){
  
 int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  changed = false;

  if(x < calibrationData[0][0]) {
    calibrationData[0][0] = x;
    changed = true;
  }
  if(x > calibrationData[0][1]) {
    calibrationData[0][1] = x;
    changed = true;
  }

  if(y < calibrationData[1][0]) {
    calibrationData[1][0] = y;
    changed = true;
  }
  if(y > calibrationData[1][1]) {
    calibrationData[1][1] = y;
    changed = true;
  }

  if(z < calibrationData[2][0]) {
    calibrationData[2][0] = z;
    changed = true;
  }
  if(z > calibrationData[2][1]) {
    calibrationData[2][1] = z;
    changed = true;
  }

  if (changed && !compassCalibDone) {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    c = millis();
  }
    t = millis();
 
  if ( (t - c > 5000) && !compassCalibDone) {
    compassCalibDone = true;
    int xmin = calibrationData[0][0];
    int xmax = calibrationData[0][1];
    int ymin = calibrationData[1][0];
    int ymax = calibrationData[1][1];
    int zmin = calibrationData[2][0];
    int zmax = calibrationData[2][1];
    
    compass.setCalibration(xmin, xmax, ymin, ymax, zmin, zmax);
    Serial.println("CALIBRATION COMPLETE.");
    
    calibrationData[0][0] = 0;
    calibrationData[0][1] = 0;
    calibrationData[1][0] = 0;
    calibrationData[1][1] = 0;
    calibrationData[2][0] = 0;
    calibrationData[2][1] = 0;
    t = 0;
    c = 0;
    
    }
}


void readCompassModule(bool logging){

    if(!compassCalibDone){
    calibrateCompass();
    }else{
        float xAcc, yAcc, zAcc;
        float xGyro, yGyro, zGyro;
      
      
        
        
        if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
          IMU.readAcceleration(xAcc, yAcc, zAcc);
          gForce = zAcc;
          IMU.readGyroscope(xGyro, yGyro, zGyro); 
          filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
          roll = filter.getPitch() * DEG_TO_RAD * -1;
          pitch = filter.getRoll()* DEG_TO_RAD ;
          
          //float pitchFiltered = 0.1 * pitch + 0.9 * pitchFilteredOld; // low pass filter
          
          //pitchFilteredOld = pitchFiltered;
        }
      
        int x, y, z;
        
        // Read compass values
        compass.read();
      
        // Return XYZ readings
        x = compass.getX();
        y = compass.getY();
        z = compass.getZ();
      
        float Xhorizontal = x* cos(pitch) + y * sin(roll) * sin(pitch) - z * cos(roll) * sin(pitch); 
        
      
        
        float Yhorizontal = y*cos(roll) + z*sin(roll);
        float azimut = atan2(Yhorizontal,Xhorizontal) * RAD_TO_DEG;
        
        //add 180 degree to get only positive values
      
        CompassBearing = (azimut + 180);

        if (logging){
          Serial.println(String(CompassBearing)); //"pitch: " + String(pitch) + "   Roll: " + String(roll) + "  Bearing: " + 
          }
        
      
          
    }
  delay(10);
}




 





  
void displayGPSInfo(bool logging)
{
  if (gps.location.isValid())
  { 

    int thisAge = gps.location.age();
    
    if (thisAge < gpsLastAge)
    {
      
      double tempLat = gps.location.lat();
      latit = tempLat * 100000;
      double tempLon = gps.location.lng();
      longit = tempLon * 100000;
      gpsAlt = gps.altitude.meters();
      gpsSpeed = gps.speed.kmph();
      gpsBearing = gps.course.deg();
      gpsLastAge = thisAge;
      gpsNewFix = true;
      
    }else{gpsNewFix = false;}
     



    if(logging){
    Serial.print("Latitude: ");
    Serial.println(latit);
    Serial.print("Longitude: ");
    Serial.println(longit);
    Serial.print("Altitude: ");
    Serial.println(gpsAlt);
    Serial.print("Speed: ");
    Serial.println(gpsSpeed);
    Serial.print("Bearing: ");
    Serial.println(gpsBearing);
    Serial.print("Sat: ");
    Serial.println(gps.satellites.value());
    Serial.print("age: ");
    Serial.println(gps.location.age());
    Serial.print("quality: ");
    Serial.println(gps.hdop.value());
    //
    }
    
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {

    if(logging){
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.println(gps.date.year());
    }
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid() )
  { 

    gpsHour = gps.time.hour();
    gpsMin = gps.time.minute();
    
    gpsTime = String(gpsHour) + ":" + String(gpsMin) + ":" + String(gps.time.second());
    if(logging){
      Serial.println(gpsTime);
    }
  }
  else
  {
    Serial.println("Not Available");
  }
 
  //delay(1000);
}
  
