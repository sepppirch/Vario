
// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
#define ENABLE_GxEPD2_GFX 0
#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include "Open_Sans_ExtraBold_120.h"

#include <Fonts/FreeMonoBold9pt7b.h>
#include <U8g2_for_Adafruit_GFX.h>
//#include <U8g2lib.h>
#include "GxEPD2_boards_added.h"
//#define MAX_DISPLAY_BUFFER_SIZE 11800 // 
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=D8*/ 7, /*DC=D3*/ 6, /*RST=D4*/ 10, /*BUSY=D2*/ 8)); // GDEH0154D67


uint16_t bg = GxEPD_WHITE;
uint16_t fg = GxEPD_BLACK;

U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;



float multi = 20; //climb bars length
float climbs [64];

float Cx,Cy,Cr;

uint32_t timer = millis();
int i = 0;

const float DEG2RAD = PI / 180.0f;

float climb;
float Alt,lastAlt;
int bearingM;
int bearingG;
int hours;
int minutes;
int timestamp,lastTimestamp; //runtime in 10 th sec
int gSpeed;
int temperature;
int latit,longit;
int latitArr [256];
int longitArr [256];
 
bool haveGyroData = false;
bool haveGpsData = false;

struct gpsFix
{
   float Alt;
   int Bearing;
   int Speed;
   int TimeStamp;
   int latit,longit;
};

typedef struct gpsFix GpsFix;

GpsFix thisGps, lastGps;


void setup()
{

  Serial1.begin(115200);
  Serial.begin(115200);
//  Serial.println();
  Serial.println("setup");
  Serial1.println("?");
  delay(100);
  display.init(115200);
  u8g2Fonts.begin(display); // connect u8g2 procedures to Adafruit GFX
  // cleanup

delay(1000);


  /// create random points
  for (int i = 0; i < 20; i++)
  {
  int b = random(0,359);

  float x = cos(b * DEG2RAD);
  float y = sin(b * DEG2RAD);
  }
}

void loop()
{


while (Serial1.available() > 0) {
  

 byte in = Serial1.read();
 if ( in == '$') {
      
      //GPS MESSAGE
      
      thisGps.Alt = Serial1.parseFloat();
      bearingM = Serial1.parseInt();
      thisGps.Bearing = Serial1.parseInt();
      thisGps.Speed = Serial1.parseInt();
      hours = Serial1.parseInt();
      minutes = Serial1.parseInt();
      thisGps.latit = Serial1.parseInt();
      thisGps.longit = Serial1.parseInt();
      thisGps.TimeStamp = Serial1.parseInt();

     
      Alt = Serial1.parseFloat();
      bearingM = Serial1.parseInt();
      bearingG = Serial1.parseInt();
      gSpeed = Serial1.parseInt();
      hours = Serial1.parseInt();
      minutes = Serial1.parseInt();
      latit = Serial1.parseInt();
      longit = Serial1.parseInt();
      timestamp = Serial1.parseInt();
      
    

    
        if (Serial1.read() == '!') {
        haveGpsData = true;
        //PrintData();
        //
        } 
        
  }
  if ( in == '&') {
      //BARO & MAGNETOM MESSAGE
      
      Alt = Serial1.parseFloat();
      bearingM = Serial1.parseInt();
      temperature = Serial1.parseInt();
      timestamp = Serial1.parseInt();
      
      if (Serial1.read() == '!') {
        haveGyroData = true;
        
        int interval = timestamp - lastTimestamp;  // 100th sec
        climb = (Alt - lastAlt)*100/interval;  // m/sec
        lastAlt = Alt;
        lastTimestamp = timestamp;
  
        Serial.print("lastBaro: ");
        Serial.println(interval);
      } 
      
      
  }

}

if (haveGyroData){
  haveGyroData = false;
  PrintData();
  pushClimb();
  Draw();

}

if (haveGpsData){
  haveGpsData = false;
  pushPos();
  Draw();
}
delay(100);

Serial1.print("?");
Serial.print("?");

}




void Draw()
{



// points for North triangle
 float dist = 98;
 float dist2 = 60;
 float rad = (270 - bearingM) * PI / 180;
 // gps bearing
 float diff =  bearingM - thisGps.Bearing;
 float brad = (thisGps.Bearing - 90 - bearingM) * PI / 180;
  
 float p1xpos = dist * cos(rad)+ 99;
 float p1ypos = dist * sin(rad)+ 99;

 float p2xpos = (dist - 20) * cos(rad + 0.1)+ 99;
 float p2ypos = (dist - 20) * sin(rad + 0.1)+ 99;

  float p3xpos = (dist - 20) * cos(rad - 0.1)+ 99;
 float p3ypos = (dist - 20) * sin(rad - 0.1)+ 99;

 float bp1xpos = dist2 * cos(brad)+ 99;
 float bp1ypos = dist2 * sin(brad)+ 99;

 float bp2xpos = (dist2 - 10) * cos(brad + 0.2)+ 99;
 float bp2ypos = (dist2 - 10) * sin(brad + 0.2)+ 99;

 float bp3xpos = (dist2 - 10) * cos(brad - 0.2)+ 99;
 float bp3ypos = (dist2 - 10) * sin(brad - 0.2)+ 99;

 float bp4xpos = (dist2 - 20) * cos(brad)+ 99;
 float bp4ypos = (dist2 - 20) * sin(brad)+ 99;


 String cBearing = String(bearingM);
 
 String gAlti = String (int(thisGps.Alt));
 String bAlti = String(int(Alt));
 int climb10 = climb*10;
 float rclimb = climb10 / 10;
 String sclimb = String(climb);
 String thismin;
 if (minutes < 10){thismin = "0" + String(minutes);}else{thismin = String(minutes);}

 String gTime = String(hours) + ":" + String(thismin);


  u8g2Fonts.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2Fonts.setFontDirection(0);   
  u8g2Fonts.setFontMode(1);// left to right (this is default)
  u8g2Fonts.setForegroundColor(fg);         // apply Adafruit GFX color
  u8g2Fonts.setBackgroundColor(bg);         // apply Adafruit GFX color
  u8g2Fonts.setFont(u8g2_font_profont17_tf);  // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
//u8g2_font_tenfatguys_tr
//u8g2_font_profont22_tf
//u8g2_font_7Segments_26x42_mn
  display.setRotation(display.epd2.WIDTH > display.epd2.HEIGHT ? 0 : 1);
  //display.setFont(&Open_Sans_ExtraBold_120);
  //display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(0, 0, 200, 200);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    //display.drawRect(bbx, bby, bbw, bbh, GxEPD_BLACK);
    u8g2Fonts.setCursor(0, 11);  
    u8g2Fonts.print(bAlti);
    u8g2Fonts.setFont(u8g2_font_profont12_tf);
     u8g2Fonts.print("m");
    u8g2Fonts.setFont(u8g2_font_profont17_tf);
    u8g2Fonts.setCursor(0, 26);  
    u8g2Fonts.print(temperature);
    u8g2Fonts.print("°");
    u8g2Fonts.setCursor(90, 26);
    u8g2Fonts.print(cBearing + "°");
   // u8g2Fonts.setFontMode(1);
   // u8g2Fonts.setCursor(bp4xpos, bp4ypos);  
    
   //u8g2Fonts.print(gBearing);
   
    u8g2Fonts.setCursor(155, 11);  
    u8g2Fonts.print(gTime);
    //u8g2Fonts.setFont(u8g2_font_open_iconic_play_4x_t);
    if (climb < 0){
      u8g2Fonts.setCursor(161, 175); 
    }else{
     u8g2Fonts.setCursor(170, 175);  
    }
     
    u8g2Fonts.print(climb, 1);
    
    // courB24_tf  font_inr24_mf maniac_tf freedoomr25_mn profont17_tf
    if (thisGps.Speed < 9){
    u8g2Fonts.setCursor(150, 200);
    }else{u8g2Fonts.setCursor(140, 200);} 
    u8g2Fonts.setFont(u8g2_font_profont12_tf);
    u8g2Fonts.print("km/h "); 
    u8g2Fonts.setFont(u8g2_font_profont29_tf);
    u8g2Fonts.print(thisGps.Speed);
    u8g2Fonts.setFont(u8g2_font_profont17_tf);


    u8g2Fonts.setCursor(90, 110);  
    u8g2Fonts.print(thisGps.Bearing);
    u8g2Fonts.print("°");
    u8g2Fonts.setCursor(0, 200);  
    u8g2Fonts.print("7.4");

    
    

    display.drawCircle(99, 99,99,  GxEPD_BLACK);
    //display.drawCircle(xpos, ypos,4,  GxEPD_BLACK);

    display.fillTriangle(p1xpos,p1ypos,p2xpos,p2ypos,p3xpos,p3ypos, GxEPD_BLACK);

    display.drawTriangle(99,1,90,10,108,10, GxEPD_BLACK);
   // display.drawTriangle(bp1xpos,bp1ypos,bp2xpos,bp2ypos,bp3xpos,bp3ypos, GxEPD_BLACK);
    
   display.drawLine(bp1xpos,bp1ypos,99,99, GxEPD_BLACK);
   display.drawLine(bp1xpos,bp1ypos,bp3xpos,bp3ypos, GxEPD_BLACK);
   display.drawLine(bp2xpos,bp2ypos,bp1xpos,bp1ypos, GxEPD_BLACK);

// wind triangle
   display.drawTriangle(20, 100, 80, 30 , 125, 150, GxEPD_BLACK);
   findCircle(20, 100, 80, 30 , 125, 150);
   display.drawCircle(int(Cx), int(Cy), int(Cr),  GxEPD_BLACK);
   display.drawPixel(int(Cx), int(Cy), GxEPD_BLACK);
   
   drawBars();
   drawRoute();
   
    //display.print(str);
 }
  while (display.nextPage());
  
  //Serial.println("showMinutes done");
}


void pushClimb(){

    for (int i = 60; i >= 0; i--) {
      if (i == 0) {
        //climbs[i] = (climb + climbs[1])/2;
        climbs[i] = climb;
      } else {
        climbs[i] = climbs[i - 1];
      }
    }

}

void pushPos(){

    for (int i = 250; i >= 0; i--) {
      if (i == 0) {
        latitArr[i] = thisGps.latit;
        longitArr[i] = thisGps.longit;
      } else {
        latitArr[i] = latitArr[i - 1];
        longitArr[i] = longitArr[i -1];
      }
    }

}


void drawRoute(){
  //u8g2.drawBox(64, 2, 20,2);
   

  for (int i = 0; i < 250; i++){
    //u8g2.drawLine(ybase, i * 2, ybase - int(climbs[i] * multi), i * 2);
    display.drawLine( (latitArr[i] - thisGps.latit + 100),(longitArr[i] - thisGps.longit + 100),  (latitArr[i + 1] - thisGps.latit + 100), (longitArr[i +1] - thisGps.longit + 100), GxEPD_BLACK);
  }

}

void drawBars(){
  //u8g2.drawBox(64, 2, 20,2);
  int ybase = 150;

  for (int i = 0; i < 60; i++){
    //u8g2.drawLine(ybase, i * 2, ybase - int(climbs[i] * multi), i * 2);
    display.drawLine( i * 3,ybase,  i * 3, ybase - int(climbs[60 - i] * multi), GxEPD_BLACK);
  }

}


void PrintData(){
  

   Serial.print ("climb = ");
   Serial.println (climb);  
   Serial.print ("alt = ");
   Serial.println (Alt);  
   Serial.print ("bearingM = ");
   Serial.println (bearingM);  
   Serial.print ("bearingG = ");
   Serial.println (thisGps.Bearing);  
   String tTime = String(hours) + ":" + String(minutes);
   Serial.println (tTime);
   Serial.print ("speed = ");
   Serial.println (thisGps.Speed); 
   Serial.print ("lat = ");
   Serial.println (thisGps.latit); 
   Serial.print ("ong = ");
   Serial.println (thisGps.longit);
   Serial.print ("time = ");
   Serial.println (timestamp);  
   
     
 
}







void full_white()
{
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  }
  while (display.nextPage());
}

void full_black()
{
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_BLACK);
  }
  while (display.nextPage());
}










/// circle from 3 points >> https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/

void findCircle(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float x12 = x1 - x2;
    float x13 = x1 - x3;
  
    float y12 = y1 - y2;
    float y13 = y1 - y3;
  
    float y31 = y3 - y1;
    float y21 = y2 - y1;
  
    float x31 = x3 - x1;
    float x21 = x2 - x1;
  
    // x1^2 - x3^2
    float sx13 = pow(x1, 2) - pow(x3, 2);
  
    // y1^2 - y3^2
    float sy13 = pow(y1, 2) - pow(y3, 2);
  
    float sx21 = pow(x2, 2) - pow(x1, 2);
    float sy21 = pow(y2, 2) - pow(y1, 2);
  
    float f = ((sx13) * (x12)
             + (sy13) * (x12)
             + (sx21) * (x13)
             + (sy21) * (x13))
            / (2 * ((y31) * (x12) - (y21) * (x13)));
    float g = ((sx13) * (y12)
             + (sy13) * (y12)
             + (sx21) * (y13)
             + (sy21) * (y13))
            / (2 * ((x31) * (y12) - (x21) * (y13)));
  
    float c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1;
  
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    // where centre is (h = -g, k = -f) and radius r
    // as r^2 = h^2 + k^2 - c
    float h = -g;
    float k = -f;
    float sqr_of_r = h * h + k * k - c;
  
    // r is the radius
    float r = sqrt(sqr_of_r);

    Cx = h;
    Cy = k;
    Cr = r;
   // cout << "Centre = (" << h << ", " << k << ")" << endl;
   // cout << "Radius = " << r;
}
