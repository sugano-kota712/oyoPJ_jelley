#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include "config.h"
#include <HUSKYLENS.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PIDLoop.h>
#include <math.h>

char auth[] = "I3ng7mU0BTlTGSoeXFETFmH201n-gvIf";
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "JCOM_MTLU";
char pass[] =  "491696085067";

TinyGPSPlus gps;
HardwareSerial mySerial2(2);
//SoftwareSerial mySerial2(16,17); // RX, TX

HUSKYLENS huskylens;
HardwareSerial mySerial0(0); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);

Servo servo1;
Servo servo2;
Servo servo3;
const int maxUs = 1900;
const int minUs = 1100;
const int servo1Pin = 16; //right
const int servo1Period = 50;
const int servo2Pin = 18; //left
const int servo2Period = 50;
const int servo3Pin = 4; //center
const int servo3Period = 50;
int servo1Us = 1500;
int servo2Us = 1500;
int servo3Us = 1500;

int turnClockAmount = 0;
int forwardAmount = 0;
float turnStrength = 0.5;
float forwardStrength = 0.7;
int manualmode = 0;

const float goal_lat = 35.712722;
const float goal_lng = 139.770067;
float now_lat = 35.712533;
float now_lng = 139.770233;
float past_lat = 35.712533;
float past_lng = 139.770233;
float distance = 30;
double rad = 0;

void myTimerEvent() 
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, millis() / 1000);
}

BLYNK_WRITE(V9)
{
  turnClockAmount = param.asInt(); // V9 = x ranges from -200 to 200
}

BLYNK_WRITE(V8)
{
  forwardAmount = param.asInt(); //V8 = y ranges from -200 to 200, which was controllable(200*0.7=140) 
                                 // >> 100 < PW-1500 < 200
}

BLYNK_WRITE(V10)
{
  manualmode = param.asInt(); //if V10 = 1, manualmode
                                 //  V10 = 0, automode
}

BlynkTimer timer1;
BlynkTimer timer2;

int curve1(int x)
{
  return 1500 + round(turnStrength*x);
}

int curve2(int x)
{
  return 1500 + round(forwardStrength*x);
}

void servoLoop()
{
  servo1Us = curve1(turnClockAmount);
  servo2Us = curve1(turnClockAmount);
  servo3Us = curve2(forwardAmount);
  servo1.writeMicroseconds(servo1Us);
  servo2.writeMicroseconds(servo2Us);
  servo3.writeMicroseconds(servo3Us);
}

float distance_togoal(float now_lng, float now_lat)
{                
   float lng_meter = abs(goal_lng - now_lng) * (91287.7885);
   float lat_meter = abs(goal_lat - now_lat) * (110940.5844);
   return sqrt((lng_meter * lng_meter) + (lat_meter * lat_meter));            
}

double direction_differ(float now_lng, float now_lat,
                        float past_lng, float past_lat)
{
  double lng_togoal = (goal_lng - now_lng) * (91287.7885);
  double lat_togoal = (goal_lat - now_lat) * (110940.5844);
  double lng_frompast = (now_lng - past_lng) * (91287.7885);
  double lat_frompast = (now_lat - past_lat) * (110940.5844);
  double arctan_togoal;
  if(lng_togoal >= 0){
    arctan_togoal = atan(lat_togoal / lng_togoal);
  }else{
    arctan_togoal = atan(lat_togoal / lng_togoal) + M_PI;
  }

  double arctan_frompast;
  if(lng_frompast >= 0){
    arctan_frompast = atan(lat_frompast / lng_frompast);
  }else{
    arctan_frompast = atan(lat_frompast / lng_frompast) + M_PI;
  }
  double rad = arctan_frompast - arctan_togoal;
  if(rad > M_PI){
    rad = rad - (2 * M_PI);
  }else if(rad < (-1 * M_PI) ){
    rad = rad + (2 * M_PI);
  }
  return rad;
}

void setup() {
    Serial.begin(115200);
    mySerial0.begin(9600);
    
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    servo1.setPeriodHertz(servo1Period);
    servo1.attach(servo1Pin, minUs, maxUs);
    
    servo2.setPeriodHertz(servo2Period);
    servo2.attach(servo2Pin, minUs, maxUs);
    
    servo3.setPeriodHertz(servo3Period);
    servo3.attach(servo3Pin, minUs, maxUs);
    delay(3000);
    
    servo1.writeMicroseconds(1500);
    servo2.writeMicroseconds(1500);
    servo3.writeMicroseconds(1500);
    Blynk.begin(auth, ssid, pass);
    
    timer1.setInterval(1000L, myTimerEvent);
    timer2.setInterval(20L, servoLoop);
  
    mySerial2.begin(9600);
    mySerial2.println("Hello, world?");
    mySerial2.println(mySerial2.available());
    while (!huskylens.begin(mySerial0))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}

void loop() {
    //Serial.println(F("dayodayo"));
    //Serial.println(mySerial2.available());
    
    
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    //else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else if(manualmode == 1){
      Blynk.run();
      timer1.run();
      timer2.run();
    }
    else {
        Serial.println(F("###########"));
        if (mySerial2.available() > 0)
        {
          char c = mySerial2.read();
          Serial.print(c);
          gps.encode(c);
          if (gps.location.isUpdated()){
            past_lat = gps.location.lat();
            past_lng = gps.location.lng();
          }
          if(distance > 20){//about distance
            servo1.writeMicroseconds(1500);
            servo2.writeMicroseconds(1500);
            servo3.writeMicroseconds(1800);
            delay(1000);
           }
           while (distance > 1.0) {// about distance
             Serial.print("lat = ");  Serial.println(gps.location.lat(),6);
             Serial.print("lng = "); Serial.println(gps.location.lng(),6);
             now_lat = gps.location.lat();
             now_lng = gps.location.lng();
             distance = distance_togoal(now_lng, now_lat);
             rad = direction_differ(now_lng, now_lat,
                                    past_lng, past_lat);
             servo1.writeMicroseconds(round(1500 + (rad * 100)));// decided by direction
             servo2.writeMicroseconds(round(1500 + (rad * 100)));// decided by direction
             servo3.writeMicroseconds(round(1500 + (distance * 50)));// decided by distance
             delay(1000);
             past_lat = now_lat;
             past_lng = now_lng;
             }
          }

          /* bool IsNot2piRadTurn = true;
          while (!huskylens.available() and IsNot2piRadTurn){
            servo1.writeMicroseconds(round(1500 + 50));
            servo2.writeMicroseconds(round(1500 + 50));
            //360度回転してないならTrueをIsNot2piRadTurnに代入
            }
          if (!IsNot2piRadTurn){
            //Game over
            } */
          int n;
          while (!huskylens.available() and n<=100){ // need to adjust this parameter
            servo1.writeMicroseconds(round(1500 + 50));
            servo2.writeMicroseconds(round(1500 + 50));
            n++;
          }
          while (huskylens.available()){
              HUSKYLENSResult result = huskylens.read();
              // ターゲットが右側にある時
              if(result.xCenter >= 170){
                Serial.println("RIGHT");
                servo1.writeMicroseconds(round(1500 + (result.xCenter-150)*50));
                servo2.writeMicroseconds(round(1500 + (result.xCenter-150)*50));
                if(distance>=5.0) {
                  servo3.writeMicroseconds(round(1500 + (distance * 50)));// decided by distance
                } else {servo3.writeMicroseconds(1600)
                       }
              }
              // ターゲットが左側にある時
              else if(result.xCenter <= 130){
                Serial.println("LEFT");
                servo1.writeMicroseconds(round(1500 - (result.xCenter-150)*50));
                servo2.writeMicroseconds(round(1500 - (result.xCenter-150)*50));
                if(distance>=5.0) {
                  servo3.writeMicroseconds(round(1500 + (distance * 50)));// decided by distance
                } else {servo3.writeMicroseconds(1600)
                       }
              }
              else{
                Serial.println("MIDDLE");
                if(distance>=5.0) {
                  servo3.writeMicroseconds(round(1500 + (distance * 50)));// decided by distance
                } else {servo3.writeMicroseconds(1600)
                       }
              }
          }    
    }
}
/*
void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}
*/
