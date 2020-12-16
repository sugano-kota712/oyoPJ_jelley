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

//morohoshi char auth[] = "RQ8i6EvviHUvFPpx9yjvJN_0Y1286rgR";
char auth[] = "I3ng7mU0BTlTGSoeXFETFmH201n-gvIf";
// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "JCOM_MTLU";
//char pass[] =  "491696085067";

char ssid[] = "iphone@moon";
char pass[] =  "8fzsp93s04222";


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
const int servo1Pin = 15; //right
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
int counter2 = 0;

const float goal_lat = 35.712722;
const float goal_lng = 139.770067;

//const float goal_lat = 35.296763;
//const float goal_lng = 139.575072;
//諸星の家

float now_lat = 35.712533;
float now_lng = 139.770233;
float past_lat = 35.712533;
float past_lng = 139.770233;
float distance = 30;
double rad = 0;

int flag = 0;
int husky_search_flag = 0;
int wait_next_husky_flag = 0;

unsigned long millis_Hs_previous = 0;
unsigned long millis_Hs_current = 0;  
unsigned long wait_next_husky = 0;

int pre_manualmode = 0;
unsigned long pre_distance_ikichi = 10.0;
unsigned long pre_rotate_direction = 1;
unsigned long pre_rotate_amount = 1;
unsigned long pre_duration_gps = 1000;
unsigned long pre_duration_Hs = 20000; //need to adjust
unsigned long pre_duration_wait_next_husky = 3000;

bool manualmode = true;
unsigned long distance_ikichi = 10.0;
unsigned long rotate_amount = 1;
unsigned long duration_gps = 1000;
unsigned long duration_Hs = 20000; //need to adjust
unsigned long duration_wait_next_husky = 3000;


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
  pre_manualmode = param.asInt(); //if V10 = 1, manualmode
                                 //  V10 = 0, automode                              
}

BLYNK_WRITE(V11)
{
  pre_distance_ikichi = param.asInt();                       
}

BLYNK_WRITE(V12)
{
  pre_rotate_direction = param.asInt();                         
}

BLYNK_WRITE(V13)
{
  pre_rotate_amount =  param.asInt();                          
}

BLYNK_WRITE(V14)
{
  pre_duration_Hs = param.asInt();                             
}

BLYNK_WRITE(V15)
{
  pre_duration_gps = param.asInt();                             
}

BLYNK_WRITE(V16)
{
  pre_duration_wait_next_husky = param.asInt();                              
}

void setBlynkDependency(){
  manualmode = (pre_manualmode == 1);
  distance_ikichi = pre_distance_ikichi;
  rotate_amount = pre_rotate_direction * pre_rotate_amount;
  duration_gps = pre_duration_gps * 100;
  duration_Hs = pre_duration_Hs * 100; //need to adjust
  duration_wait_next_husky = pre_duration_wait_next_husky * 100;
  Blynk.virtualWrite(V17, rad);
  
}


BlynkTimer timer1;
BlynkTimer timer2;
BlynkTimer timer3;


int curve1(int x)
{
  return 1500 - rotate_amount * round(turnStrength*x);
}

int curve2(int x)
{
  return 1500 + rotate_amount * round(forwardStrength*x);
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

void GPSmode() {
  /*char c = mySerial2.read();
  Serial.print(c);
  Serial.print(c);
  
  gps.encode(c);
  if (gps.location.isUpdated()){
    past_lat = gps.location.lat();
    past_lng = gps.location.lng();
  }*/
  //Serial.print("GPSmode");
  unsigned long millis_gps_previous = millis();
  unsigned long millis_gps_current = millis();
  //Serial.print("millis_gps_current="); Serial.println(millis_gps_current, 6);
    
  while (mySerial2.available() > 0){
    char c = mySerial2.read();
    //Serial.print(c);
    gps.encode(c);
    
    millis_gps_current = millis(); 
    if (gps.location.isUpdated() and (millis_gps_current - millis_gps_previous) >= duration_gps){
      //Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
      //Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      //Serial.print("ALT="); Serial.println(gps.altitude.meters());

      if(flag == 0){//about distance
        servo1.writeMicroseconds(1500);
        servo2.writeMicroseconds(1500);
        servo3.writeMicroseconds(1800);
        flag = 1;
      }else {// about distance
        //Serial.print("lat2 = ");  Serial.println(gps.location.lat(),6);
        //Serial.print("lng2 = "); Serial.println(gps.location.lng(),6);
        now_lat = gps.location.lat();
        now_lng = gps.location.lng();
        distance = distance_togoal(now_lng, now_lat);
        rad = direction_differ(now_lng, now_lat, past_lng, past_lat);
        //Serial.print("rad = "); Serial.println(rad,6);
        //Serial.println(distance,6);
        if (isnan(rad)){
          rad = 0;
          counter2 ++ ;
          }
        else{
          Serial.print("何回同じ座標を観測したのか： ");Serial.println(counter2,6);
          Serial.println("------------------------------------");
          Serial.println("------------------------------------");
          Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
          Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
          Serial.print("rad = "); Serial.println(rad,6);
          Serial.print("distance = ");Serial.println(distance,6);
          counter2 = 0;
        }
        servo1.writeMicroseconds(round(1500 - rotate_amount * rad * 2));// in most cases -1 < rad < 1 
        servo2.writeMicroseconds(round(1500 - rotate_amount * rad * 2));// so recommended to double
        if (distance > 10){
          servo3.writeMicroseconds(round(1500 + 250));// decided by distance
        }else{
         servo3.writeMicroseconds(round(1500 + (distance * 20 + 50)));// decided by distance
        }
      }
      millis_gps_previous = millis();
      past_lat = now_lat;
      past_lng = now_lng;
    }else{
      if (distance > 10){
        servo3.writeMicroseconds(round(1500 + 250));// decided by distance
      }else{
       servo3.writeMicroseconds(round(1500 + (distance * 20 + 50)));// decided by distance
      }
    }
  }
}

void HUSKYsearch(){
  
  if (!huskylens.available()) {//and (millis_Hs_current - millis_Hs_previous) < duration_Hs){
    
    if (husky_search_flag == 0){
      millis_Hs_previous = millis();
      husky_search_flag = 1;
    }
    Serial.print("(millis_Hs_current - millis_Hs_previous) ="); Serial.println((millis_Hs_current - millis_Hs_previous), 6);
    millis_Hs_current = millis();
    servo1.writeMicroseconds(round(1500 + 100));
    servo2.writeMicroseconds(round(1500 + 100)); // fix this figure so that the duration will be independently decided
  }
}

void HUSKYmode(){
  HUSKYLENSResult result = huskylens.read();
  // ターゲットが右側にある時
  if(result.xCenter >= 170){
    Serial.println("RIGHT");
    servo1.writeMicroseconds(round(1500 - pre_rotate_direction * (result.xCenter-150)*5));
    servo2.writeMicroseconds(round(1500 - pre_rotate_direction * (result.xCenter-150)*5));
    servo3.writeMicroseconds(1600);
  }
  // ターゲットが左側にある時
  else if(result.xCenter <= 130){
  
    Serial.println("LEFT");
    servo1.writeMicroseconds(round(1500 - pre_rotate_direction * (result.xCenter-150)*5));
    servo2.writeMicroseconds(round(1500 - pre_rotate_direction * (result.xCenter-150)*5));
    servo3.writeMicroseconds(1600);
  }
  else{
    Serial.println("MIDDLE");
    servo3.writeMicroseconds(1600);
  }
}

void setup() {
    Serial.begin(9600);
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
    timer3.setInterval(20L, setBlynkDependency); //TODO 要確認
    
  
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
//(false ){//
void loop() {
  Blynk.run();
  if (manualmode) { 
    timer1.run();
    timer2.run();
    Serial.println(F("dayodayo2"));
  }else if (distance > distance_ikichi){
    GPSmode();
  }
  else if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()){
    Serial.println(F("No block or arrow appears on the screen!"));
    if (husky_search_flag == 1 and  (millis_Hs_current - millis_Hs_previous) > duration_Hs){
      if(wait_next_husky_flag == 0){
        wait_next_husky = millis();
        wait_next_husky_flag = 1;
      }
      GPSmode();
      if (millis() - wait_next_husky > duration_wait_next_husky){
          husky_search_flag == 0;
          wait_next_husky_flag = 0;
      }
    }
    else{
      HUSKYsearch();
      }
    
  }else{
    while (huskylens.available()){
        HUSKYmode();
    }//else{
     // GPSmode();
    //}
  }
  timer3.run();
  //Serial.println(int_manualmode);
      
    
} 
