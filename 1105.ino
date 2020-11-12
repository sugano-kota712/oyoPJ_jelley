#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include "config.h"


char auth[] = "I3ng7mU0BTlTGSoeXFETFmH201n-gvIf";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "iphone@moon";
char pass[] =  "8fzsp93s04222";

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
float forwardStrength = -0.7;
int turnleft = 0;
int turnright = 0;




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
  turnleft = param.asInt(); // on:1, off:0
}

BLYNK_WRITE(V11)
{
  turnright = param.asInt(); // on:1, off:0
}

BLYNK_WRITE(V12)
{
  forwardStrength =  (float)param.asInt() )/100; //vertical bar ranges from 0 to 200
}

BlynkTimer timer1;
BlynkTimer timer2;

int curve1(int x, float forwardStrength, int turnleft, int turnright)
{
  if(turnleft == 1){
    return 1650;
  } else if(turnright == 1){
    return 1350;
  } else if (forwardStrength > 1.0){
    return 1500 - round(turnStrength*x) + round((forwardstrength - 1.0) * 200);
  } else{
    return 1500 - round(turnStrength*x);
  }
}

int curve2(int x, float forwardStrength, int turnleft, int turnright)
{
  if(turnleft == 1){
    return 1350;
  } else if(turnright == 1){
    return 1650;
  } else if(forwardstrength > 1.0){
    return 1500 + round(turnStrength*x) + round((forwardstrength - 1.0) * 200);
  } else {
    return 1500 + round(turnStrength*x);
  }
}

int curve3(int x, float forwardStrength, int turnleft, int turnrigh)
{
  if(turnleft == 1 || turnright == 1){
    return 1500;
  } else if(forwardstrength > 1.0){
    return 1500 + round(forwardStrength*x) + round((forwardstrength - 1.0) * 200);
  } else {
    return 1500 + round(forwardStrength*x);
  }
}

void servoLoop()
{
  servo1Us = curve1(turnClockAmount, forwardStrength, turnleft, turnright) ; //needs check
  servo2Us = curve2(turnClockAmount, forwardStrength, turnleft, turnright) ;
  servo3Us = curve3(forwardAmount, forwardStrength, turnleft, turnright) ;
  servo1.writeMicroseconds(servo1Us);
  servo2.writeMicroseconds(servo2Us);
  servo3.writeMicroseconds(servo3Us);
}

void setup()
{
  delay(100);
  // Debug console
  Serial.begin(9600);
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
}

void loop()
{
  Blynk.run();
  timer1.run();
  timer2.run();
}
