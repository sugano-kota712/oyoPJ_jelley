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
const int servo1Pin = 4;//
const int servo1Period = 50;
const int servo2Pin = 16;//
const int servo2Period = 50;
const int servo3Pin = 18;//
const int servo3Period = 50;
int servo1Us = 1500;
int servo2Us = 1500;
int servo3Us = 1500;

int turnClockAmount = 0;
int forwardAmount = 0;
float turnStrength = 0.5;
float forwardStrength = -0.7;
int speedmode = 0;




void myTimerEvent() 
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, millis() / 1000);
}

BLYNK_WRITE(V8)
{
  turnClockAmount = param.asInt();
}

BLYNK_WRITE(V9)
{
  forwardAmount = param.asInt();
}

BLYNK_WRITE(V10)
{
  speedmode = param.asInt();
}

BLYNK_WRITE(V11)
{
  turnStrength = param.asFloat();
}

BLYNK_WRITE(V12)
{
  forwardStrength = param.asFloat();
}


BlynkTimer timer1;
BlynkTimer timer2;


int curve1(int x)
{
  return 1500 + round(turnStrength*x);
}

int curve2(int x)
{
  return 1500 - round(forwardStrength*x);
}

void servoLoop()
{
  servo1Us = curve1(turnClockAmount) + speedmode;
  servo2Us = curve1(-turnClockAmount) + speedmode;
  servo3Us = curve2(forwardAmount) + speedmode;
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
