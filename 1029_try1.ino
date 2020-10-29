#define BLYNK_PRINT Serial

#include "C:\Users\SharedAccount\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\libraries\WiFi.h"
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include "config.h"

//ESP32の注意点として、書き込み時にBootボタンを押さないとコンパイルエラーが出ることがある
//
//参考
//Example集: https://examples.blynk.cc/?board=ESP32&shield=ESP32%20WiFi&example=GettingStarted%2FVirtualPinWrite
//ESP32のPWM: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
//ESP32のPWM: https://www.mgo-tec.com/blog-entry-ledc-pwm-arduino-esp32.html#title02

//トークン発行して入れてください
char auth[] = "I3ng7mU0BTlTGSoeXFETFmH201n-gvIf";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "803ZTa-04029B";
char pass[] =  "0129013a";

Servo servo1;
Servo servo2;
Servo servo3;
const int maxUs = 1900;
const int minUs = 1100;
const int servo1Pin = 13;//表記はGPIO12
const int servo1Period = 50;
const int servo2Pin = 14;//表記は13
const int servo2Period = 50;
const int servo3Pin = 0;//表記は1113
const int servo3Period = 50;
int servo1Us = 1500;
int servo2Us = 1500;
int servo3Us = 1500;

int turnClockAmount = 0;
int forwardAmount = 0;
float turnStrength = 0.5;//数値設定根拠が不明→todo
float forwardStrength = -0.7;
int speedmode = 0;



//アプリ側でVirtual Pinに書き込みがあるたびに呼ばれる関数
//paramがV6に書き込まれたデータで、asInt()でInt型として処理 asFloatとかも色々ある
//BLYNK_WRITE(V6)
//{
//  int dutycycle = param.asInt();
//  ledcWrite(led_channel, dutycycle);
//}

void myTimerEvent() 
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, millis() / 1000);  // 要理解→todo
}

BLYNK_WRITE(V8)
{
  turnClockAmount = param.asInt();  //ここでAppからの入力をセット
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
  return 1500 + round(turnStrength*x) + speedmode;
}

int curve2(int x)
{
  return 1500 - round(forwardStrength*x) + speedmode;
}

void servoLoop()
{
  servo1Us = curve1(turnClockAmount) + speedmode;   //PWM制御のパルス幅を設定
  servo2Us = curve1(-turnClockAmount) + speedmode;
  servo3Us = curve2(forwardAmount);
  servo1.writeMicroseconds(servo1Us);　//PWM制御実行。つまりモータを入力値に合わせて駆動
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
  servo1.attach(servo1Pin, minUs, maxUs);　//マイコンとモータの接続PINを設定

  servo2.setPeriodHertz(servo2Period);
  servo2.attach(servo2Pin, minUs, maxUs);

  servo3.setPeriodHertz(servo3Period);
  servo3.attach(servo3Pin, minUs, maxUs);

  delay(3000);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  Blynk.begin(auth, ssid, pass);　//APPとの接続を確立

  timer1.setInterval(1000L, myTimerEvent);　//1000m秒ごとにこの関数を実行
  timer2.setInterval(20L, servoLoop);
}

void loop()　//この中は頻繁に実行される
{
  Blynk.run();　//APPとの接続を確認
  timer1.run(); // Initiates BlynkTimer
  timer2.run();
}
