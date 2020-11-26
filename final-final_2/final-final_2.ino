#include <HUSKYLENS.h>
#include <PIDLoop.h>
#include <SoftwareSerial.h>
#include <ESP32Servo.h>

//ESP32の注意点として、書き込み時にBootボタンを押さないとコンパイルエラーが出ることがある


// Your WiFi credentials.
// Set password to "" for open networks.

//servo1:右スラスタ、正方向で後退
//servo2:前方スラスタ、正方向で前進
//servo3:左スラスタ正方向で後退
Servo servo1;
Servo servo2;
Servo servo3;
HUSKYLENS huskylens;
//HardwareSerial Serial1(2);
HardwareSerial mySerial(2);
//SoftwareSerial mySerial(3, 2); // RX（green）, TX(orange)
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
const int maxUs = 1900;
const int minUs = 1100;
const int servo1Period = 50;
const int servo2Period = 50;
const int servo3Period = 50;


const int rightfront = 4;
const int rightback = 16;
const int leftfront = 17;
const int leftback = 5;

const int servo1Pin = 9;//表記はGPIO12
const int servo2Pin = 10;//表記は13
const int servo3Pin = 11;//表記は13 11か?

int servo1Us = 1500;
int servo2Us = 1500;
int servo3Us = 1500;

int turnClockAmount = 0;
int forwardAmount = 0;
float turnStrength = 0.5;
float forwardStrength = -0.7;



//アプリ側でVirtual Pinに書き込みがあるたびに呼ばれる関数
//paramがV6に書き込まれたデータで、asInt()でInt型として処理 asFloatとかも色々ある
//BLYNK_WRITE(V6)
//{
//  int dutycycle = param.asInt();
//  ledcWrite(led_channel, dutycycle);
//}

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
  servo1Us = curve1(turnClockAmount);
  servo2Us = curve1(-turnClockAmount);
  servo3Us = curve2(forwardAmount);
  servo1.writeMicroseconds(servo1Us);
  servo2.writeMicroseconds(servo2Us);
  servo3.writeMicroseconds(servo3Us);
}

void setup()
{
   delay(100);
  // Debug console


  servo1.attach(servo1Pin, minUs, maxUs);
  servo2.attach(servo2Pin, minUs, maxUs);
  servo3.attach(servo3Pin, minUs, maxUs);

//  servo1.attach(servo1Pin);
//  servo2.attach(servo2Pin);
//  servo3.attach(servo3Pin);


  delay(5000);

  servo1.writeMicroseconds(1500);
  delay(1000);

  servo2.writeMicroseconds(1500);
delay(100);

  servo3.writeMicroseconds(1500);
    delay(1000);

     servo1.writeMicroseconds(1500);
  delay(1000);

  servo2.writeMicroseconds(1500);
delay(100);

  servo3.writeMicroseconds(1500);
    delay(1000);


  delay(3000);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
//
    Serial.begin(115200);
    mySerial.begin(9600);
    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    pinMode(4, OUTPUT);
    pinMode(16, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(5, OUTPUT);


    digitalWrite(4, LOW);
    digitalWrite(16, LOW);
    digitalWrite(17, LOW);
    digitalWrite(5, LOW);

    
}


void loop() {
  // 通信されていない時
    if (!huskylens.request()) {
      Serial.println(F("recheck the connection"));
      
    }
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    
    else if(!huskylens.available()) {
      Serial.println(F("nodata"));
        digitalWrite(leftfront, HIGH);
        digitalWrite(rightfront, LOW);// turn the LED on (HIGH is the voltage level)
          servo1.writeMicroseconds(1450); //1350
          servo2.writeMicroseconds(1550); //1350
          servo3.writeMicroseconds(1500); //1350
          delay(400);
    }
    else
    {
        digitalWrite(4, LOW);
        Serial.println(F("###########"));
        
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();

            digitalWrite(leftfront, HIGH);
            digitalWrite(rightfront, HIGH);
    // 前進
              servo1.writeMicroseconds(1450); //1450
              servo2.writeMicroseconds(1450); //1450
              servo3.writeMicroseconds(1450); //1450
              delay(400);

            
            // ターゲットが右側にある時
            if(result.xCenter >= 170){
      
              digitalWrite(rightfront, LOW);
              digitalWrite(leftfront, HIGH);
              servo1.writeMicroseconds(1500);
              servo2.writeMicroseconds(1420); //1400
              servo3.writeMicroseconds(1500);
              delay(400);
              Serial.println("LEFT");
            }

            // ターゲットが左側にある時
            else if(result.xCenter <= 130){
              digitalWrite(rightfront, HIGH);
              digitalWrite(leftfront, LOW);
              servo1.writeMicroseconds(1420); //1400
              servo2.writeMicroseconds(1500);
              servo3.writeMicroseconds(1500);
              delay(400);
              Serial.println("RIGHT");
            }
            
            printResult(result);
        }    
    }

    
}

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
