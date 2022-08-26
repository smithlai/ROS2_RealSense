/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 28 May 2015
  by Michael C. Miller
  modified 8 Nov 2013
  by Scott Fitzgerald

  http://arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <ArduinoJson.h>

StaticJsonDocument<1024> doc;


const int yawPin = 13;
const int pitchPin = 3;

Servo yawServo;  // create servo object to control a servo
Servo pitchServo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int posyaw;
int pospitch;
int stepyaw=1;
int steppitch=1;

#define BUFFER_LEN 32
char inputString[BUFFER_LEN+1];         // a String to hold incoming data



void setup() {
  Serial.begin(115200);
  yawServo.attach(yawPin);  // attaches the servo on GIO13 to the servo object
  pitchServo.attach(pitchPin);  // attaches the servo on GIO3 to the servo object
  pitchServo.write(0);
  yawServo.write(90);
}

void loop() {
  readSerial();
  delay(100);
  
  bool parsed=false;
  if (strlen(inputString)){
    Serial.print("Arduino: recv:");
    Serial.println(inputString);
    parsed=parseCmd(inputString);
    inputString[0]='\0';
  }
  if (parsed){
    int p = doc["p"];
    int r = doc["r"];
    int y = doc["y"];
    Serial.print("Arduino: rotate:");
    Serial.print(p);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(y);
    rotate(p,r,y);
    delay(1000); 
  }

}

void rotate(int p,int r, int y){
  pitchServo.write(p);
  yawServo.write(y);
}

void readSerial() {
  int cnt = 0;
  while (Serial.available() > 0) {
    // read the incoming byte:  
    char c = Serial.read();
    if ('\n' == c || '\r'== c){
      c = '\0';
    }
    if (cnt < BUFFER_LEN){
      inputString[cnt]=c;
    }
    cnt++;
  }
}
//{"p":90,"r":0,"y":90} 
bool parseCmd(const char* json){
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.c_str());
    return false;
  }
  return true;
}
