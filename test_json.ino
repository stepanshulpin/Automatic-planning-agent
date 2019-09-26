#include <ArduinoJson.h>

//Controller L298N pins
const unsigned char RightMotorForward = 8;
const unsigned char RightMotorBackward = 7;
const unsigned char RightMotorVelocity = 6;
const unsigned char LeftMotorForward = 5;
const unsigned char LeftMotorBackward = 4;
const unsigned char LeftMotorVelocity = 9;

//Encoder pins
const unsigned char LeftMotorEncoder = 2;
const unsigned char RightMotorEncoder = 3;

//Counter
volatile unsigned int pulsesL;
volatile unsigned int pulsesR;

String inputString = "";        
bool stringComplete = false; 

unsigned long timeOld;


const int capacity = JSON_OBJECT_SIZE(6);

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
    
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(RightMotorVelocity, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);  
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(LeftMotorVelocity, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);  
  
  digitalWrite(RightMotorForward, 1);
  digitalWrite(RightMotorBackward, 0);  
  digitalWrite(LeftMotorForward, 1);
  digitalWrite(LeftMotorBackward, 0);
}

void resetEncoderAndTimer() {
  pulsesL=0;
  pulsesR=0;
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoder), counterL, FALLING);
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoder), counterR, FALLING);  
  timeOld = millis();
}

void moveForward(unsigned int velocity) { 
  Serial.println(velocity); 
  resetEncoderAndTimer();  
  analogWrite(RightMotorVelocity, velocity);
  analogWrite(LeftMotorVelocity, velocity);
}

void moveRight(unsigned int velocity) {
  resetEncoderAndTimer();
  analogWrite(LeftMotorVelocity, velocity);
  analogWrite(RightMotorVelocity, 0);
}

void moveLeft(unsigned int velocity) {  
  resetEncoderAndTimer();
  analogWrite(LeftMotorVelocity, 0);
  analogWrite(RightMotorVelocity, velocity);
}

String createJson(String pulsesL, String pulsesR, String timeInterval) {
  String res = "";
  res.reserve(200);
  StaticJsonDocument<capacity> doc;
  doc["pulsesL"] = pulsesL;
  doc["pulsesR"] = pulsesR;
  doc["timeInterval"] = timeInterval;
  serializeJson(doc, res);
  return res;
}

void moveStop() {
  analogWrite(LeftMotorVelocity, 0);
  analogWrite(RightMotorVelocity, 0);  
  Serial.println(createJson(String(pulsesL), String(pulsesR), String(millis() - timeOld))); 
  detachInterrupt(digitalPinToInterrupt(LeftMotorEncoder));
  detachInterrupt(digitalPinToInterrupt(RightMotorEncoder));
}

void loop() {
  if (stringComplete) {
    if(inputString[0] == 'F') {
      moveForward((inputString.substring(1)).toInt());      
    } else if(inputString[0] == 'R') {
      moveRight((inputString.substring(1)).toInt());
    } else if(inputString[0] == 'L') {
      moveLeft((inputString.substring(1)).toInt());
    } else if (inputString[0] == 'S') {
      moveStop();      
    }    
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }  
}



void counterL()
{
  pulsesL++;
}

void counterR()
{
  pulsesR++;
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
