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
unsigned long timeStratInterval;
unsigned int reportInterval = 100;

unsigned int pulsesCountForDistance = 0;
bool isMoving = false;


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
  pulsesL = 0;
  pulsesR = 0;
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoder), counterL, FALLING);
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoder), counterR, FALLING);
  timeOld = millis();
}

void moveForward(unsigned int velocity) {
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
  unsigned long timeInterval = millis() - timeOld;
  delay(500);
  Serial.println(createJson(String(pulsesL), String(pulsesR), String(timeInterval)));
  detachInterrupt(digitalPinToInterrupt(LeftMotorEncoder));
  detachInterrupt(digitalPinToInterrupt(RightMotorEncoder));
}

void moveForwardWithDistance(unsigned int dist) {
  pulsesCountForDistance = dist / 20.7 * 20;
  Serial.print("pulsesCountForDistance= ");
  Serial.println(pulsesCountForDistance);
  isMoving = true;
  timeStartInterval = millis();
  moveForward(255);
}

void processInputString() {
  if (inputString[0] == 'F') {
      moveForward((inputString.substring(1)).toInt());
    } else if (inputString[0] == 'R') {
      moveRight((inputString.substring(1)).toInt());
    } else if (inputString[0] == 'L') {
      moveLeft((inputString.substring(1)).toInt());
    } else if (inputString[0] == 'D') {
      if(inputString[1] == 'F') {
        moveForwardWithDistance((inputString.substring(2)).toInt())
      }
    } else if (inputString[0] == 'S') {
      moveStop();
    }
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
}

void sendReport() {  
  unsigned long timeInterval = millis() - timeOld;  
  Serial.println(createJson(String(pulsesL), String(pulsesR), String(timeInterval)));
}

void isCompleteMove() {
  return (pulsesR >= pulsesCountForDistance || pulsesL >= pulsesCountForDistance);
}

void isCompleteTime() {
  bool res = false;
  if(isMoving) {
    res = (millis() - timeStartInterval) >= reportInterval;
    timeStartInterval = millis();
  }
  return res;
}

void loop() {

  if(isCompleteMove()) {
    isMoving = false;
    moveStop();  
  }

  if (isCompleteTime()) {
    sendReport();
  }

  if (stringComplete) {
    processInputString();
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
