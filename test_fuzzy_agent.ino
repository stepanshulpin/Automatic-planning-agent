#include <ArduinoJson.h>
#include <PinChangeInterrupt.h>

#include "SonarController.h"

//Пины сонаров
const unsigned char TRIG_PIN_R_TOP = 11;
const unsigned char ECHO_PIN_R_TOP = A2;
const unsigned char TRIG_PIN_L_TOP = 10;
const unsigned char ECHO_PIN_L_TOP = A1;
const unsigned char TRIG_PIN_C_BOTTOM = 12;
const unsigned char ECHO_PIN_C_BOTTOM = A0;
const unsigned char TRIG_PIN_L_BOTTOM = 13;
const unsigned char ECHO_PIN_L_BOTTOM = A3;
const unsigned char TRIG_PIN_R_BOTTOM = A5;
const unsigned char ECHO_PIN_R_BOTTOM = A4;

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
unsigned long timeStartInterval;
unsigned int reportInterval = 50;

bool isMoving = false;

const int capacity = JSON_OBJECT_SIZE(16);

SonarController sonarController(TRIG_PIN_L_TOP, ECHO_PIN_L_TOP, TRIG_PIN_R_TOP, ECHO_PIN_R_TOP, TRIG_PIN_C_BOTTOM, ECHO_PIN_C_BOTTOM, TRIG_PIN_L_BOTTOM, ECHO_PIN_L_BOTTOM, TRIG_PIN_R_BOTTOM, ECHO_PIN_R_BOTTOM);

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);

  pinMode(RightMotorBackward, OUTPUT);
  pinMode(RightMotorVelocity, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(LeftMotorVelocity, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);

  sonarController.init();
  sonarController.start();

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
  timeStartInterval = millis();
}

void moveRL(unsigned int rVelocity, unsigned int lVelocity) {
  resetEncoderAndTimer();  
  isMoving = true;
  if (rVelocity < 120 || lVelocity < 120) {
    analogWrite(RightMotorVelocity, 120);
    analogWrite(LeftMotorVelocity, 120);
    delay(40);
  }
  analogWrite(RightMotorVelocity, rVelocity);
  analogWrite(LeftMotorVelocity, lVelocity);
}

void moveBackRL(unsigned int rVelocity, unsigned int lVelocity) {
  resetEncoderAndTimer();  
  digitalWrite(RightMotorForward, 0);
  digitalWrite(RightMotorBackward, 1);
  digitalWrite(LeftMotorForward, 0);
  digitalWrite(LeftMotorBackward, 1);
  if (rVelocity < 120 || lVelocity < 120) {
    analogWrite(RightMotorVelocity, 175);
    analogWrite(LeftMotorVelocity, 175);
    delay(50);
  }
  analogWrite(RightMotorVelocity, rVelocity);
  analogWrite(LeftMotorVelocity, lVelocity);
}

void moveStop() {
  isMoving = false;
  analogWrite(LeftMotorVelocity, 0);
  analogWrite(RightMotorVelocity, 0);
  detachInterrupt(digitalPinToInterrupt(LeftMotorEncoder));
  detachInterrupt(digitalPinToInterrupt(RightMotorEncoder));
  digitalWrite(RightMotorForward, 1);
  digitalWrite(RightMotorBackward, 0);
  digitalWrite(LeftMotorForward, 1);
  digitalWrite(LeftMotorBackward, 0);
}

void processInputString() {
  if (inputString[0] == 'S') {
    moveStop();
  } else if (inputString[0] == 'M'){
    int sep = inputString.indexOf(';');
    moveRL((inputString.substring(1, sep)).toInt(), (inputString.substring(sep + 1)).toInt());
  } else if (inputString[0] == 'B'){
    int sep = inputString.indexOf(';');
    moveBackRL((inputString.substring(1, sep)).toInt(), (inputString.substring(sep + 1)).toInt());
  }
  Serial.println(inputString);
  inputString = "";
  stringComplete = false;
}

String createJson(String pulsesL, String pulsesR, String timeInterval, String rrs, String rs, String fs, String ls, String lls) {
  String res = "";
  res.reserve(200);
  StaticJsonDocument<capacity> doc;
  doc["pulsesL"] = pulsesL;
  doc["pulsesR"] = pulsesR;
  doc["timeInterval"] = timeInterval;
  doc["rrs"] = rrs;
  doc["rs"] = rs;
  doc["fs"] = fs;
  doc["ls"] = ls;
  doc["lls"] = lls;
  serializeJson(doc, res);
  return res;
}

void sendReport() {
  unsigned long timeInterval = millis() - timeOld;
  float r[SONARS_N];
  sonarController.getRanges(r);
  Serial.println(createJson(String(pulsesL), String(pulsesR), String(timeInterval), String(r[0]), String(r[1]), String(r[2]), String(r[3]), String(r[4])));
}

bool isCompleteTime() {
  bool res = false;
  if (isMoving) {
    res = (millis() - timeStartInterval) >= reportInterval;
    if (res) {
      timeStartInterval = millis();
    }
  }
  return res;
}

void loop() {

  if (sonarController.isReady()) {
    sonarController.start();
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

void _echo_isr() {
  switch (digitalRead(sonarController.getCurrent()->_echo)) {
    case HIGH:
      sonarController.getCurrent()->_start = micros(); 
      break;
    case LOW:
      sonarController.getCurrent()->stop();
      sonarController.next();
      break;
  }
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
