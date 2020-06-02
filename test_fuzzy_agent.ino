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
unsigned int reportInterval = 0;

bool isSend = true;

const int capacity = JSON_OBJECT_SIZE(16);

SonarController sonarController(TRIG_PIN_L_TOP, ECHO_PIN_L_TOP, TRIG_PIN_R_TOP, ECHO_PIN_R_TOP, TRIG_PIN_C_BOTTOM, ECHO_PIN_C_BOTTOM, TRIG_PIN_L_BOTTOM, ECHO_PIN_L_BOTTOM, TRIG_PIN_R_BOTTOM, ECHO_PIN_R_BOTTOM);

void setup() {
  Serial.begin(115200);
  inputString.reserve(20);

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
  isSend = false;
  timeStartInterval = millis();
  analogWrite(RightMotorVelocity, 0);
  analogWrite(LeftMotorVelocity, 0);
  if (rVelocity < 120 && lVelocity < 120) {
    analogWrite(RightMotorVelocity, 0);
    analogWrite(LeftMotorVelocity, 0);
  } else {
    if (rVelocity < 120 && lVelocity >= 120) {
      analogWrite(RightMotorVelocity, 0);
      analogWrite(LeftMotorVelocity, lVelocity);
    } else {
      if (rVelocity >= 120 && lVelocity < 120) {
        analogWrite(RightMotorVelocity, rVelocity);
        analogWrite(LeftMotorVelocity, 0);
      } else {
        analogWrite(RightMotorVelocity, rVelocity);
        analogWrite(LeftMotorVelocity, lVelocity);
      }
    }
  }
  sendReport();
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
  isSend = true;
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
  } else if (inputString[0] == 'M') {
    int sep = inputString.indexOf(';');
    moveRL((inputString.substring(1, sep)).toInt(), (inputString.substring(sep + 1)).toInt());
  } else if (inputString[0] == 'B') {
    int sep = inputString.indexOf(';');
    moveBackRL((inputString.substring(1, sep)).toInt(), (inputString.substring(sep + 1)).toInt());
  }
  inputString = "";
  stringComplete = false;
}

String createJson(String rrs, String rs, String fs, String ls, String lls) {
  String res = "";
  res.reserve(100);
  long times = millis();
  StaticJsonDocument<capacity> doc;
  doc["timeInterval"] = String(times);
  doc["rrs"] = rrs;
  doc["rs"] = rs;
  doc["fs"] = fs;
  doc["ls"] = ls;
  doc["lls"] = lls;
  serializeJson(doc, res);
  return res;
}

void sendReport() {
  float r[SONARS_N];
  sonarController.getRanges(r);
  Serial.println(createJson(String(r[0]), String(r[1]), String(r[2]), String(r[3]), String(r[4])));
  //moveStop();
}

bool isCompleteTime() {
  bool res = false;
  if (!isSend) {
    res = (millis() - timeStartInterval) >= reportInterval;
  }
  return res;
}

void loop() {

  if (sonarController.isReady()) {
    sonarController.start();
  }

  /*if (isCompleteTime()) {
    sendReport();
  }*/

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
