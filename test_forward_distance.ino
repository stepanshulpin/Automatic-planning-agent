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
  resetEncoderAndTimer();
  analogWrite(LeftMotorVelocity, velocity);
  analogWrite(RightMotorVelocity, velocity);
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

void moveStop() {
  analogWrite(LeftMotorVelocity, 0);
  analogWrite(RightMotorVelocity, 0);
  String res = "Left counter = ";
  res += pulsesL;
  res += " Right counter = ";
  res += pulsesR;
  res += " Time = ";
  res += (millis() - timeOld);
  res += " millis";  
  res += " Action = ";
  res += inputString;
  detachInterrupt(digitalPinToInterrupt(LeftMotorEncoder));
  detachInterrupt(digitalPinToInterrupt(RightMotorEncoder));
  Serial.println(res);
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
