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
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // put your setup code here, to run once:  
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

void moveForward() {  
  pulsesL=0;
  pulsesR=0;
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoder), counterL, FALLING);
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoder), counterR, FALLING);
  analogWrite(LeftMotorVelocity, 255);
  analogWrite(RightMotorVelocity, 255);
  timeOld = millis();
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
  detachInterrupt(digitalPinToInterrupt(LeftMotorEncoder));
  detachInterrupt(digitalPinToInterrupt(RightMotorEncoder));
  Serial.println(res);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println(inputString);
    if(inputString == "255\n"){
      moveForward();
    } else if (inputString == "0\n") {
      moveStop();      
    }
    // clear the string:
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
    // get the new byte:
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
