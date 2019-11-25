#include <Wire.h>

//PD controller constants
float Kp = 1.0;
float Kd = 1.0;


//setPoint is 0 degrees, fl
int setPoint = 511;
  
#define pinWheelR1 8
#define pinWheelR2 3
#define pinWheelL1 10
#define pinWheelL2 9
#define pinWheelREnable 5
#define pinWheelLEnable 6
#define pinGyroAngle 0

//To move forward:
//  R1: TRUE R2: FALSE
//  L1: TRUE L2: FALSE

//To move back:
//  R1: FALSE R2: TRUE
//  L1: FALSE L2: TRUE

float speedMotor = 50;

float lastAngle = 0;
void setup() {
  //pin setup
  pinMode(pinWheelR1, OUTPUT);
  pinMode(pinWheelR2, OUTPUT);
  pinMode(pinWheelL1, OUTPUT);
  pinMode(pinWheelL2, OUTPUT);
  pinMode(pinWheelREnable, OUTPUT);
  pinMode(pinWheelLEnable, OUTPUT);
  
  pinMode(pinGyroAngle, INPUT);


  
  //Begin serial interface
  Serial.begin(9600);
}

void loop() {
  //get angle from pinGyroAngle
  float analogAngle = analogRead(pinGyroAngle);
  //0 to 510 bits is a forward tilt
  //511 is vertical
  //512 to 1023 bits is a backward tilt

  //ENABLER
  analogWrite(pinWheelREnable, 255);
  analogWrite(pinWheelLEnable, 255);
  
  //getting error from feedback
  float error = setPoint - analogAngle;
  //applying Kp and Kd
  //correctionAngle = error * Kp + d/dt (error) * Kd
  float correctionAngle = error * Kp + ((error - lastAngle)/50) * Kd;
  //refresh lastAngle
  lastAngle = analogAngle;
   

     
  //if it is tilting forward(correctionAngle < 0), go forward
  if (analogAngle < 511){
    Serial.println("Moving forward");
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, HIGH);
    digitalWrite(pinWheelL1, HIGH);
    digitalWrite(pinWheelL2, LOW);
    //analogWrite(pinWheelR1, speedMotor);
    //analogWrite(pinWheelR2, 0);
    //analogWrite(pinWheelL1, 0);
    //analogWrite(pinWheelL2, speedMotor);
  }
  //if it is tilting backward(correctionAngle > 0), go back
  if (analogAngle > 511){
    Serial.println("Moving backward");
    digitalWrite(pinWheelR1, HIGH);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, HIGH);
    //analogWrite(pinWheelR1, 0);
    //analogWrite(pinWheelR2, speedMotor);
    //analogWrite(pinWheelL1, speedMotor);
    //analogWrite(pinWheelL2, 0);
  }
  //if it's balanced, leave it
  if (analogAngle == 511){
    Serial.println("Stable");
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, LOW);
    //analogWrite(pinWheelR1, 0);
    //analogWrite(pinWheelR2, 0);
    //analogWrite(pinWheelL1, 0);
    //analogWrite(pinWheelL2, 0);
  }

  delay(100);
  
}
