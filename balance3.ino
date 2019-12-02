
//PID controller constants
float Kp = 1.0;
float Kd = 1.0;
float Ki = 1.0;

//setPoint is 0 degrees, 511 is the analog bit equivalent
int setPoint = 511;
  
#define pinWheelR1 8
#define pinWheelR2 3
#define pinWheelL1 10
#define pinWheelL2 9
#define pinWheelREnable 5
#define pinWheelLEnable 6
#define pinGyroAngle 0

//To move forward:
//  R1: LOW R2: HIGH
//  L1: HIGH L2: LOW

//To move back:
//  R1: HIGH R2: LOW
//  L1: LOW L2: HIGH


//To keep track of area by using trapzeium rule so for 
//every loop iteration(50ms), get the iteration's 
//trapezium area and add this to previousArea.
//This is for the integral part of PID control
int previousArea = 0;
int previousError = 0;
//To get the derivative by getting the gradient from
//this variable to the current measured angle
//This is for the derivative part of the PID control
int lastAngle = 0;


//Time between each measurement and correction
int intervalTime = 50;

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
  int analogAngle = analogRead(pinGyroAngle);
  //0 to 510 bits is a forward tilt
  //511 is vertical
  //512 to 1023 bits is a backward tilt

  
  //getting error from feedback
  //positive if angle > 511 (forward tilt)
  //negative if angle < 511 (backward tilt) 
  int error = setPoint - analogAngle;
  
  //applying Kp, Ki and Kd
  //current error * Kp
  int pCorrection = error * Kp;
  //current interval's trapezium area * Ki
  int iTrapeziumArea = 0.5 * (previousError + error) * intervalTime; 
  int iCorrection = iTrapeziumArea * Ki;
  //current interval's gradient * kd
  int dDeriv = ((error - lastAngle)/intervalTime);
  int dCorrection = dDeriv * Kd;

  //Adding individual P, I and D corrections to total correction
  int correction= pCorrection + iCorrection + dCorrection;
  
  //refresh variables
  previousArea = previousArea + iTrapeziumArea;
  previousError = error;
  lastAngle = analogAngle;
   

  //correction is positive/negative to deduce the direction
  //of motor rotation.
  //We don't need that as we use analogAngle to dictate the direction.
  //Therefore correctionAngle is only needed for its magnitude
  //so we will get its absolute value.
  correction = abs(correction);
  
  //if it is tilting forward(neg correctionAngle), go forward
  if (analogAngle < 511){
    Serial.println("Moving forward");
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, HIGH);
    digitalWrite(pinWheelL1, HIGH);
    digitalWrite(pinWheelL2, LOW);

    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, correction);
    analogWrite(pinWheelLEnable, correction);
  }
  //if it is tilting backward(pos correctionAngle), go back
  if (analogAngle > 511){
    Serial.println("Moving backward");
    digitalWrite(pinWheelR1, HIGH);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, HIGH);

    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, correction);
    analogWrite(pinWheelLEnable, correction);
  }
  //if it's balanced, leave it
  if (analogAngle == 511){
    Serial.println("Stable");
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, LOW);

    analogWrite(pinWheelREnable, 0);
    analogWrite(pinWheelLEnable, 0);
  }

  delay(intervalTime);
}
