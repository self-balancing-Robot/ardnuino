
//PID controller constants
float Kp = 1.0;
float Kd = 1.0;
float Ki = 1.0;

//setPoint is 0 degrees, 680 is the analog bit equivalent
int setPoint = 680;
  
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


//The area under the graph of error over time from 
//t = 0 to t = currentTime - intervalTime
int previousArea = 0;
//The error recorded from the previous interval
int previousError = 0;


//Time between each measurement and correction
int intervalTime = 35;

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
  
  //NOTE: although Ki, Kd and Kp are float types and the calculations
  //assign to integer variables, by C's design when a calculation is 
  //done with both integers and floats, the integers are converted to
  //float by default. Furthermore, also by default, when a float is 
  //assigned to an integer variable it gets rounded to the nearest.
  
  //current error * Kp
  int pCorrection = error * Kp;
  //current interval's trapezium area * Ki
  int iTrapeziumArea = 0.5 * (previousError + error) * intervalTime; 
  int iCorrection = iTrapeziumArea * Ki;
  //current interval's gradient * kd
  int dDeriv = ((error - previousError)/intervalTime);
  int dCorrection = dDeriv * Kd;

  //Adding individual P, I and D corrections to total correction
  int correction = pCorrection + iCorrection + dCorrection;
  
  //refresh variables
  previousArea = previousArea + iTrapeziumArea;
  previousError = error;
   

  //correction is positive/negative to deduce the direction
  //of motor rotation.
  //We don't need that as we use analogAngle to dictate the direction.
  //Therefore correctionAngle is only needed for its magnitude
  //so we will get its absolute value.
  correction = abs(correction);
  
  //deadband zones
  int lowerDeadband = 640;
  int upperDeadband = 720;
  
  //if it is tilting forward(neg correctionAngle), go forward
  if (analogAngle < lowerDeadband){
    Serial.println(analogAngle);
    digitalWrite(pinWheelR1, HIGH);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, HIGH);

    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, correction);
    analogWrite(pinWheelLEnable, correction);
  }
  //if it is tilting backward(positive correction), go back
  if (analogAngle > upperDeadband){
    Serial.println(analogAngle);
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, HIGH);
    digitalWrite(pinWheelL1, HIGH);
    digitalWrite(pinWheelL2, LOW);

    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, correction);
    analogWrite(pinWheelLEnable, correction);
  }
  //if it's balanced, leave it
  if (lowerDeadband <= analogAngle && analogAngle <= upperDeadband){
    Serial.println(analogAngle);
    
    //setMotorSpeed(0)
    analogWrite(pinWheelREnable, 0);
    analogWrite(pinWheelLEnable, 0);
  }

  delay(intervalTime);
}
