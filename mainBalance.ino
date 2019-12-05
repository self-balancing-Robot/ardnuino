//PID controller constants
float Kp = 100.0;
float Kd = 0.0;
float Ki = 0.0;

//setPoint is 0 degrees, 560 is the analog bit equivalent
int setPoint = 560;
  
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



//The error recorded from the previous interval
int previousError = 0;

//Time between each measurement and correction
int intervalTime = 1;

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
  int error = setPoint - analogAngle;
  //positive error = backwards tilt
  //negative error = forwards tilt
  
  
  //applying Kp, Ki and Kd 
  
  //NOTE: although Ki, Kd and Kp are float types and the calculations
  //assign to integer variables, by C's design when a calculation is 
  //done with both integers and floats, the integers are converted to
  //float by default. Furthermore, also by default, when a float is 
  //assigned to an integer variable it gets rounded to the nearest.
  
  //current error * Kp
  int pCorrection = error * Kp;
  //current interval's trapezium area * Ki
  int iTrapeziumArea = (intervalTime/1000) * (previousError + error) * intervalTime; 
  int iCorrection = iTrapeziumArea * Ki;
  //current interval's gradient * kd
  int dDeriv = ((error - previousError)/intervalTime);
  int dCorrection = dDeriv * Kd;

  //Adding individual P, I and D corrections to total correction
  int correction = pCorrection + iCorrection + dCorrection;
  
  //refresh variables
  previousError = error;
   

  //correction is positive/negative to deduce the direction
  //of motor rotation.
  //We don't need that as we use error to dictate the direction.
  //Therefore correction is only needed for its magnitude
  //so we will get its absolute value.
  correction = abs(correction);
  wheelSpeed = correction/(512 * Kp) * 255;
  
  //deadband zones
  int lowerDeadband = -40;
  int upperDeadband = 40;
  
  //if it is tilting forward(negative correction), go forward
  if (error < lowerDeadband){
    Serial.println(analogAngle);
    digitalWrite(pinWheelR1, LOW);
    digitalWrite(pinWheelR2, HIGH);
    digitalWrite(pinWheelL1, HIGH);
    digitalWrite(pinWheelL2, LOW);

    //setMotorSpeed(correctionAngle)
    int wheelSpeed = correction / (error * )
    analogWrite(pinWheelREnable, wheelSpeed);
    analogWrite(pinWheelLEnable, wheelSpeed);
  }
  //if it is tilting backward(positive correction), go back
  if (error > upperDeadband){
    Serial.println(analogAngle);
    digitalWrite(pinWheelR1, HIGH);
    digitalWrite(pinWheelR2, LOW);
    digitalWrite(pinWheelL1, LOW);
    digitalWrite(pinWheelL2, HIGH);

    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, wheelSpeed);
    analogWrite(pinWheelLEnable, wheelSpeed);
  }
  //if it's balanced, leave it
  if (error <= analogAngle && error <= upperDeadband){
    Serial.println(analogAngle);
    
    //setMotorSpeed(0)
    analogWrite(pinWheelREnable, 0);
    analogWrite(pinWheelLEnable, 0);
  }

  delay(intervalTime);
}
