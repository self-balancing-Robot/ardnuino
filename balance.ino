//PD controller constants
Kp = 1;
Kd = 1;

//setPoint is 0 degrees, flat
setPoint = 0;

pinWheelsDir = pin for wheels direction;
pinWheelsSpeed = analogue pin for wheels speed;
pinGyroAngle = analogue pin for gyroscope angle;

void setup() {
  //digital pins
  pinMode(pinWheelsDir, OUTPUT);

  //analogue pins
  pinMode(pinGyroAngle, INPUT);
  pinMode(pinWheelsSpeed, OUTPUT);
}

void loop() {
  //calculate angle from pinGyroAngle
  analogAngleIn = analogRead(pinGyroAngle)
  //calculations ...
  degreesAngleIn = assign angle from calculation with pinGyroAngle;
  
  //getting error from feedback
  error = setPoint - degreesAngleIn;

  //applying Kp and Kd
  correctionAngle = error * Kp + error * Kd
  //converting correction from angle to analog votlage to supply wheels
  //calculations...
  correctionAnalog = result of calculation from angle to volts
  
  //supply correctionAnalog to pinWheelsSpeed
  analogWrite(pinWheelsSpeed, correctionAnalog)
}
