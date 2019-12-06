// Power enables left and right motors.
#define pinWheelREnable 5
#define pinWheelLEnable 6
//Motor Left
#define motorL1   10    //  IN4                       
#define motorL2   9     //  IN3                       
//Motor Right
#define motorR1   8    //     IN1                    
#define motorR2   3     //    IN2                     
float mspeed;

float gyrAngle;                                   
float elapsedTime, currentTime, previousTime;
float setpoint;

float PID, error, previousError, cumError;
float pid_p, pid_i, pid_d;
float minPID, maxPID;


float kp=1;
float ki=0;
float kd=1;
int wait;

void setup() {
  
    Serial.begin(9600);
  
    pinMode(motorL1, OUTPUT);
    pinMode(motorL2, OUTPUT);
    pinMode(motorR1, OUTPUT);
    pinMode(motorR2, OUTPUT);
   
    setpoint = 500;    //REQUIRED ANGLE VALUE

    minPID = -523;
    maxPID = 500;
    wait = 50;
}

void loop() {
    currentTime = millis();  
    elapsedTime = (currentTime - previousTime);
  
    gyrAngle = analogRead(A0);     //IMU READING
    gyrAngle = constrain(gyrAngle, 0, 1023);      //SENSOR RANGE

    error = setpoint - gyrAngle;        
    cumError += error * elapsedTime;
    //Serial.println(setpoint);
    pid_p = kp*error;             //PROPORTIONAL ERROR
    pid_i = ki*cumError;          //INTERGRAL ERROR 
    pid_d = kd*((error - previousError)/elapsedTime);      //DIFFERENTIAL ERROR
    if (currentTime != 0)
    {
        /*
        Serial.print("current time:     ");
        Serial.println(currentTime);
        Serial.print("previous time:     ");
        Serial.println(previousTime);
        Serial.print("elapsed time:     ");
        Serial.println(elapsedTime);
        */
        PID = pid_p + pid_i + pid_d;       //TOTAL PID VALUE
        Serial.print("PID:     ");
        Serial.println(PID);
        previousError = error;            //UPDATING THE ERROR VALUE
        previousTime = currentTime;       //UPDATING PREVIOUS TIME


        if(gyrAngle<setpoint-5)
          {//neg
            mspeed = map(PID, 0, maxPID, 0, 255);           //Mapping PID values to a range of values to control motor speed
            mspeed = constrain(mspeed, 0, 255);
            forward();

          }
        if(gyrAngle>setpoint+5)
          {//pos
           mspeed = map(abs(PID), 0, abs(minPID), 0, 255);           //Mapping PID values to a range of values to control motor speed
           mspeed = constrain(mspeed, 0, 255);
               //mspeed = map(PID, 0, maxPID, 0, 255);           //Mapping PID values to a range of values to control motor speed
              // mspeed = constrain(mspeed, 0, 255);       
               back();
          }
        if(gyrAngle<=setpoint+5 && gyrAngle>=setpoint-5)
          {
              rest();
          }
    }
    Serial.println("");
    delay(wait);
}

//MOVEMENT FUNCTION
void forward()
{
    Serial.println("moves forward");
    Serial.print("Angle:     ");
    Serial.println(gyrAngle);
    //digitalWrite(motorL1, HIGH);
    digitalWrite(motorL1, HIGH);
    analogWrite(motorL2, LOW);
    //digitalWrite(motorR1, HIGH);
    digitalWrite(motorR1, HIGH);
    analogWrite(motorR2, LOW);
    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, mspeed);
    analogWrite(pinWheelLEnable, mspeed);
    Serial.print("mspeed     ");
    Serial.println(mspeed);
   // Serial.print("b     ");
   // Serial.println(b);
}
void back()
{
    Serial.println("moves backward");
    Serial.print("Angle:     ");
    Serial.println(gyrAngle);
    analogWrite(motorL1, LOW);
    //digitalWrite(motorL2, HIGH);
    digitalWrite(motorL2, HIGH);
    analogWrite(motorR1, LOW);
    //digitalWrite(motorR2, HIGH);
    digitalWrite(motorR2, HIGH);
    //setMotorSpeed(correctionAngle)
    analogWrite(pinWheelREnable, mspeed);
    analogWrite(pinWheelLEnable, mspeed);
    Serial.print("mspeed:     ");
    Serial.println(mspeed);

}
void rest()
{
    Serial.println("stops");
    Serial.print("Angle:     ");
    Serial.println(gyrAngle);
    digitalWrite(motorL1, LOW);
    digitalWrite(motorL2, LOW);
    digitalWrite(motorR1, LOW);
    digitalWrite(motorR2, LOW);
    Serial.print("mspeed     ");
    Serial.println(mspeed);

}
