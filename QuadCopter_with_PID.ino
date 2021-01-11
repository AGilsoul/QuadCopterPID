#include <Wire.h>
#include <Servo.h>
#include <Ethernet.h>

//radians to degrees multiplier
const float Radians_Degrees = 180/3.141592654;

//radio input pin
const int THR = 3;
const int AIL = 10;
const int ELE = 6;
const int RUD = 11;

//ESC output pin
const int EpinBL = 2;
const int EpinBR = 8;
const int EpinFL = 7;
const int EpinFR = 4;

//Throttle at 90 should be hover, above elevate, below go down
int Throttle;
int Aileron;
int Eleron;
int Rudder;

bool moving;

float desired_AngleX;
float desired_AngleY;

float finalValue;

//ESC constants
double kp = 0.05;
double ki = 0.0;
double kd = 0.05;


//16 bit data from IMU
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;;

//Float arrays for X and Y values
float Acceleration_Angle[2];
float Gyro_Angle[2];
float Total_Angle[2];

//Variables for time keeping
float elapsedTime, time, timePrev;

//variables for ESC math
float difX = 0;
float difY = 0;

float PropX = 0;
float IntgX = 0;
float DerX = 0;

float PropY = 0;
float IntgY = 0;
float DerY = 0;
float lastDifX = 0;
float lastDifY = 0;
float PIDX;
float PIDY;

//ESCs for each motor
Servo EscBL;
Servo EscBR;
Servo EscFL;
Servo EscFR;

double frSpeed;
double brSpeed;
double flSpeed;
double blSpeed;
double baseLine = 0;

bool moveMent = false;

//checks if motors are in range to be added or subtracted to
bool motorsInRange;
bool motorsBelow;
bool motorsAbove;

//Averages for finding hovering values for baseline speeds
int AvgCount = 0;
double xAvg = 0;
double yAvg = 0;

void setup() {

  //Set up radio input pin
  pinMode (THR, INPUT);
  pinMode (AIL, INPUT);
  pinMode (ELE, INPUT);
  pinMode (RUD, INPUT);

  //Attach ESCs to pins
  EscBL.attach(EpinBL, 1000, 2000);
  EscBL.write(0);
  EscBL.write(180);
  EscBL.write(0);
  delay(1000);

  EscBR.attach(EpinBR, 1000, 2000);
  EscBR.write(0);
  EscBR.write(180);
  EscBR.write(0);
  delay(1000);

  EscFL.attach(EpinFL, 1000, 2000);
  EscFL.write(0);
  EscFL.write(180);
  EscFL.write(0);
  delay(1000);

  EscFR.attach(EpinFR, 1000, 2000);
  EscFR.write(0);
  EscFR.write(180);
  EscFR.write(0);
  delay(1000);

  //Start wire communications with IMU
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  Serial.begin(9600);

  //Start keeping track time in milliseconds
  time = millis();
  
  flSpeed =  0;
  blSpeed = 0;
  frSpeed = 0;
  brSpeed = 0;
  desired_AngleX = -4.5;
  desired_AngleY = -2.55;
  
}

void loop() {
  //Time keeping
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  //UNCOMMENT WHEN READY FOR CONTROLLER INPUT
  
  //commented out for now cus they make it run slower when controller isnt on
  Throttle = pulseIn(THR, HIGH);
  Aileron = pulseIn(AIL, HIGH);
  Eleron = pulseIn(ELE, HIGH);
  Rudder = pulseIn(RUD, HIGH);

  Aileron = map(Aileron, 1089, 1875, 0, 180) - 90;
  Eleron = map(Eleron, 1089, 1875, 0, 180) - 90;
  Rudder = map(Rudder, 1089, 1875, 0, 180) - 90;
  

  //Get accelerometer values
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  Acc_rawX = Wire.read()<<8|Wire.read();
  Acc_rawY = Wire.read()<<8|Wire.read();
  Acc_rawZ = Wire.read()<<8|Wire.read();

  //Angles calculated with Euler equation
  Acceleration_Angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * Radians_Degrees;
  Acceleration_Angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * Radians_Degrees;

  //Get gyro data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  Gyr_rawX = Wire.read()<<8|Wire.read();
  Gyr_rawY = Wire.read()<<8|Wire.read();
  Gyro_Angle[0] = Gyr_rawX / 131.0;
  Gyro_Angle[1] = Gyr_rawY / 131.0;
  
  //Get total X angle change rate over time
  Total_Angle[0] = 0.98 * (Total_Angle[0] + Gyro_Angle[0] * elapsedTime) + 0.02 * Acceleration_Angle[0];
  //Get total Y angle change rate over time
  Total_Angle[1] = 0.98 * (Total_Angle[1] + Gyro_Angle[1] * elapsedTime) + 0.02 * Acceleration_Angle[1];
  //IGNORE Stuff with controller

  /*
  Serial.println("\nX angle: ");
  Serial.println(Total_Angle[0]);
  Serial.println("Y angle: ");
  Serial.println(Total_Angle[1]);
  */
  
  
  //Gets error between desired angle and current angle
  difX = Total_Angle[0] - desired_AngleX;
  // if propX is greater than 0, it is tilting right, if less than 0, tilting left
  difY = Total_Angle[1] - desired_AngleY;
  // if propY is greater than 0, tilting backward, if less than 0, tilting forwards
  
  //Gets the proportion of the error
  PropX = kp * difX;
  PropY = kp * difY;

  //Gets integrals of angles
  if ( -3 < PropX < 3) {
    IntgX += ki * PropX;
  }

  if (-3 < PropY < 3) {
    IntgY += ki * PropY;
  }
  
  //Gets the derivative, or rate of change of the error
  DerX = kd * ((difX - lastDifX) / elapsedTime);
  DerY = kd * ((difY - lastDifY) / elapsedTime);

  //Final PID value
  PIDX = PropX + IntgX + DerX; 
  PIDY = PropY + IntgY + DerY;

  if (PIDX < -1000) {
    PIDX = -1000;
  }
  if (PIDY < -1000) {
    PIDX = -1000;
  }
  if (PIDX > 1000) {
    PIDX = 1000;
  }
  if (PIDY > 1000) {
    PIDX = 1000;
  }

  flSpeed = Throttle - PIDX;
  blSpeed = Throttle - PIDX;
  frSpeed = Throttle + PIDX;
  brSpeed = Throttle + PIDX;

  flSpeed = (flSpeed + Throttle - PIDY) / 2;
  frSpeed = (frSpeed + Throttle - PIDY) / 2;
  blSpeed = (blSpeed + Throttle + PIDY) / 2;
  brSpeed = (brSpeed + Throttle + PIDY) / 2;

  flSpeed = map(flSpeed, 1089, 1875, 0, 180);
  frSpeed = map(frSpeed, 1089, 1875, 0, 180);
  blSpeed = map(blSpeed, 1089, 1875, 0, 180);
  brSpeed = map(brSpeed, 1089, 1875, 0, 180);

  
  flSpeed += ((-Aileron / 5) - (Eleron / 5) + (Rudder / 5)) / 3;
  blSpeed += ((-Aileron / 5) + (Eleron / 5) - (Rudder / 5)) / 3;
  frSpeed += ((Aileron / 5) - (Eleron / 5) - (Rudder / 5)) / 3;
  brSpeed += ((Aileron / 5) + (Eleron / 5) + (Rudder / 5)) / 3;
  

  if (flSpeed > 180) {
    flSpeed = 180;
  }
  if (flSpeed < 0) {
    flSpeed = 0;
  }
  if (frSpeed > 180) {
    frSpeed = 180;
  }
  if (frSpeed < 0) {
    frSpeed = 0;
  }
  if (blSpeed > 180) {
    blSpeed = 180;
  }
  if (blSpeed < 0) {
    blSpeed = 0;
  }
  if (brSpeed > 180) {
    brSpeed = 180;
  }
  if (brSpeed < 0) {
    brSpeed = 0;
  }

  EscFL.write(flSpeed);
  EscFR.write(frSpeed);
  EscBL.write(blSpeed);
  EscBR.write(brSpeed);

  lastDifX = difX;
  lastDifY = difY;

  //Serial.println(PIDX);
  //Serial.println(PIDY);

  /*
  Serial.print("\nFront Right: ");
  Serial.println(frSpeed); 
  Serial.print("Front Left: ");
  Serial.println(flSpeed); 
  Serial.print("Back Right: ");
  Serial.println(brSpeed); 
  Serial.print("Back Left: ");
  Serial.println(blSpeed);
  */
  

  //Maps input from controller to ESC usable values
  //Throttle = map(Throttle, 1150, 1875, 0, 180);
  
}
