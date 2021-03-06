#include <SharpIR.h>

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ********************************************************* Define global variables **********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
int ENCODER_1_PIN, ENCODER_2_PIN; // Encoder pinouts
int MOTOR_A_DIR, MOTOR_A_BRAKE, MOTOR_A_PWM, MOTOR_B_DIR, MOTOR_B_BRAKE, MOTOR_B_PWM; // Motor pinouts; A is right, B is left
int MOTOR_A_FWD, MOTOR_A_REV, MOTOR_B_REV, MOTOR_B_FWD; // Motor direction constants
int CLOCKWISE = 1, COUNTER_CLOCKWISE = 0;
int ENCODER_1, ENCODER_2; // To track when the encoders receive pulses

double IR_SENSOR_DISTANCE = 30; // distance from center of device to IR sensor

SharpIR IRLeft(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRRight(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFront(SharpIR::GP2Y0A21YK0F, A5);

int MAX_SPEED = 255;
int DEMO_DIST = 50;

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * With the exception of setup() and loop(), functions are initialized below and alphabetized beneath setup() and loop() for easy navigation.
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void Brake(int brakePin, bool brake);
void Forward(int s);
void Reverse(int spd);
void TurnRight(int spd);
void TurnLeft(int spd);
int ReadIRFront();
int ReadIRLeft();
int ReadIRRight();

// --- Don't call this directly, call the above methods ---
void RightTrack(int dir, int spd); 
void LeftTrack(int dir, int spd);
int ReadIR(SharpIR sensor);
/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

void setup()
{
  // Begin serial comms for testing
  Serial.begin(9600);
  
  // Define analog pinouts #TODO - set actual pinouts
  ENCODER_1_PIN = A0;
  ENCODER_2_PIN = A2;

  // Define digital pinouts
  pinMode(12, OUTPUT);
  MOTOR_A_DIR = 12;
  pinMode(9, OUTPUT);
  MOTOR_A_BRAKE = 9;
  pinMode(3, OUTPUT);
  MOTOR_A_PWM = 3;
  MOTOR_A_FWD = LOW;
  MOTOR_A_REV = HIGH;
  pinMode(13, OUTPUT);
  MOTOR_B_DIR = 13;
  pinMode(8, OUTPUT);
  MOTOR_B_BRAKE = 8;
  pinMode(11, OUTPUT);
  MOTOR_B_PWM = 11;
  MOTOR_B_FWD = HIGH;
  MOTOR_B_REV = LOW;

  // init motors
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
}

void loop()
{
  int front_IR_dist = 0, left_IR_dist = 0, right_IR_dist = 0;
  int spd = MAX_SPEED;
  while(!ReadHallEffect())
  {
    Forward(spd);
  }
  // Debouncing
  delay(50);
  while(ReadHallEffect());
  delay(50);
  while(!ReadHallEffect())
  {
    Reverse(spd);
  }
  // Debouncing
  delay(50);
  while(ReadHallEffect());
  delay(50);
  while(!ReadHallEffect())
  {
    Stop();
    //Forward(spd);
    //Reverse(spd);
    //RightTrack(MOTOR_A_FWD, spd);
    //LeftTrack(MOTOR_B_FWD, spd);
    //RightTrack(MOTOR_A_REV, spd);
    //LeftTrack(MOTOR_B_REV, spd);
  }
  // Debouncing
  delay(50);
  while(ReadHallEffect());
  delay(50);
  Serial.println(ReadHallEffect());
  // demo loop
  /*
  while (true)
  {
    front_IR_dist = ReadIRFront();
    Serial.println(front_IR_dist);
    if (front_IR_dist < DEMO_DIST)
    {
      TurnRight(MAX_SPEED);
    }
    else
    {
      Forward(MAX_SPEED);
    }
  }
  */
}


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * *********************************************************** Functions are below. ***********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

void Brake(int brakePin, bool brake)
{
  if(brake) 
  {
    digitalWrite(brakePin, HIGH);
  }
  else
  {
    digitalWrite(brakePin, LOW);
  }
}

void Forward(int spd)
{
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

void Reverse(int spd)
{
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void TurnRight(int spd)
{
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

void TurnLeft(int spd)
{
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void Stop()
{
  Brake(MOTOR_A_BRAKE, true);
  Brake(MOTOR_B_BRAKE, true);
}

int ReadIRLeft()
{
  return ReadIR(IRLeft);
}

int ReadIRRight()
{
  return ReadIR(IRRight);
}

int ReadIRFront()
{
  return ReadIR(IRFront);
}

/* ------------------------------------------------------------------------------------------------------------------------------
 * ********************************************* Low Level Functions are below. *************************************************
 * ------------------------------------------------------------------------------------------------------------------------------
 */

void RightTrack(int dir, int spd)
{
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }

/*
  Serial.println("DIR:");
  Serial.println(digitalRead(MOTOR_A_DIR));
  Serial.println("PIN:");
  Serial.println(dir);
  delay(1000);
*/
  if (digitalRead(MOTOR_A_DIR) == dir)
  {
    Brake(MOTOR_A_BRAKE, false);
    analogWrite(MOTOR_A_PWM, spd);
  }
  else // If we're changing directions, we need to stop first
  { 
    Brake(MOTOR_A_BRAKE, true);
    digitalWrite(MOTOR_A_DIR, dir);
  }
}

void LeftTrack(int dir, int spd)
{
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }
  
  if (digitalRead(MOTOR_B_DIR) == dir)
  {
    Brake(MOTOR_B_BRAKE, false);
    analogWrite(MOTOR_B_PWM, spd);
  }
  else // If we're changing directions, we need to stop first
  { 
    Brake(MOTOR_B_BRAKE, true);
    digitalWrite(MOTOR_B_DIR, dir);
  }
}

int ReadIR(SharpIR sensor)
{
  int number_of_readings = 5;
  int sum = 0;
  for(int i = 0; i < number_of_readings; i++)
  {
    sum += sensor.getDistance();
  }
  return sum / number_of_readings;  
}

bool ReadHallEffect()
{
  Serial.println(!digitalRead(52));
  return !digitalRead(52);
}
