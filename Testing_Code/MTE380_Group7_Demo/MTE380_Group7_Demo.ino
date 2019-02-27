#include <SharpIR.h>

/* MTE 380 Group 7
 * Date Created: 2019/02/16
 * Author: Catherine Fowler
 * Last Updated: 2019/02/20
 * By: Catherine Fowler
 */

// Search for #TODO for missing items

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ********************************************************* Define global variables **********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
int ENCODER_1_PIN, ENCODER_2_PIN; // Encoder pinouts
int IR_LEFT_PIN, IR_RIGHT_PIN, IR_FRONT_PIN; // Infrared pinouts
int MOTOR_A_DIR, MOTOR_A_BRAKE, MOTOR_A_PWM, MOTOR_B_DIR, MOTOR_B_BRAKE, MOTOR_B_PWM; // Motor pinouts; A is right, B is left
int MOTOR_A_FWD, MOTOR_B_FWD; // Motor direction constants
int ENCODER_1, ENCODER_2; // To track when the encoders receive pulses
int CURRENT_DIRECTION; // To track grid location/direction
int NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4; // Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH

// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6; //, NOTHING = 7; #TODO: implement nothing if determined useful

double DISTANCE_NORTH, DISTANCE_EAST; // Distance based on center of nose of robot, as measured from the lower-left corner of the current tile [mm].
double TILE_DISTANCE = 304.8; // length of each tile (1 ft = 304.8mm) #TODO - update with actual measurements/testing
double IR_SENSOR_DISTANCE = 30; // distance from center of device to IR sensor

// Define sensor ratios
double ENCODER_1_RATIO = 1, ENCODER_2_RATIO = 1.1; // [mm/encoder pulse] #TODO - determine actual ratios
double IR_LEFT_RATIO = 1, IR_RIGHT_RATIO = 1, IR_FRONT_RATIO = 1; // [mm/value] #TODO - determine actual ratios

SharpIR IRLeft(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRRight(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFront(SharpIR::GP2Y0A21YK0F, A5);


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * With the exception of setup() and loop(), functions are initialized below and alphabetized beneath setup() and loop() for easy navigation.
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void Brake(int brakePin, bool brake);
void Encoder1_ISR(); // ISR to monitor encoder 1 and increment when a pulse is received
void Encoder2_ISR(); // ISR to monitor encoder 1 and increment when a pulse is received
void Forward(double s); // Function to drive the vehicle forward at speed s [mm/s]
void Head (int dir); // Function to adjust heading (N/W/E/S)
void Turn (int degCW); // Function to turn the device degCW degrees clockwise and update current direction
int ReadIRFront();

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

void setup() {
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
  MOTOR_A_FWD = 1;
  pinMode(13, OUTPUT);
  MOTOR_B_DIR = 13;
  pinMode(8, OUTPUT);
  MOTOR_B_BRAKE = 8;
  pinMode(11, OUTPUT);
  MOTOR_B_PWM = 11;
  MOTOR_B_FWD = 0;

  // Attach interrupts and define encoder starting values
  ENCODER_1 = 0;
  ENCODER_2 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), Encoder1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), Encoder2_ISR, FALLING);
  
  // Define starting position #TODO - update to actual expected starting position
  CURRENT_DIRECTION = EAST;
  DISTANCE_NORTH = 150;
  DISTANCE_EAST = 150;
}

void loop() {
  int distance_reading = 0;
  int testX, testY;
  // Variables to keep track of expected distance measurements to be received from the IR sensors
  double leftIRDist = 0, rightIRDist = 0;

  // loop to run both motors at a user-specified speed
  /*while (true) {
    Serial.print("Set speed of motors [%]: ");
    while (!Serial.available()){}
    testX = Serial.parseInt();
    Serial.read();
    Serial.println(testX);
    Forward(testX);
  }*/
  
  // demo loop #TODO
  while (true) {
    distance_reading = ReadIRRight();
    Serial.println(distance_reading);
    if (distance_reading < 50) { // while it's a valid value and not too close
      Brake(MOTOR_A_BRAKE, false); // release brakes if they were applied
      Brake(MOTOR_B_BRAKE, false); // release brakes if they were applied
      Forward(255);
    } else {
      Brake(MOTOR_A_BRAKE, true);
      Brake(MOTOR_B_BRAKE, true); 
    }
  }
}


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * *********************************************************** Functions are below. ***********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */


// Function to apply and release brake a specific motor/track. Inputs are the brake pin of the motor and whether or not we're applying/releasing brakes
void Brake(int brakePin, bool brake) {
  // BRAKE: Set pin HIGH to brake, set pin LOW to not
  if (!brake) {
    digitalWrite(brakePin, LOW);
  } else {
    digitalWrite(brakePin, HIGH);
  }
}

// Interrupts
void Encoder1_ISR(){
  ENCODER_1++;
}
void Encoder2_ISR(){
  ENCODER_2++;
}

// Function to drive the vehicle forward at speed s [%]. Negative speed drives it backwards. #TODO
void Forward(double s){
  // Set both motors to go forward as required
  TrackForward(MOTOR_A_PWM, s);
  TrackForward(MOTOR_B_PWM, s);
}

// Function to adjust heading #TODO: improve to adjust heading based on IMU
void Head(int dir) {
  // Use the fact that the integer respresentation of each direction is incremented by one for each cardinal direction going CW 
  int degCW = (dir - CURRENT_DIRECTION) * 90; 
  if (degCW < 0) {
    degCW += 360;
  }

  Turn(degCW);
}

// Function to drive a track forward at speed s [%]. Negative speed drives it backwards. #TODO
void TrackForward(int pwmPin, double s){
  int dir;
  int fwd, dirPin, brakePin;

  // Set variables depending on motor
  if (pwmPin == MOTOR_A_PWM) {
    fwd = MOTOR_A_FWD;
    dirPin = MOTOR_A_DIR;
    brakePin = MOTOR_A_BRAKE;
  } else if (pwmPin == MOTOR_B_PWM) {
    fwd = MOTOR_B_FWD;
    dirPin = MOTOR_B_DIR;
    brakePin = MOTOR_B_BRAKE;
  } else {
    // this is an invalid motor selection
    Serial.println("INVALID MOTOR SELECTION");
    return;
  }
  
  Brake(brakePin, false); // release brakes if they were applied
  
  // Confirm it is not told to go faster than it can
  if (s > 255) {
    s = 255;
  }
  
  // Direction: Set pin HIGH for forward, LOW for backward
  if (s > 0) {
    dir = fwd;
  } else if (s < 0) {
    dir = abs(fwd - 1);
    s = -1*s;
  } else {
    Brake(pwmPin, true);
    analogWrite(pwmPin, 0);
    return;
  }

  if (dir == digitalRead(dirPin)) {
    // Update PWM control
    analogWrite(pwmPin, s);
  } else { // If we're changing directions, we need to stop first
    Brake(brakePin, true);
    // Update direction
    digitalWrite(dirPin, dir);
  }
  return;
}

// Function to turn the device degCW degrees clockwise and update current direction #TODO
void Turn (int degCW) {
  // Note that the distances shouldn't be affected and the center of the nose should return to the same place (or the distance track should be aware where it is now). One optional soln is to disable and reenable the interrupts:
  noInterrupts(); // disable interrupts (stop encoders from keeping track)

  interrupts(); // re-enable interrupts (allow encoders to continue keeping track)
  return;
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
