// Includes
#include <SharpIR.h>
#include <SparkFun_MMA8452Q.h>
#include <Wire.h> // for I2C

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ********************************************************* Define global variables **********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
 // Encoder constants
int ENCODER_1_PIN, ENCODER_2_PIN; // Pinouts
int ENCODER_1, ENCODER_2; // To track when the encoders receive pulses

// IR sensors
SharpIR IRLeft(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRRight(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFront(SharpIR::GP2Y0A21YK0F, A5);

// Accelerometer
MMA8452Q accel;

// Define sensor ratios
double ENCODER_1_RATIO = 2.4, ENCODER_2_RATIO = 2.4; // [mm/encoder pulse] #TODO - determine actual ratios
double IR_LEFT_RATIO = 1, IR_RIGHT_RATIO = 1, IR_FRONT_RATIO = 1; // [mm/value] #TODO - determine actual ratios

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************* Sensor functions are below. *******************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
int ReadIRFront(){
  return ReadIR(IRFront);
}

int ReadIRLeft(){
  return ReadIR(IRLeft);
}

int ReadIRRight(){
  return ReadIR(IRRight);
}

double ReadEncoders(){
  double enc1 = ReadEncoder1(), enc2 = ReadEncoder2();
  double distance = (enc1 + enc2)/2; // average what each encoder thinks

  if (abs(enc1 - enc2) > 10) {
    Serial.println("ERROR: ENCODER DISCREPANCY");
    if (enc1 > enc2) {
      Serial.println("ENCODER 2 LAGGING");
    } else {
      Serial.println("ENCODER 1 LAGGING");
    }
  }
  
  // Update distance travelled
  if (CURRENT_DIRECTION == NORTH) {
    DISTANCE_NORTH += distance; 
  } else if (CURRENT_DIRECTION == SOUTH) {
    DISTANCE_NORTH -= distance;
  } else if (CURRENT_DIRECTION == EAST) {
    DISTANCE_EAST += distance;
  } else {
    DISTANCE_EAST -= distance;
  } 
  return distance;
}

/* ------------------------------------------------------------------------------------------------------------------------------
 * ********************************************* Low Level Functions are below. *************************************************
 * ------------------------------------------------------------------------------------------------------------------------------
 */
// Interrupts
void Encoder1_ISR(){
  ENCODER_1++;
}
void Encoder2_ISR(){
  ENCODER_2++;
}

double ReadEncoder1(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = ENCODER_1;
  ENCODER_1 = 0;

  /* Test code
  if(encoder != 0) {
    Serial.print("ENCODER 1: ");
    Serial.print(encoder * ENCODER_1_RATIO);
  }
  */
  
  // Return distance conversion
  return ENCODER_1_RATIO * encoder;
}

double ReadEncoder2(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = ENCODER_2;
  ENCODER_2 = 0;

  /* Test code
  if(encoder != 0) {
    Serial.print(" ENCODER 2 ");
    Serial.println(encoder * ENCODER_2_RATIO);
  }
  */
  
  // Return distance conversion
  return ENCODER_2_RATIO * encoder;
}

bool ReadHallEffect(){
  Serial.println(!digitalRead(52));
  return !digitalRead(52);
}

int ReadIR(SharpIR sensor){
  int number_of_readings = 5;
  int sum = 0;
  for(int i = 0; i < number_of_readings; i++)
  {
    sum += sensor.getDistance();
  }
  return sum / number_of_readings;  
}
