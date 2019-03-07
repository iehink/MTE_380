/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ********************************************************* Define global variables **********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

// Motor constants
int MOTOR_A_DIR, MOTOR_A_BRAKE, MOTOR_A_PWM, MOTOR_B_DIR, MOTOR_B_BRAKE, MOTOR_B_PWM; // Motor pinouts; A is right, B is left
int MOTOR_A_FWD, MOTOR_A_REV, MOTOR_B_REV, MOTOR_B_FWD; // Motor direction constants
double MOTOR_A_SPEED_RATIO = 1, MOTOR_B_SPEED_RATIO = 0.85; // MUST NOT BE GREATER THAN 1
int CLOCKWISE = 1, COUNTER_CLOCKWISE = 0;
int MAX_SPEED = 250;

// Pathfinding globals
int CURRENT_DIRECTION = 1; // To track grid location/direction
int NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4; // Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ****************************************************** Movement functions are below. ******************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void Brake(int brakePin, bool brake){
  if(brake) 
  {
    digitalWrite(brakePin, HIGH);
  }
  else
  {
    digitalWrite(brakePin, LOW);
  }
}

void Forward(int spd){
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

void Head(int dir) { // Function to adjust heading #TODO: improve to adjust heading based on IMU
  // Use the fact that the integer respresentation of each direction is incremented by one for each cardinal direction going CW 
  int degCW = (dir - CURRENT_DIRECTION) * 90; 
  if (degCW < 0) {
    degCW += 360;
  }

  Turn(degCW);

  CURRENT_DIRECTION = dir;
}

void Reverse(int spd){
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void Stop(){
  Brake(MOTOR_A_BRAKE, true);
  Brake(MOTOR_B_BRAKE, true);
}

void Turn (int degCW) { // Function to turn the device degCW degrees clockwise and update current direction #TODO
  ReadEncoders(); // Update distance value first so we don't upset that measurement

  double distPerDeg = 4.03;
  double turnDist = ReadEncoder1()/2.0;
  
  degCW = degCW%360;

  if (degCW <= 180) {
    while(turnDist < distPerDeg*degCW){
      TurnRight(200);
      turnDist += ReadEncoder1();
      delay(100);
    }
  } else {
    while(turnDist < distPerDeg*(360 - degCW)){
      TurnLeft(200);
      turnDist += ReadEncoder1();
      delay(100);
    }
  }

  Stop();
  // reset encoders to not mess with other functions
  ENCODER_1 = 0;
  ENCODER_2 = 0;
  return;
}

void TurnLeft(int spd){
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void TurnRight(int spd){
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

// Low-level

void RightTrack(int dir, int spd){
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }

  spd = int(spd * MOTOR_A_SPEED_RATIO);
  
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

void LeftTrack(int dir, int spd){
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }  

  spd = int(spd * MOTOR_B_SPEED_RATIO);

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
