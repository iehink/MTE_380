// Motor constants
// Right motor:
int MOTOR_A_DIR = 12, MOTOR_A_BRAKE = 9, MOTOR_A_PWM = 3; // Motor pinouts
int MOTOR_A_FWD = LOW, MOTOR_A_REV = HIGH; // Motor direction constants
// Left motor:
int MOTOR_B_DIR = 13, MOTOR_B_BRAKE = 8, MOTOR_B_PWM = 11; // Motor pinouts
int MOTOR_B_REV = LOW, MOTOR_B_FWD = HIGH; // Motor direction constants
double MOTOR_A_SPEED_RATIO = 1, MOTOR_B_SPEED_RATIO = 0.85; // MUST NOT BE GREATER THAN 1
int CLOCKWISE = 1, COUNTER_CLOCKWISE = 0;
int MAX_SPEED = 250;
int TURN_SPEED = 200;

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

void Head(int dir) { // Function to adjust heading #TODO: improve to adjust heading based on IMU; #TODO: Update DISTANCE_NORTH and DISTANCE_EAST to reflect distance change when turning
  // Use the fact that the integer respresentation of each direction is incremented by one for each cardinal direction going CW 
  int degCW = (dir - CURRENT_DIRECTION) * 90; 
  if (degCW < 0) {
    degCW += 360;
  }

  Turn(degCW);

  CURRENT_DIRECTION = dir;
}

void InitMotors() {
  // Initialize motors
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
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
  double turnDist = (ReadEncoder1() + ReadEncoder2())/2.0;
  
  degCW = degCW%360;

  if (degCW <= 180) {
    while(turnDist < distPerDeg*degCW){
      TurnRight(TURN_SPEED);
      turnDist += (ReadEncoder1() + ReadEncoder2())/2.0;
      delay(100);
    }
  } else {
    while(turnDist < distPerDeg*(360 - degCW)){
      TurnLeft(TURN_SPEED);
      turnDist += (ReadEncoder1() + ReadEncoder2())/2.0;
      delay(100);
    }
  }

  Stop();
  // reset encoders to not mess with other functions
  ResetEncoders();
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
