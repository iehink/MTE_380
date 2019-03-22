// Motor constants
// Right motor:
int MOTOR_A_DIR = 12, MOTOR_A_BRAKE = 9, MOTOR_A_PWM = 3; // Motor pinouts
int MOTOR_A_FWD = HIGH, MOTOR_A_REV = LOW; // Motor direction constants
// Left motor:
int MOTOR_B_DIR = 13, MOTOR_B_BRAKE = 8, MOTOR_B_PWM = 11; // Motor pinouts
int MOTOR_B_REV = HIGH, MOTOR_B_FWD = LOW; // Motor direction constants

double MOTOR_A_SPEED_RATIO = 1, MOTOR_B_SPEED_RATIO = 1; //0.6; // MUST NOT BE GREATER THAN 1
// instead:
double rightMotorSpeedModifier, leftMotorSpeedModifier;

int CLOCKWISE = 1, COUNTER_CLOCKWISE = 0;
int MAX_SPEED = 250;
int TURN_SPEED = 230;


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

void Forward(int spd) {
  double spdR = spd + rightMotorSpeedModifier, spdL = spd + leftMotorSpeedModifier;
  double angleDiff = CardinalToDegrees(CURRENT_DIRECTION) - ReadYaw();

  if (angleDiff > 180) {
    angleDiff -= 360;
  }
  if (angleDiff < -180) {
    angleDiff += 360;
  }
  
  // adjust motor speed modifiers based on gyro readings
  rightMotorSpeedModifier -= angleDiff;
  leftMotorSpeedModifier += angleDiff;
  
  RightTrack(MOTOR_A_FWD, spdR);
  LeftTrack(MOTOR_B_FWD, spdL);
}

double CardinalToDegrees(int heading){ // Function to convert directional heading (NORTH, SOUTH, etc) to degrees heading manageable by gyroscope
  if (heading == NORTH) {
    return 0; 
  } else if (heading == EAST) {
    return 90;
  } else if (heading == SOUTH) {
    return 180;
  } else if (heading == WEST) {
    return 270;
  } else {
    return -1;
  }
}

bool Head(int dir) { // Function to adjust heading
  double offset = 110;

  if (Center()) {
    centering = false;
    
    if (dir == CURRENT_DIRECTION) { // catch just in case
      return true;
    }
    
    if (TurnGyro(CardinalToDegrees(dir))) {
      // Update distances
      if (CURRENT_DIRECTION == NORTH) {
        DISTANCE_NORTH -= offset;
      } else if (CURRENT_DIRECTION == EAST) {
        DISTANCE_EAST -= offset;
      } else if (CURRENT_DIRECTION == SOUTH) {
        DISTANCE_NORTH += offset;
      } else if (CURRENT_DIRECTION == WEST) {
        DISTANCE_EAST += offset;
      }
  
      if (dir == NORTH) {
        DISTANCE_NORTH += offset;
      } else if (dir == EAST) {
        DISTANCE_EAST += offset;
      } else if (dir == SOUTH) {
        DISTANCE_NORTH -= offset;
      } else if (dir == WEST) {
        DISTANCE_EAST -= offset;
      }
      
      CURRENT_DIRECTION = dir; 
      time_last_called = millis();    
      return true;
    }
  } else {
    centering = true;
  }
  
  return false;
}

void InitMotors() {
  // Initialize motors
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_A_BRAKE, OUTPUT);
  pinMode(MOTOR_B_BRAKE, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);

  leftMotorSpeedModifier = 0;
  rightMotorSpeedModifier = 0;
}

void Move() { // Function to act on the current state of global variables
  if (turn_right) {
    TurnRight(TURN_SPEED);
  } else if (turn_left) {
    TurnLeft (TURN_SPEED);
  } else if (forward) {
    Forward (MAX_SPEED);
  } else if (reverse) {
    Reverse (MAX_SPEED);
  } else {
    Stop();
  }
}

void Reverse(int spd){
  double spdR = spd + rightMotorSpeedModifier, spdL = spd + leftMotorSpeedModifier;
  double angleDiff = CardinalToDegrees(CURRENT_DIRECTION) - ReadYaw();

  if (angleDiff > 180) {
    angleDiff -= 360;
  }
  if (angleDiff < -180) {
    angleDiff += 360;
  }
  
  // adjust motor speed modifiers based on gyro readings
  rightMotorSpeedModifier += angleDiff;
  leftMotorSpeedModifier -= angleDiff;
  
  RightTrack(MOTOR_A_REV, spdR);
  LeftTrack(MOTOR_B_REV, spdL);
}

void Stop(){
  Brake(MOTOR_A_BRAKE, true);
  Brake(MOTOR_B_BRAKE, true);
}

bool TurnGyro (double heading) { // Takes degrees heading (with North as 0 degrees, heading CW, e.g. east is 90) and sets turning accordingly
  turning = true;
  // Correct input if out of 360 deg heading
  if (heading > 360) {
    heading -= 360;
  } else if (heading < -360) {
    heading += 360;
  }
  
  double angleDiff = heading - ReadYaw();
  double TOL = 2;

  if (angleDiff > 180) {
    angleDiff -= 360;
  } else if (angleDiff < -180) {
    angleDiff += 360;
  }

  if (angleDiff > 0) {
    turn_right = true;
  } else {
    turn_left = true;
  }

  if (angleDiff > -TOL && angleDiff < TOL) {
    Stop();
    turn_right = false;
    turn_left = false;
    time_last_called = millis(); // Reset timer when a turn is completed
    turning = false;
    return true;
  }

  return false;
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
  
  if (digitalRead(MOTOR_A_DIR) != dir) { // If we're changing directions, we need to stop first
    Brake(MOTOR_A_BRAKE, true);
    digitalWrite(MOTOR_A_DIR, dir);
  }
  
  Brake(MOTOR_A_BRAKE, false);
  analogWrite(MOTOR_A_PWM, spd);
}

void LeftTrack(int dir, int spd){
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }  

  spd = int(spd * MOTOR_B_SPEED_RATIO);

  if (digitalRead(MOTOR_B_DIR) != dir) { // If we're changing directions, we need to stop first
    Brake(MOTOR_B_BRAKE, true);
    digitalWrite(MOTOR_B_DIR, dir);
  }
  
  Brake(MOTOR_B_BRAKE, false);
  analogWrite(MOTOR_B_PWM, spd);
}
