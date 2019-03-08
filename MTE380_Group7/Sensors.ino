 // Encoder constants
int ENCODER_1_PIN = 2, ENCODER_2_PIN = 18; // Pinouts - must be 2, 3, 18, 19, 20, or 21 (viable pins for interrupts)
int ENCODER_1, ENCODER_2; // To track when the encoders receive pulses
double ENCODER_1_RATIO = 2.4, ENCODER_2_RATIO = 2.4; // [mm/encoder pulse] #TODO - determine actual ratios


// IR sensors
SharpIR IRLeft(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRRight(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFront(SharpIR::GP2Y0A21YK0F, A5);
double IR_LEFT_RATIO = 1, IR_RIGHT_RATIO = 1, IR_FRONT_RATIO = 1; // [mm/value] #TODO - determine actual ratios
double IR_SENSOR_DISTANCE = 30; // distance from center of device to IR sensor

// Accelerometer
MMA8452Q accel;

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************* Sensor functions are below. *******************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void InitAccelerometer() {
  // Initialize accelerometer
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Accelerometer not connected. Please check connections and read the hookup guide.");
    while (1);
  }
}

void InitEncoders() {
  // Attach interrupts and define encoder starting values
  ENCODER_1 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), Encoder1_ISR, CHANGE);
  ENCODER_2 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), Encoder2_ISR, CHANGE);
}

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

void ResetEncoders() {
  ENCODER_1 = 0;
  ENCODER_2 = 0;
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


/*
// Scanning function to check long-range IR and add tiles to the target path if the sensors pick things up #TODO - could be optimized to find things when turning
double ScanLongIR(int IR_Pin, double ratio, double dist){
  double newDist = 0;
  double threshold = 5000; // threshold at which a change in IR reading will be considered significant enough to investigate [mm]

  newDist = analogRead(IR_Pin) * ratio;
  if (abs(newDist - dist) > threshold) {
    int numTiles, i;
    struct Tile tile;
    
    // Use a multiplier to minimize repetitiveness of the code, since one is just the opposite of the other
    if(IR_Pin == IR_LEFT_PIN) { // left IR sensor picked up something interesting
      i = 1;
    } else { // right IR sensor picked up something interesting
      i = -1;
    }

    numTiles = floor((newDist + IR_SENSOR_DISTANCE)/TILE_DISTANCE);

    // Determine where new target tile is
    if (CURRENT_DIRECTION == NORTH) { 
      tile.row = (*CURRENT_TILE).row - i*numTiles;
    } else if (CURRENT_DIRECTION == EAST) {
      tile.col = (*CURRENT_TILE).col - i*numTiles;
    } else if (CURRENT_DIRECTION == SOUTH) {
      tile.row = (*CURRENT_TILE).row + i*numTiles;
    } else {
      tile.col = (*CURRENT_TILE).col + i*numTiles;
    }

    COURSE[tile.row][tile.col].goal = POSSIBILITY;
    SelectPath(&COURSE[tile.row][tile.col]);
    
  } else {
    dist = 0.9 * dist + 0.1 * newDist; // Update expectation of sensor reading, weighted towards what it has been previously
  }

  return dist;
}
*/
