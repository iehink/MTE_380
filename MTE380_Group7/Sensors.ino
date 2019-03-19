// Encoder constants
// Pinouts - must be 2, 3, 18, 19, 20, or 21 (viable pins for interrupts)
#define ENCODER_LEFT_PIN 18
#define ENCODER_RIGHT_PIN 19

double gyro_pitch, gyro_roll, gyro_yaw;
int previous_MPU_interrupt_time;
int encoder_left, encoder_right; // To track when the encoders receive pulses

// Distance sensors
#define LOX_LEFT_ADDRESS 0x30
#define LOX_FRONT_ADDRESS 0x31
#define LOX_RIGHT_ADDRESS 0x32

#define SHT_LOX_LEFT 24
#define SHT_LOX_FRONT 26
#define SHT_LOX_RIGHT 28

#define LEFT_TO_EDGE 56
#define RIGHT_TO_EDGE 60

#define INTEGRATION_TIMESTEP 0.02

// objects for the vl53l0x
Adafruit_VL53L0X_MTE380 lox_left = Adafruit_VL53L0X_MTE380();
Adafruit_VL53L0X_MTE380 lox_front = Adafruit_VL53L0X_MTE380();
Adafruit_VL53L0X_MTE380 lox_right = Adafruit_VL53L0X_MTE380();

// this holds the measurement
VL53L0X_RangingMeasurementData_t distance_left;
VL53L0X_RangingMeasurementData_t distance_front;
VL53L0X_RangingMeasurementData_t distance_right;

// MPU
MPU6050 mpu;

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************* Sensor functions are below. *******************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void InitMPU() {
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  //mpu.setSleepEnabled(false);
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  gyro_pitch = 0, gyro_roll = 0, gyro_yaw = 0;
}

void InitDistanceSensors() {
  pinMode(SHT_LOX_LEFT, OUTPUT);
  pinMode(SHT_LOX_FRONT, OUTPUT);
  pinMode(SHT_LOX_RIGHT, OUTPUT);

  digitalWrite(SHT_LOX_LEFT, LOW);
  digitalWrite(SHT_LOX_FRONT, LOW);
  digitalWrite(SHT_LOX_RIGHT, LOW);
  delay(10);
  
  // all unreset
  digitalWrite(SHT_LOX_LEFT, HIGH);
  digitalWrite(SHT_LOX_FRONT, HIGH);
  digitalWrite(SHT_LOX_RIGHT, HIGH);
  delay(10);

  // activating LOX_LEFT
  digitalWrite(SHT_LOX_LEFT, HIGH);
  digitalWrite(SHT_LOX_FRONT, LOW);
  digitalWrite(SHT_LOX_RIGHT, LOW);
  delay(10);

  // initializing LOX_LEFT
  if(!lox_left.begin(LOX_LEFT_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
  }
  delay(10);

  // activating LOX_FRONT
  digitalWrite(SHT_LOX_FRONT, HIGH);
  delay(10);

  //initializing LOX_FRONT
  if(!lox_front.begin(LOX_FRONT_ADDRESS)) {
    Serial.println(F("Failed to boot front VL53L0X"));
  }
  delay(10);

  // activating LOX_RIGHT
  digitalWrite(SHT_LOX_RIGHT, HIGH);
  delay(10);

  //initializing LOX_RIGHT
  if(!lox_right.begin(LOX_RIGHT_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
  }

  lox_left.startMeasurement();
  lox_front.startMeasurement();
  lox_right.startMeasurement();
}

void InitEncoders() {
  // Attach interrupts and define encoder starting values
  encoder_left = 0;
  pinMode(ENCODER_LEFT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), EncoderLeft_ISR, CHANGE);
  encoder_right = 0;
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), EncoderRight_ISR, CHANGE);
}

double ReadEncoders(){
  double encL = ReadEncoderLeft(), encR = ReadEncoderRight();
  double distance = (encL + encR)/2; // average what each encoder thinks

  if (abs(encL - encR) > 10) {
    Serial.println("ERROR: ENCODER DISCREPANCY");
    if (encL > encR) {
      Serial.println("ENCODER Right LAGGING");
    } else {
      Serial.println("ENCODER Left LAGGING");
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
  encoder_left = 0;
  encoder_right = 0;
}

double ReadPitch() {
  return gyro_roll;
}

double ReadRoll() {
  return gyro_pitch;
}

double ReadYaw() {
  // Convert to deg CW 
  double yaw = -gyro_yaw;
  while (yaw < 0) {
    yaw += 360;
  }
  while (yaw > 360) {
    yaw -= 360;
  }
  return yaw;
}

bool Fiyah() { // Function to return whether or not the flame sensor is picking up fiyah
  return false;
}

void ReadMPU(){
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  gyro_pitch = gyro_pitch + norm.YAxis * INTEGRATION_TIMESTEP;
  gyro_roll = gyro_roll + norm.XAxis * INTEGRATION_TIMESTEP;
  gyro_yaw = gyro_yaw + norm.ZAxis * INTEGRATION_TIMESTEP;
}

void ReadTOF() {
  left_dist = ReadDistance(lox_left, distance_left);
  right_dist = ReadDistance(lox_right, distance_right);
  front_dist = ReadDistance(lox_front, distance_front);
}

/* ------------------------------------------------------------------------------------------------------------------------------
 * ********************************************* Low Level Functions are below. *************************************************
 * ------------------------------------------------------------------------------------------------------------------------------
 */
// Interrupts
void EncoderLeft_ISR(){
  encoder_left++;
}
void EncoderRight_ISR(){
  encoder_right++;
}

double ReadEncoderLeft(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = encoder_left;
  encoder_left = 0;

  // Return distance conversion
  return ENCODER_LEFT_RATIO * encoder;
}

double ReadEncoderRight(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = encoder_right;
  encoder_right = 0;

  // Return distance conversion
  return ENCODER_RIGHT_RATIO * encoder;
}

bool ReadHallEffect(){
  Serial.println(!digitalRead(52));
  return !digitalRead(52);
}

int ReadDistance(Adafruit_VL53L0X_MTE380 sensor, VL53L0X_RangingMeasurementData_t measurement){
  if (sensor.DataReady()) {
    sensor.getData(&measurement);
    if(measurement.RangeStatus != 4) {     // if not out of range
      return measurement.RangeMilliMeter;
    }
  }
  return -1;
}

void UpdateWallDistance(){
  double distNorthToWall = ((*CURRENT_TILE).row + 1) * TILE_DISTANCE - DISTANCE_NORTH;
  double distEastToWall = (5 - (*CURRENT_TILE).col + 1) * TILE_DISTANCE - DISTANCE_EAST;
  double distWestToWall = ((*CURRENT_TILE).col) * TILE_DISTANCE + DISTANCE_EAST;
  double distSouthToWall = (5 - (*CURRENT_TILE).row) * TILE_DISTANCE + DISTANCE_NORTH;
  
  if (CURRENT_DIRECTION == NORTH) {
    front_to_wall = distNorthToWall;
    right_to_wall = distEastToWall;
    left_to_wall = distWestToWall;
  } else if (CURRENT_DIRECTION == EAST) {
    front_to_wall = distEastToWall;
    right_to_wall = distSouthToWall;
    left_to_wall = distNorthToWall;
  } else if (CURRENT_DIRECTION == SOUTH) {
    front_to_wall = distSouthToWall;
    right_to_wall = distWestToWall;
    left_to_wall = distEastToWall;
  } else if (CURRENT_DIRECTION == WEST) {
    front_to_wall = distWestToWall;
    right_to_wall = distNorthToWall;
    left_to_wall = distSouthToWall;
  }

  // Since the TOF aren't working well, 
  left_to_wall = left_dist;
  right_to_wall = right_dist;
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
