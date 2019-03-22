// Hall Effect pins
#define HALL_EFFECT_1 A9
#define HALL_EFFECT_2 A10
#define HALL_EFFECT_3 A11
#define HALL_EFFECT_4 A12

// TOF variables
double left_distances[10] = {-2,-2,-2,-2,-2,-2,-2,-2,-2,-2};
double left_avg = 0;
double left_diff_avg = 0;
double front_distances[10] = {-2,-2,-2,-2,-2,-2,-2,-2,-2,-2};
double front_avg = 0;
double front_diff_avg = 0;
double right_distances[10] = {-2,-2,-2,-2,-2,-2,-2,-2,-2,-2};
double right_avg = 0;
double right_diff_avg = 0;
unsigned long scan_count_sensors = 0;

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

#define FLAME_SENSOR_PIN 30

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

void InitFlame() {
  pinMode(FLAME_SENSOR_PIN, INPUT);
}

void InitHallEffect() {
  pinMode(HALL_EFFECT_1, INPUT);
  pinMode(HALL_EFFECT_2, INPUT);
  pinMode(HALL_EFFECT_3, INPUT);
  pinMode(HALL_EFFECT_4, INPUT);
}

double ReadPitch() {
  return gyro_roll;
}

double ReadRoll() {
  return gyro_pitch;
}

double ReadYaw() {
  // Convert to deg CW 
  double yaw = -gyro_yaw + (STARTING_DIRECTION - 1) * 90;
  while (yaw < 0) {
    yaw += 360;
  }
  while (yaw > 360) {
    yaw -= 360;
  }
  return yaw;
}

bool Fiyah() { // Function to return whether or not the flame sensor is picking up fiyah
  return (!digitalRead(FLAME_SENSOR_PIN));
}

void ReadMPU(){
  // Read normalized values
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  gyro_pitch = gyro_pitch + normGyro.YAxis * INTEGRATION_TIMESTEP;
  gyro_roll = gyro_roll + normGyro.XAxis * INTEGRATION_TIMESTEP;
  gyro_yaw = gyro_yaw + normGyro.ZAxis * INTEGRATION_TIMESTEP;

  Vector normAccel = mpu.readNormalizeAccel();
}

void ReadTOF() {
  left_distances[scan_count_sensors%10] = ReadDistance(lox_left, distance_left);
  front_distances[scan_count_sensors%10] = ReadDistance(lox_front, distance_front);
  right_distances[scan_count_sensors%10] = ReadDistance(lox_right, distance_right);
  scan_count_sensors++;
  if (scan_count_sensors > 9) {
    double left_sum = 0;
    bool left_oor = false;
    double front_sum = 0;
    bool front_oor = false;
    double right_sum = 0;
    bool right_oor = false;
    for (int i = 0; i <= 9; i++) {
      left_sum += left_distances[i];
      if (left_distances[i] < 0) left_oor = true;
      front_sum += front_distances[i];
      if (front_distances[i] < 0) front_oor = true;
      right_sum += right_distances[i];
      if (right_distances[i] < 0) right_oor = true;
    }
    if (left_oor) left_avg = -1;
    else left_avg = left_sum / 10.0;
    if (front_oor) front_avg = -1;
    else front_avg = front_sum / 10.0;
    if (right_oor) right_avg = -1;
    else right_avg = right_sum / 10.0;
    
    double left_error_sum = 0;
    double front_error_sum = 0;
    double right_error_sum = 0;
    for (int i = 0; i <= 9; i++) {
      left_error_sum += abs(left_distances[i] - left_avg);
      front_error_sum += abs(front_distances[i] - front_avg);
      right_error_sum += abs(right_distances[i] - right_avg);
    }
    if (left_oor) left_diff_avg = -1;
    else left_diff_avg = left_error_sum / 10.0;
    if (front_oor) front_diff_avg = -1;
    else front_diff_avg = front_error_sum / 10.0;
    if (right_oor) right_avg = -1;
    else right_diff_avg = right_error_sum / 10.0;

    left_dist = LeftDistToActual(left_avg, left_diff_avg);
    front_dist = FrontDistToActual(front_avg, front_diff_avg);
    right_dist = RightDistToActual(right_avg, right_diff_avg);

    //Serial.print("LEFT: ");
    //Serial.print(left_avg);
    //Serial.print(", ");
    //Serial.print(left_diff_avg);
    //Serial.print("; FRONT: ");
    //Serial.print(front_avg);
    //Serial.print(", ");
    //Serial.print(front_diff_avg);
    //Serial.print("; RIGHT: ");
    //Serial.print(right_avg);
    //Serial.print(", ");
    //Serial.println(right_diff_avg);
    //delay(100);
  }
}

bool ReadHallEffect(){
  bool return_val = false;

  if (analogRead(HALL_EFFECT_1) > 700.0) return_val = true;
  else if (analogRead(HALL_EFFECT_2) > 700.0) return_val = true;
  else if (analogRead(HALL_EFFECT_3) > 700.0) return_val = true;
  else if (analogRead(HALL_EFFECT_4) > 700.0) return_val = true;
  
  return return_val;
}

/* ------------------------------------------------------------------------------------------------------------------------------
 * ********************************************* Low Level Functions are below. *************************************************
 * ------------------------------------------------------------------------------------------------------------------------------
 */
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

  if (distNorthToWall > 800) distNorthToWall = -1;
  if (distEastToWall > 800) distEastToWall = -1;
  if (distSouthToWall > 800) distSouthToWall = -1;
  if (distWestToWall > 800) distWestToWall = -1;
  
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
  //left_to_wall = left_dist;
  //right_to_wall = right_dist;
}

double LeftDistToActual(double dist, double error) {
  if (dist < 30 || error > 9) return -1;
  else if (dist < 700) return dist;
  else return -1;
}

double FrontDistToActual(double dist, double error) {
  if (dist < 30 || error > 5) return -1;
  else if (dist < 450) return dist;
  else if (dist < 700) return dist * 1.1;
  else return -1;
}

double RightDistToActual(double dist, double error) {
  if (dist < 30 || error > 9) return -1;
  else if (dist < 700) return dist;
  else return -1;
}
