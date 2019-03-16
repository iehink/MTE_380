// Encoder constants
// Pinouts - must be 2, 3, 18, 19, 20, or 21 (viable pins for interrupts)
#define ENCODER_LEFT_PIN 18
#define ENCODER_RIGHT_PIN 19
// [mm/encoder pulse] #TODO - determine actual ratios
#define ENCODER_LEFT_RATIO 7.9
#define ENCODER_RIGHT_RATIO 7.2
int gyro_pitch, gyro_roll, gyro_yaw;
int previous_MPU_interrupt_time;
int ENCODER_LEFT, ENCODER_RIGHT; // To track when the encoders receive pulses

#define MPU_INTERRUPT_PIN 18

// Distance sensors
#define LOX_LEFT_ADDRESS 0x30
#define LOX_FRONT_ADDRESS 0x31
#define LOX_RIGHT_ADDRESS 0x32

#define SHT_LOX_LEFT 8
#define SHT_LOX_FRONT 7
#define SHT_LOX_RIGHT 6

// objects for the vl53l0x
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();

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

  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(true);
  mpu.setIntFreeFallEnabled(false);

  mpu.setMotionDetectionThreshold(3);
  mpu.setMotionDetectionDuration(100);
  
  
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  gyro_pitch = 0, gyro_roll = 0, gyro_yaw = 0;
  pinMode(MPU_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), MPU_ISR, FALLING);
  previous_MPU_interrupt_time = millis();
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

  // initing LOX_LEFT
  if(!lox_left.begin(LOX_LEFT_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
  }
  delay(10);

  // activating LOX_FRONT
  digitalWrite(SHT_LOX_FRONT, HIGH);
  delay(10);

  //initing LOX_FRONT
  if(!lox_front.begin(LOX_FRONT_ADDRESS)) {
    Serial.println(F("Failed to boot front VL53L0X"));
  }
  delay(10);

  // activating LOX_RIGHT
  digitalWrite(SHT_LOX_FRONT, HIGH);
  delay(10);

  //initing LOX_RIGHT
  if(!lox_right.begin(LOX_RIGHT_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
  }
}

void InitEncoders() {
  // Attach interrupts and define encoder starting values
  ENCODER_LEFT = 0;
  pinMode(ENCODER_LEFT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), EncoderLeft_ISR, CHANGE);
  ENCODER_RIGHT = 0;
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), EncoderRight_ISR, CHANGE);
}

int ReadDistanceLeft(){
  return ReadDistance(lox_left, distance_left);
}

int ReadDistanceFront(){
  return ReadDistance(lox_front, distance_front);
}

int ReadDistanceRight(){
  return ReadDistance(lox_right, distance_right);
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
  ENCODER_LEFT = 0;
  ENCODER_RIGHT = 0;
}

double getPitch(){
  return gyro_pitch;
}

double getRoll(){
  return gyro_roll;
}

double getYaw(){
  Serial.println(previous_MPU_interrupt_time);
  Serial.println(mpu.getIntStatus());
  Serial.println((mpu.readActivites()).isInactivity);
  
  Serial.println((mpu.readActivites()).isActivity);
  return gyro_yaw;
}

/* ------------------------------------------------------------------------------------------------------------------------------
 * ********************************************* Low Level Functions are below. *************************************************
 * ------------------------------------------------------------------------------------------------------------------------------
 */
// Interrupts
void EncoderLeft_ISR(){
  ENCODER_LEFT++;
}
void EncoderRight_ISR(){
  ENCODER_RIGHT++;
}
void MPU_ISR(){
  int current_MPU_interrupt_time = millis();
  int time_step = current_MPU_interrupt_time - previous_MPU_interrupt_time;
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  gyro_pitch = gyro_pitch + norm.YAxis * time_step;
  gyro_roll = gyro_roll + norm.XAxis * time_step;
  gyro_yaw = gyro_yaw + norm.ZAxis * time_step;

  previous_MPU_interrupt_time = millis();
}

double ReadEncoderLeft(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = ENCODER_LEFT;
  ENCODER_LEFT = 0;

  // Return distance conversion
  return ENCODER_LEFT_RATIO * encoder;
}

double ReadEncoderRight(){ // Returns distance and resets encoder values
  // Store current encoder value and immediately clear it (to minimize misses of rotations)
  int encoder = ENCODER_RIGHT;
  ENCODER_RIGHT = 0;

  // Return distance conversion
  return ENCODER_RIGHT_RATIO * encoder;
}

bool ReadHallEffect(){
  Serial.println(!digitalRead(52));
  return !digitalRead(52);
}

int ReadDistance(Adafruit_VL53L0X sensor, VL53L0X_RangingMeasurementData_t measurement){
  int number_of_readings = 5;
  int sum = 0;
  for(int i = 0; i < number_of_readings; i++)
  {
    sensor.rangingTest(&measurement, false);
    if(measurement.RangeStatus != 4) {     // if not out of range
      sum += measurement.RangeMilliMeter;
    }
  }
  return sum / number_of_readings;
}

double ReadPitch() {
  return 0;
}

double ReadYaw() {
  return 0;
}

double ReadRoll() {
  return 0;
}

bool Fiyah() { // Function to return whether or not the flame sensor is picking up fiyah
  return true;
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
