/* MTE 380 Group 7
 * Date Created: 2019/02/16
 * Author: Catherine Fowler
 * Last Updated: 2019/02/20
 * By: Catherine Fowler
 */

// Search for #TODO for missing items

/* Grid layout begins at (1,1) in the upper lefthand corner.
 * North indicates going up a column, right across a row, etc.
 * Coordinates indicated as (RC), e.g. 12 means first row, second column.
 */

// Includes
#include <MPU6050.h>
#include <Adafruit_VL53L0X_MTE380.h>
#include <SparkFun_MMA8452Q.h>
#include <Wire.h> // for I2C

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ************************************************************** Define structs **************************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
struct Tile {
  int type = 0; // flat, sand, gravel, water, or unknown, as described below in the global variables
  int row = 0;
  int col = 0;
  int goal = 0; // possibility, people, lost person, food, fire, as described below in the global variables; for storing where goals are found
  bool pathTarget = false; // for tracking if this tile is already targeted on the current path
};

struct PathPoint {
  struct Tile* tile;
  struct PathPoint *next;
};


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ********************************************************* Define global variables **********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
// Encoder values [mm/encoder pulse] #TODO - determine actual ratios
#define ENCODER_LEFT_RATIO 7.9
#define ENCODER_RIGHT_RATIO 7.2
 
// Pathfinding globals
int CURRENT_DIRECTION; // To track grid location/direction
int STARTING_DIRECTION;
struct Tile* CURRENT_TILE; // Pointer to tile in COURSE array that we are currently on
struct Tile* STARTING_TILE; // Pointer to tile in COURSE array that we started on
// Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

// Define the course and tile meanings
Tile COURSE[6][6];
#define UNK 0
#define FLAT 1
#define SAND 2
#define GRAVEL 3
#define WATER 99

/* Path planning will be optimized by:
 * 1) Taking the fewest number of turns (since that is the most prone to throw our trajectory off), and
 * 2) Prioritizing unknown (UNK) tiles - the sum of each route will be taken, and the lowest score will be preferable.
 * Therefore, weightings for desire to avoid tiles/find tiles should be taken into account for the global variable definitions of tile types.
 */
struct PathPoint* PATH_HEAD = NULL;
struct PathPoint* PATH_TAIL = NULL;

double DISTANCE_NORTH, DISTANCE_EAST; // Distance based on center of nose of robot, as measured from the south-west corner of the current tile [mm].
#define TILE_DISTANCE 304.8 // length of each tile (1 ft = 304.8mm) #TODO - update with actual measurements/testing
double TIME_PER_MM = 3950.0/300.0; // ms/mm DO NOT DEFINE THIS - IT BREAKS EVERYTHING
unsigned long time_last_called = 0; // variable to store the last time UpdateDistance() was called for the purposes of judging distance

// Movement commands
bool turn_left = false, turn_right = false, forward = false;

// Distance sensor readings
double left_dist = -1, right_dist = -1, front_dist = -1;
double right_to_wall = 0, left_to_wall = 0, front_to_wall = 0; // The expected distance to the walls

// Structure dimensions
#define STRUCT_WIDTH 127 //mm
#define SMALL_STRUCT_LEN 63 //mm
#define BIG_STRUCT_LEN 191 //mm

#define FAN_ON_SPEED 250
#define FAN_OFF_SPEED 0
double CURRENT_FAN_SPEED = 0;

// Variable to note if we are in water or not
bool inWater;

bool fan_on = false;
int fan_on_count = 0;

#define TEST true
#define LOOP_RUNTIME 20 // milliseconds

#define FRONT_TO_NOSE 80

bool btnState = false;

bool temporary_stop = false;
int temporary_stop_counter = 0;

// Initialize functions
void InitMotors();
void InitEncoders();
void InitGyro();
void InitDistanceSensors();
void InitMPU();
void InitFlame();
void InitTileID();
double ReadPitch();
double ReadYaw();
bool PastCenter();
void ReadMPU();
void ReadTOF();
void Stop();
void Button();
void NavToTile();
void AddToPath(struct Tile* newTile);

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void setup() {
  // Begin serial comms for testing
  Serial.begin(9600);
  pinMode(4, INPUT); // Button

  InitFan();
  Serial.println("Fan initialized.");

  InitMotors();
  Serial.println("Motors initialized.");

  InitEncoders();
  Serial.println("Encoders intialized.");

  InitDistanceSensors();
  Serial.println("Distance sensors intialized.");

  InitMPU();
  Serial.println("MPU initialized.");

  InitFlame();
  Serial.println("Flame sensor initialized.");

  InitHallEffect();
  Serial.println("Hall effect sensor initialized.");

  //InitTileID();

  // Set up COURSE matrix
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      COURSE[row][col].row = row;
      COURSE[row][col].col = col;
      COURSE[row][col].type = FLAT;
    }
  }

  // Hard Code COURSE tiles
  COURSE[1][1].type = SAND;
  COURSE[2][3].type = SAND;
  COURSE[4][4].type = SAND;
  COURSE[0][3].type = GRAVEL;
  COURSE[3][5].type = GRAVEL;
  COURSE[4][1].type = GRAVEL;
  COURSE[2][0].type = WATER;
  COURSE[1][4].type = WATER;
  COURSE[5][2].type = WATER;

  // Define starting position #TODO - update to actual expected starting position
  STARTING_TILE = &COURSE[5][4];
  CURRENT_TILE = STARTING_TILE;
  STARTING_DIRECTION = NORTH;
  CURRENT_DIRECTION = STARTING_DIRECTION;
  DISTANCE_NORTH = 0;
  DISTANCE_EAST = 0;
}

void loop() {
  int loopStartTime = millis();
  //ObjectOnTile();
  //struct PathPoint* testPoint = (struct PathPoint*)malloc(sizeof(struct PathPoint));
  
  while(!btnState) {
    Stop();
    Button();
    //left_to_wall = left_dist;
    //right_to_wall = right_dist;
  }

  if (!fan_on) {
    ReadMPU();
  }
  ReadTOF();
  ReadHallEffect();

  if (Fiyah() && !fan_on) {
    RunFan();
    fan_on = true;
  } else if (fan_on_count > 200) {
    StopFan();
    fan_on = false;
    fan_on_count = 0;
  } else if (fan_on) {
    fan_on_count++;
  } 
  
  if(TEST) {
    //EncoderHighLow();
    //EncoderTurning();
    //TestStructureIDing();
    //NavToTile();
    //Serial.println(CURRENT_DIRECTION);
    //Serial.println((*CURRENT_TILE).col);
    //Serial.println((*CURRENT_TILE).col);
    //TestGoalSearching();
    //TravelTest();
    //DistanceTest();
    //BoxTest();
    //Test3();
    //TurnGyro(90);
    //HeadingTest();
    //Move();
    
    /*
    Serial.print("LEFT: ");
    Serial.print(left_dist);
    Serial.print(", FRONT: ");
    Serial.print(front_dist);
    Serial.print(", RIGHT: ");
    Serial.println(right_dist);
    */
  }
  else { /*
    // Variables to keep track of expected distance measurements to be received from the IR sensors
    double leftIRDist = 0, rightIRDist = 0;

    // Production loop #TODO implement front IR scanner to handle when we're gonna hit a wall (maybe)
    while (CheckGoals() < 5) {
      // Scan for targets (note that ScanLongIR will update the path as required)
      //leftIRDist = ScanLongIR(IR_LEFT_PIN, IR_LEFT_RATIO, leftIRDist);
      //rightIRDist = ScanLongIR(IR_RIGHT_PIN, IR_RIGHT_RATIO, rightIRDist);

      // Update navigation
      Navigate();

      // If we have not identified the current tile, attempt to do so now #TODO: decide if we want to continue doing this after finding food
      if((*CURRENT_TILE).type == UNK){
        IDTile();
      }
    }

    SelectPath(STARTING_TILE);

    while(true){
      //just keep trucking til you're home
      Navigate();
    } */
  }
  int delayTime = (LOOP_RUNTIME) - (millis() - loopStartTime);
  //Serial.println(delayTime);
  if (delayTime > 0) {
    delay(delayTime);
  }
}

void ProductionLoop(){
  /* 1. Have we identified the current tile? If no, try to.
   * 2. Check for objects
   * 3. Check path
   * 4. Check straightness of path
   *//*
  // Travel around the course as required
  Navigate();
  
  // ID tile if we don't know it yet
  if ((*CURRENT_TILE).type == 0) {
    if (IDTile()) {
      // LED for testing purposes
    }
  }

  // Check surroundings if we are not in water
  if (!inWater) {
    //Scan();
  }*/
}
