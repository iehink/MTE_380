/* MTE 380 Group 7
 * Date Created: 2019/02/16
 * Author: Catherine Fowler
 * Last Updated: 2019/03/21
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
#define WATER 99 // just don't do it...

/* Path planning will be optimized by:
 * 1) Taking the fewest number of turns (since that is the most prone to throw our trajectory off), and
 * 2) Prioritizing unknown (UNK) tiles - the sum of each route will be taken, and the lowest score will be preferable.
 * Therefore, weightings for desire to avoid tiles/find tiles should be taken into account for the global variable definitions of tile types.
 */
struct PathPoint* PATH_HEAD = NULL;
struct PathPoint* PATH_TAIL = NULL;
int path_state = 0;

double DISTANCE_NORTH, DISTANCE_EAST; // Distance based on center of nose of robot, as measured from the south-west corner of the current tile [mm].
#define TILE_DISTANCE 304.8 // length of each tile (1 ft = 304.8mm) #TODO - update with actual measurements/testing
double TIME_PER_MM = 3950.0/TILE_DISTANCE; // ms/mm DO NOT DEFINE THIS - IT BREAKS EVERYTHING; set it by measuring the time it takes to traverse a tile
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
bool centering = false;
bool turning = false;

// Keep track of what state we are in
#define ANY_STATE 0
#define SEARCHING 1
#define GOAL_APPROACH 2
#define GOAL_HANDLING 3
#define RETURNING_TO_PATH 4
#define DELIVERING 5
#define FINDING_FOOD 6
#define TRAVELLING 7
#define DONE 8
int deliveryNum = 0;
int foodNum = 0;
int production_state = 0;

// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6, STRUCTURE = 7, NONE = 8; 
// Variable to keep track of where the people are
struct Tile* people_tile;
struct Tile* lost_tile; // we need to return to both since we can't tell the difference
// LEDs to indicate goals; correspond to the indices above
int LED_pin[6] = {0, 32, 34, 36, 38, 40};
bool food_sensed = false;

// Initialize functions
void InitMotors();
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

  // LEDs
  for (int i = 1; i <= 5; i++) {
    pinMode(LED_pin[i], OUTPUT);
  }

  InitFan();
  Serial.println("Fan initialized.");

  InitMotors();
  Serial.println("Motors initialized.");

  InitDistanceSensors();
  Serial.println("Distance sensors intialized.");

  InitMPU();
  Serial.println("MPU initialized.");

  InitFlame();
  Serial.println("Flame sensor initialized.");

  InitHallEffect();
  Serial.println("Hall effect sensor initialized.");

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
  DISTANCE_NORTH = 260;
  DISTANCE_EAST = 150;
}

void loop() {
  int loopStartTime = millis();

  while(!btnState) {
    Stop();
    Button();
  }

  if (!fan_on) {
    ReadMPU();
  }
  if (!GOAL[FOOD]) {
    if (ReadHallEffect()) {
      food_sensed = true;
    } else {
      food_sensed = false;
    }
  }
  
  ReadTOF();

/*
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

  */
  
  if(TEST) {
    NavToTile();
    //TestGoalSearching();
    //TravelTest();
    //DistanceTest();
    //BoxTest();
    //Test3();
    //TurnGyro(90);
    //HeadingTest();
    //Move();
  }
  else { 
    // For now, just try to approach the thing in front of you and either blow out the candle or light the correct LED
    if (production_state == 0) {
      while(!btnState) {
        Stop();
        Button();
      }
      btnState = false;
      production_state = GOAL_APPROACH;
    }
    
    ProductionLoop();
  }
  
  int delayTime = (LOOP_RUNTIME) - (millis() - loopStartTime);
  //Serial.println(delayTime);
  if (delayTime > 0) {
    delay(delayTime);
  }
}

void ProductionLoop(){ // Full code
  // If we are not in a specific state at the moment, assess what state we should be in based on goals completed
  if (production_state = ANY_STATE) {
    if (!GOAL[PEOPLE] || !GOAL[LOST] || !GOAL[FIRE]){
      production_state = SEARCHING;
    } else if (!GOAL[FOOD]) {
      production_state = FINDING_FOOD;
    } else if (!GOAL[DELIVER]) {
      production_state = DELIVERING;
    } else {
      SelectPath(STARTING_TILE);
      production_state = TRAVELLING;
    }
  }
  
  if (food_sensed) {
    (*CURRENT_TILE).goal = FOOD;
    production_state = GOAL_HANDLING;
  }
  
  if (production_state == SEARCHING) {
    SearchState();
  } else if (production_state == GOAL_APPROACH) {
    GoalApproach();
  } else if (production_state == GOAL_HANDLING) {
    GoalHandling();
  } else if (production_state == RETURNING_TO_PATH) {
    ReturningToPath();
  } else if (production_state == FINDING_FOOD) {
    FindingFood();
  } else if (production_state == DELIVERING) {
    Delivering();
  } else if (production_state == TRAVELLING) {
    Travelling();
  } else if (production_state == DONE) {
    Done();
  }
}

void SearchState() { // Navigation with searching
  if (!turning && ObjectOnTile()) {
    path_state = -1; 
  }
  
  // Run along hard-coded path if there is no path found (i.e. no objects have been found from our path)
  if (PATH_HEAD == NULL) {
    path_state++; // If we reset the path counter to -1, then we will return to the pre-programmed path. If we reached the first pre-programmed path, proceed to the next pathpoint
  }
  if (path_state == 0) {
    SelectPath(&COURSE[3][2]);
  } else if (path_state == 1) {
    SelectPath(&COURSE[2][2]);
  } else if (path_state == 2) {
    SelectPath(&COURSE[2][4]);
  } else if (path_state == 3) {
    SelectPath(&COURSE[3][4]);
  } 

  int dir = -1;
  if (!centering && !temporary_stop) dir = Navigate();
  
  if (temporary_stop) {
    forward = false;
    turn_left = false;
    turn_right = false;
    if (temporary_stop_counter > 30)
    {
      temporary_stop = false;
      temporary_stop_counter = 0;
    }
    temporary_stop_counter++;
  } else if ((*CURRENT_TILE).goal == POSSIBILITY){
    forward = false;
    turn_left = false;
    turn_right = false;
    production_state = GOAL_APPROACH;
  } else if (dir == -1) { // Path head is NULL; stop moving!
    forward = false;
    turn_left = false;
    turn_right = false;
  } else if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == 0) {
    if (Center()) centering = false; // We need to ensure that we are centered on the tile for the turn about to take place
    else centering = true;
  } else {
    turning = true;
    if (Head(dir)) turning = false;
  }
  
  Move();
}

void GoalApproach() { // As you approach a structure
  if (front_dist < FRONT_TO_NOSE + 10) {
    forward = false;
    if (Fiyah()) {
      (*CURRENT_TILE).goal = FIRE;
    } else if (GOAL[PEOPLE]) {
      (*CURRENT_TILE).goal = LOST;
    } else {
      (*CURRENT_TILE).goal = STRUCTURE;
    }
    production_state = GOAL_HANDLING;
  } else {
    forward = true;
  }
  
  Move();
}

void GoalHandling() { // If you are on a goal tile, assess which one, handle it as required (including LEDs)
  if ((*CURRENT_TILE).goal == FIRE) {
    if(Fiyah()) {
      RunFan();
    } else {
      StopFan();
      digitalWrite(LED_pin[FIRE], HIGH);
      GOAL[FIRE] = true;
      production_state = RETURNING_TO_PATH;
    }
  } else if ((*CURRENT_TILE).goal == STRUCTURE) {
    //let's arbitrarily mark the first structure as PEOPLE for now
    (*CURRENT_TILE).goal = PEOPLE;
    people_tile = CURRENT_TILE;
    digitalWrite(LED_pin[PEOPLE], HIGH);
    GOAL[PEOPLE] = true;
    production_state = RETURNING_TO_PATH;
  } else if ((*CURRENT_TILE).goal == LOST) {
    lost_tile = CURRENT_TILE;
    digitalWrite(LED_pin[LOST], HIGH);
    GOAL[LOST] = HIGH;
    production_state = RETURNING_TO_PATH;
  } else if ((*CURRENT_TILE).goal == FOOD) {
    digitalWrite(LED_pin[FOOD], HIGH);
    GOAL[FOOD] = true;
    production_state = ANY_STATE;
  }
}

void ReturningToPath() { // Go back to the center of the previous tile
  if (CURRENT_DIRECTION == NORTH) {
    CURRENT_TILE = &COURSE[(*CURRENT_TILE).row + 1][(*CURRENT_TILE).col];
  } else if (CURRENT_DIRECTION == EAST) {
    CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col + 1];
  } else if (CURRENT_DIRECTION == SOUTH) {
    CURRENT_TILE = &COURSE[(*CURRENT_TILE).row - 1][(*CURRENT_TILE).col];
  } else if (CURRENT_DIRECTION == WEST) {
    CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col - 1];
  }
  if (Center()) {
    production_state = ANY_STATE;
  }
}

void Delivering() {
  int dir = Navigate();

  if (deliveryNum == 0) {
    SelectPath(people_tile);
    deliveryNum = 1;
  } else if (deliveryNum == 2) {
    SelectPath(lost_tile);
    deliveryNum = 3;
  }

  if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == -1) { // Path head is NULL; stop moving!
    forward = false;
    turn_left = false;
    turn_right = false;
    production_state = RETURNING_TO_PATH;
    if (deliveryNum == 1) {
      deliveryNum = 2;
    } else {
      GOAL[DELIVER] = true;
      production_state = GOAL_HANDLING;
    }
  } else if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == 0) {
    forward = false;
    turn_left = false;
    turn_right = false;
  } else {
    Head(dir);
  }
  
  Move();
}

void FindingFood() {
  if (foodNum == 0) {
    SelectPath(&COURSE[1][1]);
    foodNum = 1;
  } else if (foodNum == 1) {
    SelectPath(&COURSE[2][3]);
    foodNum = 2;
  } else if (foodNum == 2) {
    SelectPath(&COURSE[4][4]);
    foodNum = 0;
  }

  production_state = TRAVELLING;
}

void Travelling() { // Navigation without searching; only occurs after all structures have been found
  int dir = Navigate();

  if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == -1) { // Path head is NULL; stop moving!
    forward = false;
    turn_left = false;
    turn_right = false;
    production_state = ANY_STATE;
  } else if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == 0) {
    forward = false;
    turn_left = false;
    turn_right = false;
  } else {
    Head(dir);
  }
  
  Move();
}

void Done() { // End state (flash lights on the spot)
  forward = false;
  turn_right = false;
  turn_left = false;

  for (int i = 1; i <= 5; i++) {
    digitalWrite(LED_pin[i], LOW);
  }

  delay(1000);

  for (int i = 1; i <= 5; i++) {
    digitalWrite(LED_pin[i], HIGH);
  }
}
