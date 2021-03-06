/* MTE 380 Group 7
   Date Created: 2019/02/16
   Author: Catherine Fowler
   Last Updated: 2019/03/21
   By: Catherine Fowler
*/

// Search for #TODO for missing items

/* Grid layout begins at (1,1) in the upper lefthand corner.
   North indicates going up a column, right across a row, etc.
   Coordinates indicated as (RC), e.g. 12 means first row, second column.
*/

// Includes
#include <MPU6050.h>
#include <Adafruit_VL53L0X_MTE380.h>
#include <Wire.h> // for I2C

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ************************************************************** Define structs **************************************************************
   --------------------------------------------------------------------------------------------------------------------------------------------
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
   --------------------------------------------------------------------------------------------------------------------------------------------
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
#define GRAVEL 8
#define WATER 99 // just don't do it...

/* Path planning will be optimized by:
   1) Taking the fewest number of turns (since that is the most prone to throw our trajectory off), and
   2) Prioritizing unknown (UNK) tiles - the sum of each route will be taken, and the lowest score will be preferable.
   Therefore, weightings for desire to avoid tiles/find tiles should be taken into account for the global variable definitions of tile types.
*/
struct PathPoint* PATH_HEAD = NULL;
struct PathPoint* PATH_TAIL = NULL;
int path_state = 0;

double DISTANCE_NORTH, DISTANCE_EAST; // Distance based on center of nose of robot, as measured from the south-west corner of the current tile [mm].
#define TILE_DISTANCE 304.8 // length of each tile (1 ft = 304.8mm) #TODO - update with actual measurements/testing
double TIME_PER_MM = 4250.0 / TILE_DISTANCE; // ms/mm DO NOT DEFINE THIS - IT BREAKS EVERYTHING; set it by measuring the time it takes to traverse a tile
unsigned long time_last_called = 0; // variable to store the last time UpdateDistance() was called for the purposes of judging distance

// Movement commands
bool turn_left = false, turn_right = false, forward = false, reverse = false;

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

// MPU variables
double gyro_pitch, gyro_roll, gyro_yaw, accel_vel, accel_dist;
int previous_MPU_interrupt_time;

bool fan_on = false;
int fan_on_count = 0;

#define LOOP_RUNTIME 20 // milliseconds

#define FRONT_TO_NOSE 80

bool btnState = false;

bool temporary_stop = false;
int temporary_stop_counter = 0;
int approach_counter = 0;
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
int fire_count = 0, structure_loop = 0;
int production_state = 0;

double approach_dist = 0;

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
bool Button();
void NavToTile();
void AddToPath(struct Tile* newTile);

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
   --------------------------------------------------------------------------------------------------------------------------------------------
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
  STARTING_TILE = &COURSE[1][0];
  CURRENT_TILE = STARTING_TILE;
  STARTING_DIRECTION = EAST;
  CURRENT_DIRECTION = STARTING_DIRECTION;
  DISTANCE_NORTH = 150;
  DISTANCE_EAST = 295;
}

void loop() {
  int loopStartTime = millis();

  if (!Button()) {
    Stop();
    InitMPU();
    CURRENT_DIRECTION = STARTING_DIRECTION;
    DISTANCE_NORTH = 150;
    DISTANCE_EAST = 295;
  } else {
    if (!fan_on) ReadMPU();
    if (!GOAL[FOOD]) {
      if (ReadHallEffect()) {
        food_sensed = true;
      } else {
        food_sensed = false;
      }
    }
    ReadTOF();

    ProductionLoop();
    //Serial.println(production_state);
    //PrintPath();
    /*if (PATH_HEAD != NULL) {
      Serial.print((*PATH_HEAD->tile).row);
      Serial.println((*PATH_HEAD->tile).col);
    } else {
      Serial.println("Empty");
    }*/
  }

  int delayTime = (LOOP_RUNTIME) - (millis() - loopStartTime);
  //Serial.println(delayTime);
  if (delayTime > 0) {
    delay(delayTime);
  }
}

void ProductionLoop() { // Full code
  // If we are not in a specific state at the moment, assess what state we should be in based on goals completed
  if (production_state == ANY_STATE) {
    if (!GOAL[PEOPLE] || !GOAL[LOST] || !GOAL[FIRE]) {
      production_state = SEARCHING;
    } else if (!GOAL[FOOD]) {
      production_state = FINDING_FOOD;
    } else if (!GOAL[DELIVER]) {
      production_state = DELIVERING;
    } else if (CURRENT_TILE == STARTING_TILE) {
      production_state = DONE;
    } else {
      SelectPath(STARTING_TILE);
      production_state = TRAVELLING;
    }
  }

  if (!GOAL[FOOD] && food_sensed) {
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
  if (!turning && ReadPitch() >= -1 && ObjectOnTile()) {
    production_state = TRAVELLING;
  }

  // Run along hard-coded path if there is no path found (i.e. no objects have been found from our path)
  if (PATH_HEAD == NULL) {
    forward = false; 
    turn_left = false;
    turn_right = false;
    reverse = false;
    SetSandPath();
  }

  // If we began centering, finish centering
  if (centering) {
    if (Center()) {
      centering = false;
    }
  } else {
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
    } else if ((*CURRENT_TILE).goal == POSSIBILITY) {
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
      if ((*CURRENT_TILE).goal == 0) {
        centering = true;
      }
    } else {
      turning = true;
      if (Head(dir)) turning = false;
    }
  
    Move();
  }
}

void GoalApproach() { // As you approach a structure
  // Store how far away you are from the candle
  if (approach_dist <= 0) {
    approach_dist = front_dist - (FRONT_TO_NOSE + 50);
  }

  // If we *EVER* see fire on approach, this is the candle!
  if (Fiyah()) {
    fire_count++;
  }
    
  if (front_dist == -1) {
    approach_counter++;
    if (approach_counter > 30) { 
      forward = false;
      approach_counter = 0;
    }
  } else if (front_dist < FRONT_TO_NOSE + 50) {
    forward = false;
    structure_loop++;

    if (structure_loop > 50) {
      if (fire_count > 0) {
        (*CURRENT_TILE).goal = FIRE;
      } else if (GOAL[PEOPLE]) {
        //let's arbitrarily mark the first structure as PEOPLE for now
        (*CURRENT_TILE).goal = LOST;
      } else {
        (*CURRENT_TILE).goal = PEOPLE;
      }

      structure_loop = 0;
      fire_count = 0;
      approach_counter = 0;

      // adjust distance travelled
      if (CURRENT_DIRECTION == NORTH) {
        DISTANCE_NORTH += approach_dist;
      } else if (CURRENT_DIRECTION == EAST) {
        DISTANCE_EAST += approach_dist;
      } else if (CURRENT_DIRECTION == SOUTH) {
        DISTANCE_NORTH -= approach_dist;
      } else if (CURRENT_DIRECTION == WEST) {
        DISTANCE_EAST -= approach_dist;
      }

      approach_dist = 0;

      production_state = GOAL_HANDLING;
    }
  } else {
    forward = true;
  }

  Move();
}

void GoalHandling() { // If you are on a goal tile, assess which one, handle it as required (including LEDs)
  if ((*CURRENT_TILE).goal == FIRE) {
    if (!fan_on) {
      RunFan();
      fan_on = true;
    } else if (fan_on_count > 200) {
      StopFan();
      fan_on = false;
      fan_on_count = 0;
      digitalWrite(LED_pin[FIRE], HIGH);
      GOAL[FIRE] = true;
      production_state = RETURNING_TO_PATH;
    } else if (fan_on) {
      fan_on_count++;
    }
  } else if ((*CURRENT_TILE).goal == PEOPLE) {
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
    food_sensed = false;
    production_state = ANY_STATE;
  }

  if (GOAL[DELIVER]) {
    digitalWrite(LED_pin[DELIVER], HIGH);
    production_state = ANY_STATE;
  }
}

void ReturningToPath() { // Go back to the center of the previous tile
  if (!centering) {
    if (CURRENT_DIRECTION == NORTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row + 1][(*CURRENT_TILE).col];
      DISTANCE_NORTH += TILE_DISTANCE;
    } else if (CURRENT_DIRECTION == EAST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col - 1];
      DISTANCE_EAST += TILE_DISTANCE;
    } else if (CURRENT_DIRECTION == SOUTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row - 1][(*CURRENT_TILE).col];
      DISTANCE_NORTH -= TILE_DISTANCE;
    } else if (CURRENT_DIRECTION == WEST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col + 1];
      DISTANCE_EAST -= TILE_DISTANCE;
    }
    centering = true;
    time_last_called = millis(); // reset this so we don't update a crazy distance
  }
  if (Center()) {
    centering = false;
    production_state = ANY_STATE;
  }
}

void Delivering() {
  int dir = Navigate();

  if (deliveryNum == 0) {
    ClearPath();
    SelectPath(lost_tile);
    SelectPath(people_tile);
    deliveryNum = 1;
  }

  if ((*CURRENT_TILE).goal == PEOPLE) {
    production_state == RETURNING_TO_PATH;
  } else if ((*CURRENT_TILE).goal == LOST) {
    production_state == RETURNING_TO_PATH;
  }

  if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else if (dir == -1) { // Path head is NULL; stop moving!
    forward = false;
    turn_left = false;
    turn_right = false;
    GOAL[DELIVER] = true;
    production_state = GOAL_HANDLING;
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

void Travelling() { // Navigation without searching
  // If we are gonna hit something, don't.
  if (!centering && !turning && front_dist < 300 && front_dist != -1) {
    ObjectOnTile(); // this should set the thing ahead of us as a possibility if needed
  }
    
  // If we began centering, finish centering
  if (centering) {
    if (Center()) {
      centering = false;
    }
  } else {
    int dir = Navigate();
  
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
    } else if ((*CURRENT_TILE).goal == POSSIBILITY) {
      forward = false;
      turn_left = false;
      turn_right = false;
      production_state = GOAL_APPROACH; 
    } else if (dir == CURRENT_DIRECTION) {
      forward = true;
    } else if (dir == -1) { // Path head is NULL; stop moving!
      forward = false;
      turn_left = false;
      turn_right = false;
      production_state = ANY_STATE;
    } else if (dir == CURRENT_DIRECTION) {
      forward = true;
    } else if (dir == 0) {
      if ((*CURRENT_TILE).goal == 0) {
        centering = true;
      }
    } else {
      Head(dir);
    }
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
  
  Move();
}
