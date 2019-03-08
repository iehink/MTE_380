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
#include <SharpIR.h>
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
struct Tile* CURRENT_TILE; // Pointer to tile in COURSE array that we are currently on
struct Tile* STARTING_TILE; // Pointer to tile in COURSE array that we started on
int NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4; // Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH

// Define the course and tile meanings
Tile COURSE[6][6];
int UNK = 0, FLAT = 1, SAND = 2, GRAVEL = 3, WATER = 4; 

/* Path planning will be optimized by: 
 * 1) Taking the fewest number of turns (since that is the most prone to throw our trajectory off), and
 * 2) Prioritizing unknown (UNK) tiles - the sum of each route will be taken, and the lowest score will be preferable.
 * Therefore, weightings for desire to avoid tiles/find tiles should be taken into account for the global variable definitions of tile types.
 */
struct PathPoint* PATH_HEAD = NULL;
struct PathPoint* PATH_TAIL = NULL;

double DISTANCE_NORTH, DISTANCE_EAST; // Distance based on center of nose of robot, as measured from the south-west corner of the current tile [mm].
double TILE_DISTANCE = 304.8; // length of each tile (1 ft = 304.8mm) #TODO - update with actual measurements/testing

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void setup() {
  // Begin serial comms for testing
  Serial.begin(9600);

  InitMotors();

  InitEncoders();

  InitAccelerometer();
  
  // Set up COURSE matrix
  for (int x = 0; x < 6; x++) {
    for (int y = 0; y < 6; y++) {
      COURSE[x][y].row = x+1;
      COURSE[x][y].col = y+1;
    }
  }
  
  // Define starting position #TODO - update to actual expected starting position
  STARTING_TILE = &COURSE[3][3];
  CURRENT_TILE = STARTING_TILE;
  CURRENT_DIRECTION = EAST;
  DISTANCE_NORTH = 150;
  DISTANCE_EAST = 200;
}

void loop() {
  struct PathPoint* testPoint = (struct PathPoint*)malloc(sizeof(struct PathPoint));
  
  // Variables to keep track of expected distance measurements to be received from the IR sensors
  double leftIRDist = 0, rightIRDist = 0;

  while(true){
    Test1();
  }

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
  }
}
