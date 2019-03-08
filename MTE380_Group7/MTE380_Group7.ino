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

bool TEST = true;
 
// Pathfinding globals
int CURRENT_DIRECTION; // To track grid location/direction
struct Tile* CURRENT_TILE; // Pointer to tile in COURSE array that we are currently on
struct Tile* STARTING_TILE; // Pointer to tile in COURSE array that we started on
int NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4; // Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH

// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6; //, NOTHING = 7; #TODO: implement nothing if determined useful

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
double IR_SENSOR_DISTANCE = 30; // distance from center of device to IR sensor

// Define sensor ratios
double ENCODER_1_RATIO = 2.4, ENCODER_2_RATIO = 2.4; // [mm/encoder pulse] #TODO - determine actual ratios
double IR_LEFT_RATIO = 1, IR_RIGHT_RATIO = 1, IR_FRONT_RATIO = 1; // [mm/value] #TODO - determine actual ratios

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

  if (TEST){
    // Add test call in here...
  }
  else{
    // Variables to keep track of expected distance measurements to be received from the IR sensors
    double leftIRDist = 0, rightIRDist = 0;
    // Variable to keep track of number of completed goals
    int completedGoals = 0;
    // Variable to keep track of where the people are
    struct Tile* peopleTile;
  
    // Production loop #TODO implement front IR scanner to handle when we're gonna hit a wall (maybe)
    while (completedGoals < 5) {
      // Scan for targets (note that ScanLongIR will update the path as required)
      //leftIRDist = ScanLongIR(IR_LEFT_PIN, IR_LEFT_RATIO, leftIRDist);
      //rightIRDist = ScanLongIR(IR_RIGHT_PIN, IR_RIGHT_RATIO, rightIRDist); 
  
      // Update distance travelled
      if (NewTile()){
        //flash LED for testing purposes #TODO
      }
  
      // Update navigation
      Navigate();
  
      // If we are on a tile identified as a possibility, look for a goal on the tile
      if ((*CURRENT_TILE).goal == POSSIBILITY) {
        LookForGoal();
      }
      
      // If we have not identified the current tile, attempt to do so now #TODO: decide if we want to continue doing this after finding food
      if((*CURRENT_TILE).type == UNK){
        IDTile();
      }
  
      // Check goals
      // If we haven't found food yet and we are on a sand tile, search for food
      if (!GOAL[FOOD] && (*CURRENT_TILE).type == SAND && SearchSand()) {
        GOAL[FOOD] = true;
        completedGoals++;
        if (GOAL[PEOPLE]){
          AddToPath(peopleTile);
        }
      }
      
      if (!GOAL[PEOPLE] && (*CURRENT_TILE).goal == PEOPLE) {
        // #TODO: Flash LED certain number of times
        GOAL[PEOPLE] = true;
        completedGoals++;
        peopleTile = CURRENT_TILE;
      }
  
      if (!GOAL[LOST] && (*CURRENT_TILE).goal == LOST) {
        // #TODO: Flash LED certain number of times
        GOAL[LOST] = true;
        completedGoals++;
      }
      
      if (!GOAL[FIRE] && (*CURRENT_TILE).goal == FIRE) {
        // #TODO: Start that fan!
        /*  while (fire sensor says there's fire) {
         *    blow it out
         *  }
         */
         GOAL[FIRE] = true;
         completedGoals++;
      }
  
      // If we haven't delivered food yet but we have found the food and are back at the people 
      // (note that finding food adds the people tile to the path if people have been found)
      if (!GOAL[DELIVER] && GOAL[FOOD] && (*CURRENT_TILE).goal == PEOPLE){
        // #TODO: Flash LED certain number of times
        GOAL[DELIVER] = true;
        completedGoals++;
      }
    }
  
    SelectPath(STARTING_TILE);
  
    while(true){
      //just keep trucking til you're home
      Navigate();
    }
  }
}
