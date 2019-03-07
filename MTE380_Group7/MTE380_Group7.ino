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
 // Encoder constants
int ENCODER_1_PIN, ENCODER_2_PIN; // Pinouts
int ENCODER_1, ENCODER_2; // To track when the encoders receive pulses

// IR sensors
SharpIR IRLeft(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRRight(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFront(SharpIR::GP2Y0A21YK0F, A5);

// Motor constants
int MOTOR_A_DIR, MOTOR_A_BRAKE, MOTOR_A_PWM, MOTOR_B_DIR, MOTOR_B_BRAKE, MOTOR_B_PWM; // Motor pinouts; A is right, B is left
int MOTOR_A_FWD, MOTOR_A_REV, MOTOR_B_REV, MOTOR_B_FWD; // Motor direction constants
double MOTOR_A_SPEED_RATIO = 1, MOTOR_B_SPEED_RATIO = 0.85; // MUST NOT BE GREATER THAN 1
int CLOCKWISE = 1, COUNTER_CLOCKWISE = 0;
int MAX_SPEED = 250;

// Accelerometer
MMA8452Q accel;

// Pathfinding globals
int CURRENT_DIRECTION; // To track grid location/direction
struct Tile* CURRENT_TILE; // Pointer to tile in COURSE array that we are currently on
struct Tile* STARTING_TILE; // Pointer to tile in COURSE array that we started on
int NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4; // Directional constants; KEEP THESE THE SAME BECAUSE THEIR VALUES ARE USED FOR MATH

// Define the course and tile meanings
Tile COURSE[6][6];
int UNK = 0, FLAT = 1, SAND = 2, GRAVEL = 3, WATER = 4; 

// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6; //, NOTHING = 7; #TODO: implement nothing if determined useful

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
 * With the exception of setup() and loop(), functions are initialized below and alphabetized beneath setup() and loop() for easy navigation.
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void AddToPath(struct Tile* tile); // Function to add a new target to the path
void Encoder1_ISR(); // ISR to monitor encoder 1 and increment when a pulse is received
void Encoder2_ISR(); // ISR to monitor encoder 1 and increment when a pulse is received
void Forward(double spd); // Function to drive the vehicle forward at speed s [mm/s]
void Head (int dir); // Function to adjust heading (N/W/E/S)
bool IDTile(); // Function to attempt to identify the current tile. Returns TRUE when tile is identified and identity has been stored in the COURSE matrix.
bool LookForGoal(); // Function to search the current tile for any goal and identify what goal is. Returns TRUE if it found a goal and updates goal of tile as suitable.
void Navigate(); // Function to adjust path direction and speed as indicated by the path list. Heads to center of next target tile at a speed relative to its distance from the tile.
bool NewTile(); // Function to determine if a new tile has been reached
void PathPointReached(); // Function to pop the target off the list
double ScanLongIR(int IR_Pin, double ratio, double dist); // Scanning function to check long-range IR and add tiles to the target path if the sensors pick things up
bool SearchSand(); // Function to search a sand tile for food. Returns TRUE (and acknowledges food) if food is found.
void SelectPath(struct Tile* target); // Function to determine the optimal path to a given target tile
void Turn (int degCW); // Function to turn the device degCW degrees clockwise and update current direction

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************** Running code begins below. ********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void setup() {
  // Begin serial comms for testing
  Serial.begin(9600);
  
  // Define pinouts #TODO - set actual pinouts
  // Define encoder pinouts - must be 2, 3, 18, 19, 20, or 21 (viable pins for interrupts)
  ENCODER_1_PIN = 2; // left
  ENCODER_2_PIN = 18; //right

  // Define motor pinouts
  pinMode(12, OUTPUT);
  MOTOR_A_DIR = 12;
  pinMode(9, OUTPUT);
  MOTOR_A_BRAKE = 9;
  pinMode(3, OUTPUT);
  MOTOR_A_PWM = 3;
  MOTOR_A_FWD = LOW;
  MOTOR_A_REV = HIGH;
  pinMode(13, OUTPUT);
  MOTOR_B_DIR = 13;
  pinMode(8, OUTPUT);
  MOTOR_B_BRAKE = 8;
  pinMode(11, OUTPUT);
  MOTOR_B_PWM = 11;
  MOTOR_B_FWD = HIGH;
  MOTOR_B_REV = LOW;

  // Initialize motors
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);

  // Attach interrupts and define encoder starting values
  ENCODER_1 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), Encoder1_ISR, CHANGE);
  ENCODER_2 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), Encoder2_ISR, CHANGE);

  // Initialize accelerometer
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Accelerometer not connected. Please check connections and read the hookup guide.");
    while (1);
  }
  
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
  bool SQUARE_TEST = false;
  bool TESTING = true; // set to true to simulate/test, set to false when actually using sensor values
  int testX = 1, testY = 0, testZ = 0;
  struct PathPoint* testPoint = (struct PathPoint*)malloc(sizeof(struct PathPoint));
  
  // Variables to keep track of expected distance measurements to be received from the IR sensors
  double leftIRDist = 0, rightIRDist = 0;
  // Variable to keep track of number of completed goals
  int completedGoals = 0;
  // Variable to keep track of where the people are
  struct Tile* peopleTile;
  
  // testing loop
  while(TESTING){
    SelectPath(&COURSE[0][4]);
    
    Stop();
    while(testX == 1){
      if (digitalRead(4) == HIGH) {
        testX = 0;
        Serial.print("Go to ");
        Serial.print((*PATH_HEAD->tile).row);
        Serial.println((*PATH_HEAD->tile).col);
      }
    }
    
    testX = 1;

    while (PATH_HEAD != NULL) {
      ReadEncoders();
      if (NewTile()) {
        Serial.println("New tile!");
        Serial.print(CURRENT_DIRECTION);
        Serial.print(" : ");
        Serial.print((*CURRENT_TILE).row);
        Serial.print(", ");
        Serial.println((*CURRENT_TILE).col);
      }
      
      Navigate();
    }
  }

  while (SQUARE_TEST) {
    Stop();
    while(testX == 1){
      if (digitalRead(4) == HIGH) {
        testX = 0;
      }
    }
    
    testX = 1;
    
    CURRENT_DIRECTION = NORTH;
    while (DISTANCE_NORTH < 200) {
      Forward(200);
      ReadEncoders();
      Serial.println(DISTANCE_NORTH);
    }

    Head(EAST);
    Stop();

    while(DISTANCE_EAST < 100) {
      Forward(MAX_SPEED);
      ReadEncoders();
    }

    Head(SOUTH);
    Stop();

    while(DISTANCE_NORTH > 0) {
      Forward(MAX_SPEED);
      ReadEncoders();
    }

    Head(WEST);
    while(DISTANCE_EAST > 0){
      Forward(MAX_SPEED);
      ReadEncoders();
    }

    Head(NORTH);
    if (DISTANCE_NORTH < 0) DISTANCE_NORTH = 0; // fixing weird bug where it goes heckin negative
    if (DISTANCE_EAST < 0) DISTANCE_EAST = 0; // fixing weird bug where it goes heckin negative
  }
  /*
  while (TESTING) {
    for (int x = 0; x <= 5; x++) {
      for (int y = 0; y <= 5; y++) {
        if ((*CURRENT_TILE).row == x && (*CURRENT_TILE).col == y) {
          Serial.print("C");
        }
        Serial.print(COURSE[x][y].type);
        Serial.print(", ");
        Serial.print(COURSE[x][y].goal);
        Serial.print("\t");
      }
      Serial.println();
    }

    Serial.print("Input target tile (x y, e.g. 3 4 for second row third column): ");

    while (!Serial.available()){}
    testX = Serial.parseInt();
    while (!Serial.available()){}
    testY = Serial.parseInt();
    Serial.read();
    Serial.println(testX);
    Serial.println(testY);
    
    SelectPath(&COURSE[testX][testY]);
    COURSE[testX][testY].goal = POSSIBILITY;

    testPoint = PATH_HEAD;
    
    while(testPoint != PATH_TAIL) {
      Serial.print((*testPoint->tile).row);
      Serial.print(", ");
      Serial.print((*testPoint->tile).col);
      Serial.print("\t");
      testPoint = testPoint -> next;
    }
    Serial.print((*testPoint->tile).row);
    Serial.print(", ");
    Serial.print((*testPoint->tile).col);
    Serial.println();

    if (accel.available()) {      // Wait for new data from accelerometer
    // Acceleration of x, y, and z directions in g units
    Serial.print(accel.getCalculatedX(), 3);
    Serial.print("\t");
    Serial.print(accel.getCalculatedY(), 3);
    Serial.print("\t");
    Serial.print(accel.getCalculatedZ(), 3);
    Serial.println();
  }
  }
  */
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


/* --------------------------------------------------------------------------------------------------------------------------------------------
 * *********************************************************** Functions are below. ***********************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
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

void Forward(int spd){
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

void Head(int dir) { // Function to adjust heading #TODO: improve to adjust heading based on IMU
  // Use the fact that the integer respresentation of each direction is incremented by one for each cardinal direction going CW 
  int degCW = (dir - CURRENT_DIRECTION) * 90; 
  if (degCW < 0) {
    degCW += 360;
  }

  Turn(degCW);

  CURRENT_DIRECTION = dir;
}

void Reverse(int spd){
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void Stop(){
  Brake(MOTOR_A_BRAKE, true);
  Brake(MOTOR_B_BRAKE, true);
}

void Turn (int degCW) { // Function to turn the device degCW degrees clockwise and update current direction #TODO
  ReadEncoders(); // Update distance value first so we don't upset that measurement

  double distPerDeg = 4.03;
  double turnDist = ReadEncoder1()/2.0;
  
  degCW = degCW%360;

  if (degCW <= 180) {
    while(turnDist < distPerDeg*degCW){
      TurnRight(200);
      turnDist += ReadEncoder1();
      delay(100);
    }
  } else {
    while(turnDist < distPerDeg*(360 - degCW)){
      TurnLeft(200);
      turnDist += ReadEncoder1();
      delay(100);
    }
  }

  Stop();
  // reset encoders to not mess with other functions
  ENCODER_1 = 0;
  ENCODER_2 = 0;
  return;
}

void TurnLeft(int spd){
  RightTrack(MOTOR_A_FWD, spd);
  LeftTrack(MOTOR_B_REV, spd);
}

void TurnRight(int spd){
  RightTrack(MOTOR_A_REV, spd);
  LeftTrack(MOTOR_B_FWD, spd);
}

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ******************************************************* Sensor functions are below. *******************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
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

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ************************************************* Tile identification functions are below. *************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
// Function to attempt to identify the current tile. Returns TRUE when tile is identified and identity has been stored in COURSE matrix. #TODO
bool IDTile(){
  return true;
}
 
/* --------------------------------------------------------------------------------------------------------------------------------------------
 * **************************************************** Goal-handling functions are below. ****************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
// Function to search the current tile for any goal and identify what goal is. Returns TRUE if it found a goal and updates goal of tile as suitable. #TODO
bool LookForGoal(){
  /*  if (the fire sensor says there's fire) {
   *    (*CURRENT_TILE).goal = FIRE;
   *  }
   *  ...
   *  else {
   *    (*CURRENT_TILE).goal = NOTHING; // Maybe?
   *  }
   *  etc.
   */
}

// Function to search a sand tile for food. Returns TRUE (and acknowledges food) if food is found. #TODO
bool SearchSand(){
  return false;
}

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ***************************************************** Pathfinding functions are below. *****************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */
void AddToPath(struct Tile* newTile){ // Function to add the next path point to the path list #TODO - could optimize to head to closer points first
  struct PathPoint* nextTile = (struct PathPoint*) malloc(sizeof(struct PathPoint));
  nextTile->tile = newTile;
  nextTile->next = NULL;
  
  if (PATH_HEAD == NULL) {
    PATH_HEAD = nextTile;
    PATH_TAIL = nextTile;
  } else { // append to end of list
    PATH_TAIL->next = nextTile;
    PATH_TAIL = nextTile;
  }
  return;
}

void Navigate(){ // Function to adjust path direction and speed as indicated by the path list. Heads to center of next target tile at a speed relative to its distance from the tile.
  double distToTile = 0, speedRatio = MAX_SPEED/TILE_DISTANCE;

  // If we have no objectives presently #TODO
  if (PATH_HEAD == NULL) {
    Stop(); // #TODO change this to whatever speed we determine is useful
    return;
  }

  // Determine row/column difference; note that any given next step will be a straight line from where we presently are.
  int rowDiff = (*PATH_HEAD->tile).row - (*CURRENT_TILE).row;
  int colDiff = (*PATH_HEAD->tile).col - (*CURRENT_TILE).col;

  // Adjust heading
   if (rowDiff == 0 && colDiff == 0) { // if we are already on the correct tile, navigate to the center
    if (CURRENT_DIRECTION == NORTH) {
      distToTile = 0.5 * TILE_DISTANCE - DISTANCE_NORTH;
    } else if (CURRENT_DIRECTION == SOUTH) {
      distToTile = 0.5 * TILE_DISTANCE + DISTANCE_NORTH;
    } else if (CURRENT_DIRECTION == EAST) {
      distToTile = 0.5 * TILE_DISTANCE - DISTANCE_EAST;
    } else if (CURRENT_DIRECTION == WEST) {
      distToTile = 0.5 * TILE_DISTANCE + DISTANCE_EAST;
    }
    Serial.println(distToTile);
    if (abs(distToTile) < 10) {
      PathPointReached();
    }
   } else if (rowDiff == 0) { // If we are already in the correct row and just need to go across a column
    if (colDiff < 0) {
      Head(WEST);
      colDiff = -1 * colDiff;
      distToTile = DISTANCE_EAST;
    } else {
      Head(EAST);
      distToTile = (TILE_DISTANCE - DISTANCE_EAST);
    }
    distToTile += (colDiff - 0.5) * TILE_DISTANCE;
  } else if (colDiff == 0) { // If we are already in the correct column and just need to go up/down a row
    if (rowDiff < 0) {
      Head(SOUTH);
      rowDiff = -1 * rowDiff;
      distToTile = DISTANCE_NORTH;
    } else {
      Head(NORTH);
      distToTile = (TILE_DISTANCE - DISTANCE_NORTH);
    }
    distToTile += (rowDiff - 0.5) * TILE_DISTANCE;
  }


  // Head toward target at speed relative to the distance from the target tile
  //Forward(int(distToTile * speedRatio));
  Forward(MAX_SPEED);
  return;
}

bool NewTile(){ // Function to determine if a new tile has been reached

  // Determine if we are now on a new tile
  if (abs(DISTANCE_NORTH) > TILE_DISTANCE) {
    if (CURRENT_DIRECTION == NORTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col + 1];
      DISTANCE_NORTH -= TILE_DISTANCE;
    } else if (CURRENT_DIRECTION == SOUTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col - 1];
      DISTANCE_NORTH += TILE_DISTANCE;
    }
  } else if (abs(DISTANCE_EAST) > TILE_DISTANCE) {
    if (CURRENT_DIRECTION == EAST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row + 1][(*CURRENT_TILE).col];
      DISTANCE_EAST -= TILE_DISTANCE;
    } else if (CURRENT_DIRECTION == WEST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row - 1][(*CURRENT_TILE).col];
      DISTANCE_EAST += TILE_DISTANCE;
    }
  } else {
    return false;
  }
  return true; // we either incremented N-S or E-W, or we would've returned false at this point
}

void PathPointReached(){ // Function to pop the target off the list
  struct PathPoint* temp = new PathPoint;

  // Store that the tile is being removed from the path
  (*PATH_HEAD->tile).pathTarget = false;

  // Remove the tile from the list
  temp = PATH_HEAD;
  PATH_HEAD = PATH_HEAD->next;
  delete temp;
}

void SelectPath(struct Tile* target){ // Function to determine the optimal path to a given target tile
  // If the target tile is already being targeted, leave the path alone and log to console for testing purposes #TODO: remove for production
  if ((*target).pathTarget) {
    Serial.print("Attempted to target same tile again");
    return;
  }
  
  // Store that the target tile is being targeted
  (*target).pathTarget = true;
  
  // Determine the tile it will be travelling from
  struct Tile prevTile;
  
  if (PATH_TAIL == NULL) {
    prevTile = *CURRENT_TILE;
    AddToPath(CURRENT_TILE); // Ensure alignment to center of tile before we begin travelling to the target
  } else {
    prevTile = *PATH_TAIL->tile;
  }
  
  // Determine best route to get to the new target tile from the previous tile
  if (prevTile.col - (*target).col == 0 || prevTile.row - (*target).row == 0) { // if the target is in the same column as the previous target already or the same row
    AddToPath(target);
    return;
  } else { // determine potential corners and calculate best path
    struct Tile corner[2];
    corner[0] = COURSE[(*target).row][prevTile.col];
    corner[1] = COURSE[prevTile.row][(*target).col];
    int total[2] = {0,0};

    for (int i = 0; i < 2; i++) { // for both corner options
      // Break down into starting and ending row and column for loop usage
      int rowStart = corner[i].row, colStart = corner[i].col;
      int rowEnd = rowStart, colEnd = colStart;

      // Determine which row is less, target or corner, and assign rowStart and End accordingly
      if ((*target).row < rowStart) {
        rowStart = (*target).row;
      } else {
        rowEnd = (*target).row;
      }

      // Determine which column is less, target or corner, and assign colStart and End accordingly
      if ((*target).col < colStart) {
        colStart = (*target).col;
      } else {
        colEnd = (*target).col;
      }

      //Sum path option
      for (int x = rowStart; x < rowEnd; x++) {
         total[i] += COURSE[x][(*target).col].type;
      }
      for (int y = colStart; y < colEnd; y++) {
         total[i] += COURSE[(*target).row][y].type;
      }

      // Add the corner tile type as well (omitted from previous calculations)
      total[i] += corner[i].type;
    }

    // Make path choice #TODO: optimize to prefer direction it will be facing
    if (total[1] < total[2]) {
      AddToPath(&corner[1]);
    } else {
      AddToPath(&corner[2]);
    }

    AddToPath(target);
    return;
  }
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

void LeftTrack(int dir, int spd){
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }  

  spd = int(spd * MOTOR_B_SPEED_RATIO);

  if (digitalRead(MOTOR_B_DIR) == dir)
  {
    Brake(MOTOR_B_BRAKE, false);
    analogWrite(MOTOR_B_PWM, spd);
  }
  else // If we're changing directions, we need to stop first
  { 
    Brake(MOTOR_B_BRAKE, true);
    digitalWrite(MOTOR_B_DIR, dir);
  }
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

void RightTrack(int dir, int spd){
  if (spd > MAX_SPEED)
  {
    spd = MAX_SPEED;
  }

  spd = int(spd * MOTOR_A_SPEED_RATIO);
  
  if (digitalRead(MOTOR_A_DIR) == dir)
  {
    Brake(MOTOR_A_BRAKE, false);
    analogWrite(MOTOR_A_PWM, spd);
  }
  else // If we're changing directions, we need to stop first
  { 
    Brake(MOTOR_A_BRAKE, true);
    digitalWrite(MOTOR_A_DIR, dir);
  }
}
