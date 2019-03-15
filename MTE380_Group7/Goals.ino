// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6; //, NOTHING = 7; #TODO: implement nothing if determined useful
// Variable to keep track of where the people are
struct Tile* peopleTile;

// Indicators for tile identification
double WATER_THRESHOLD_ANGLE = 55, NOT_FLAT_THRESHOLD_ANGLE = 30, BUMP_ANGLE = 10; // degrees
int notFlat;
bool sandOrGravel;

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * **************************************************** Goal-handling functions are below. ****************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

bool IDTile(){ // Function to attempt to identify the current tile. Returns TRUE when tile is identified and identity has been stored in COURSE matrix. #TODO
  if (ReadPitch() >= WATER_THRESHOLD_ANGLE && inWater) { // if we're leaving water, note it
    inWater = false;
  }
  
  // if IMU says angle is that of descending into the pit that's supposed to be water;
  if (-ReadPitch() >= WATER_THRESHOLD_ANGLE || inWater) {
    (*CURRENT_TILE).type = WATER;
    inWater = true;
  } else if (ReadPitch() >= NOT_FLAT_THRESHOLD_ANGLE) {
    sandOrGravel = true;
    return false;
  } else if (notFlat >= 10) {
    (*CURRENT_TILE).type = GRAVEL;
  } else if (PastCenter()) {
    if (sandOrGravel) {
      (*CURRENT_TILE).type = SAND;
    } else {
      (*CURRENT_TILE).type = FLAT;
    }
  } else {
    if (ReadPitch() >= BUMP_ANGLE || ReadYaw() >= BUMP_ANGLE) {
      notFlat++;
    }
    return false;
  }

  // one of the identification conditions has been met; reset notFlat counter to zero and return true
  notFlat = 0;
  sandOrGravel = false;
  return true;
}

void InitTileID(){
  notFlat = 0;
  inWater = false;
  sandOrGravel = false;
}

bool PastCenter(){ // Function to identify if we have reached the center of the tile. Returns true if at or past center in direction travelled, false otherwise.
  if (CURRENT_DIRECTION == EAST && DISTANCE_EAST >= TILE_DISTANCE/2.0) {
    return true;
  } else if (CURRENT_DIRECTION == WEST && DISTANCE_EAST <= TILE_DISTANCE/2.0) {
    return true;
  } else if (CURRENT_DIRECTION == NORTH && DISTANCE_NORTH >= TILE_DISTANCE/2.0) {
    return true;
  } else if (CURRENT_DIRECTION == SOUTH && DISTANCE_NORTH <= TILE_DISTANCE/2.0) {
    return true;
  } else {
    return false;
  }
}

int CheckGoals(){ // Returns number of goals remaining to complete.
  int completedGoals = 0;
  
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

  // If we are on a tile identified as a possibility, look for a goal on the tile
  if ((*CURRENT_TILE).goal == POSSIBILITY) {
    LookForGoal();
  }

  return completedGoals;
}

// Function to search the current tile for any goal and identify what goal is. Returns TRUE if it found a goal and updates goal of tile as suitable. #TODO
bool LookForGoal(){
  if (Fiyah()) {
    (*CURRENT_TILE).goal = FIRE;
  }
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
