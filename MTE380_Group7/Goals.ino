// Define goal array and goal meanings
bool GOAL[6] = {false, false, false, false, false, false}; // 0th array unused, array indices correspond to values listed below
int PEOPLE = 1, LOST = 2, FOOD = 3, FIRE = 4, DELIVER = 5, POSSIBILITY = 6, STRUCTURE = 7, NONE = 8; 
// Variable to keep track of where the people are
struct Tile* peopleTile;

// Indicators for tile identification
double WATER_THRESHOLD_ANGLE = 55, NOT_FLAT_THRESHOLD_ANGLE = 30, BUMP_ANGLE = 10; // degrees
int notFlat;
bool sandOrGravel;

// Scanning globals
double SIZE_ID_DIST = FRONT_TO_NOSE + 80; // Distance at which to begin scanning [mm]
double right_scan_limit = 0, left_scan_limit = 0;
bool scanning_complete = false, scanning = false;
int scan_off_count = 0, scan_count = 0;
int left_scan_off_count = 0, front_scan_off_count = 0, right_scan_off_count = 0;
int scan_state = 0;
int scan_dir = 0; // 1 = right, 2 = left
double object_size = 0, object_dist = 0;
long unsigned int init_time_scan = millis();

#define FAN_PWM 6
#define WALL_TOL 100

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * **************************************************** Goal-handling functions are below. ****************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

void AssignGoal(double objSize) {
  // uses distance, size to assign goal to a tile
}

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
  /*if ((*CURRENT_TILE).goal == POSSIBILITY) {
    LookForGoal();
  }*/

  return completedGoals;
}

/*
bool LookForGoal(){ // Function to search the current tile for any goal and identify what goal is. Returns TRUE if it found a goal and updates goal of tile as suitable. #TODO
  if (!scanning && front_dist > SIZE_ID_DIST) {
    forward = true;
    return false;
  } else {
    forward = false;
    
    if (!scanning && Fiyah()) {
      (*CURRENT_TILE).goal = FIRE;
      return true;
    } else {
      ScanSize();
      if (scanning_complete) {
        if (object_size > 20) {
          (*CURRENT_TILE).goal = PEOPLE;
        } else if (object_size > 10) {
          (*CURRENT_TILE).goal = LOST;
        } else {
          (*CURRENT_TILE).goal = NONE;
        }

        Serial.println(object_size); 
        // Reset scanning globals
        scanning = false;
        scanning_complete = false; 
        object_size = 0;

        return true;
      }
    }
  }
}

void ScanSize() { // deprecated if we can't turn for size
  double heading = ReadYaw(), distance = front_dist, expectedDist = SIZE_ID_DIST / cos(PI/180*heading);
  double TOL = 20; // [mm] tolerance on distance as compared to expected
  scanning = true;

  if (scan_state == 0) {
    if (abs(distance - expectedDist) < TOL) {
      TurnGyro(heading + 1);
    } else {
      turn_right = false;
      right_scan_limit = abs(heading - CardinalToDegrees(CURRENT_DIRECTION));      
      if (right_scan_limit > 180) {
        right_scan_limit = abs(right_scan_limit - 360);
      }
      scan_state = 1;
    }
  } else if (scan_state == 1) {
    if (TurnGyro(CardinalToDegrees(CURRENT_DIRECTION))){
      scan_state = 2;
    }
  } else if (scan_state == 2) {
    if (abs(distance - expectedDist) < TOL) {
      TurnGyro(heading - 1);
    } else {
      turn_left = false;
      left_scan_limit = abs(heading - CardinalToDegrees(CURRENT_DIRECTION));
      if (left_scan_limit > 180) {
        left_scan_limit = abs(left_scan_limit - 360);
      }
      scan_state = 3;
    }
  } else if (scan_state == 3) {
    if (TurnGyro(CardinalToDegrees(CURRENT_DIRECTION))){
      scan_state = 4;
    }
  }  else if (scan_state == 4) {
    object_size = SIZE_ID_DIST * (tan(PI/180*left_scan_limit) + tan(PI/180*right_scan_limit));
    scanning_complete = true;
    scan_state = 0;
  } else {
    Serial.println("Error!");
    scan_state = 0;
  }
}
*/

int IDGoal(double objLen) {
  double LEN_TOL = 15;
  
  if (abs(objLen - BIG_STRUCT_LEN) < LEN_TOL) {
    return PEOPLE;
  } else if (abs(objLen - SMALL_STRUCT_LEN) < LEN_TOL) {
    return LOST;
  } else if (abs(objLen - STRUCT_WIDTH) < LEN_TOL) {
    if (GOAL[PEOPLE]) {
      return LOST;
    } else if (GOAL[LOST]) {
      return PEOPLE;
    } else {
      return STRUCTURE;
    }
  } else {
    return NONE;
  }
}

double FindLength() { // Uses left TOF to assess size of object being passed
  double TOL = 50, objLen = 0; 
  int SCAN_TOL = 5;

  if (scan_state == 0) {
    /*if (abs(right_dist - right_to_wall) > TOL) {
      if (scan_dir == 2) {
        scan_count = 0;
      }
      scan_dir = 1;
      scan_count++;
      if (scan_count > SCAN_TOL) {
        scan_count = 0;
        scan_state = 1;
        scanning = true;
      }
    } else*/ 
    if (left_dist != -1 && abs(left_dist - left_to_wall) > TOL) {
      /*if (scan_dir == 1) {
        scan_count = 0;
      }*/
      //scan_dir = 2;
      //scan_count++;
      //if (scan_count > SCAN_TOL) {
        scan_count = 0;
        scan_state = 2;
        scanning = true;
        init_time_scan = millis();
        object_dist = left_dist;
      //}
    } else {
      scan_count = 0;
    }
  } else if (scan_state == 1) {
    if (scan_off_count > SCAN_TOL) {
      objLen = scan_count * LOOP_RUNTIME / TIME_PER_MM;
      scan_count = 0;
      scan_off_count = 0;
      scan_state = 0;
      scanning_complete = true;
      return objLen;
    } else if (abs(right_dist - right_to_wall) < TOL) {
      scan_off_count++;
    }
    scan_count ++;
  } else if (scan_state == 2) {
    scan_count++;
    if (scan_off_count > SCAN_TOL) {
      objLen = (millis() - init_time_scan) / TIME_PER_MM;
      scan_count = 0;
      scan_off_count = 0;
      scan_state = 0;
      scanning_complete = true;
      return objLen;
    } else if (abs(left_dist - object_dist) > TOL || left_dist == -1) {
      scan_off_count++;
    } else {
      object_dist = (scan_count - 1.0)/(double)scan_count * object_dist + left_dist/(double)scan_count;
    }
    if (scan_count <= scan_off_count) {
      scan_count = 0;
      scan_off_count = 0;
      scan_state = 0;
      object_dist = 0;
    }
  }
  return -1;
}

// Function to search a sand tile for food. Returns TRUE (and acknowledges food) if food is found. #TODO
bool SearchSand(){
  return false;
}

void InitFan() {
  pinMode(FAN_PWM, OUTPUT);
}

void RunFan(int fanSpeed) {
  analogWrite(FAN_PWM, fanSpeed);
}

bool ObjectOnTile() {
  if (left_dist < left_to_wall - WALL_TOL || left_dist > left_to_wall + WALL_TOL) left_scan_off_count++;
  else left_scan_off_count = 0;
  if (front_dist < front_to_wall - WALL_TOL || front_dist > front_to_wall + WALL_TOL) front_scan_off_count++;
  else front_scan_off_count = 0;
  if (right_dist < right_to_wall - WALL_TOL || right_dist > right_to_wall + WALL_TOL) right_scan_off_count++;
  else right_scan_off_count = 0;

  if (left_scan_off_count > 7) {
    //Serial.println((int)(left_dist/300.0));
    /*
    if (CURRENT_DIRECTION == NORTH) {
      COURSE[(*CURRENT_TILE).row - (int)(left_dist/300.0)][(*CURRENT_TILE).col].goal = 1;
    } else if (CURRENT_DIRECTION == EAST) {
      COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col + (int)(left_dist/300.0)].goal = 1;
    } else if (CURRENT_DIRECTION == SOUTH) {
      COURSE[(*CURRENT_TILE).row + (int)(left_dist/300.0)][(*CURRENT_TILE).col].goal = 1;
    } else if (CURRENT_DIRECTION == WEST) {
      COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col - (int)(left_dist/300.0)].goal = 1;
    }*/
  }
  if (front_scan_off_count > 7) {
    //Serial.println((int)(front_dist/300.0));
  }
  if (right_scan_off_count > 7) {
    //Serial.println((int)(right_dist/300.0));
  }
}
