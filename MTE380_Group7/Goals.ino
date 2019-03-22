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
#define WALL_TOL 120

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * **************************************************** Goal-handling functions are below. ****************************************************
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

void InitFan() {
  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, 0);
}

void RunFan() {
  analogWrite(FAN_PWM, FAN_ON_SPEED);
}

void StopFan() {
  analogWrite(FAN_PWM, FAN_OFF_SPEED);
}

bool ObjectOnTile() {
  int row = 0, col = 0;
  
  if ((left_dist < left_to_wall - WALL_TOL || left_dist > left_to_wall + WALL_TOL) && left_dist != -1) left_scan_off_count++;
  else left_scan_off_count = 0;
  if ((front_dist < front_to_wall - WALL_TOL || front_dist > front_to_wall + WALL_TOL) && front_dist != -1) front_scan_off_count++;
  else front_scan_off_count = 0;
  if ((right_dist < right_to_wall - WALL_TOL || right_dist > right_to_wall + WALL_TOL) && right_dist != -1) right_scan_off_count++;
  else right_scan_off_count = 0;


  // Front sensor
  if (front_scan_off_count > 7) {
    front_scan_off_count = 0;
    
    if (CURRENT_DIRECTION == NORTH) {
      row = (*CURRENT_TILE).row - (((int)(front_dist/300.0))+1);
      col = (*CURRENT_TILE).col;
    } else if (CURRENT_DIRECTION == EAST) {
      row = (*CURRENT_TILE).row;
      col = (*CURRENT_TILE).col + (((int)(front_dist/300.0))+1);
    } else if (CURRENT_DIRECTION == SOUTH) {
      row = (*CURRENT_TILE).row + (((int)(front_dist/300.0))+1);
      col = (*CURRENT_TILE).col;
    } else if (CURRENT_DIRECTION == WEST) {
      row = (*CURRENT_TILE).row;
      col = (*CURRENT_TILE).col - (((int)(front_dist/300.0))+1);
    }

    //Serial.print("FRONT: ");
    //Serial.println((int)(front_dist/300.0));
    
    if (COURSE[row][col].goal == 0) {
      COURSE[row][col].goal = POSSIBILITY;
      COURSE[row][col].type = WATER; // avoid running through this tile
      //if (path_state == -1) {
        struct Tile* tile = ClearPath();
        SelectPath(&COURSE[row][col]);
        //SelectPath(tile);
      /*} else {
        SelectPath(&COURSE[row][col]);
      }*/
      return true;
    }
  }


  // Right sensor
  if (right_scan_off_count > 15) {
    right_scan_off_count = 0;
    if (CURRENT_DIRECTION == NORTH) {
      if (DISTANCE_NORTH < 100) { // if we caught a reading from the previous row
        row = (*CURRENT_TILE).row + 1;
      } else {
        row = (*CURRENT_TILE).row;
      }
      col = (*CURRENT_TILE).col + (((int)(right_dist/400.0))+1);
    } else if (CURRENT_DIRECTION == EAST) {
      row = (*CURRENT_TILE).row + (((int)(right_dist/400.0))+1);
      if (DISTANCE_EAST < 100) { // if we caught a reading from the previous column
        col = (*CURRENT_TILE).col - 1;
      } else {
        col = (*CURRENT_TILE).col;
      }
    } else if (CURRENT_DIRECTION == SOUTH) {
      if (DISTANCE_NORTH > -100) { // if we caught a reading from the previous row
        row = (*CURRENT_TILE).row - 1;
      } else {
        row = (*CURRENT_TILE).row;
      }
      col = (*CURRENT_TILE).col - (((int)(right_dist/400.0))+1);
    } else if (CURRENT_DIRECTION == WEST) {
      row = (*CURRENT_TILE).row - (((int)(right_dist/400.0))+1);
      if (DISTANCE_EAST > -100) { // if we caught a reading from the previous column
        col = (*CURRENT_TILE).col + 1;
      } else {
        col = (*CURRENT_TILE).col;
      }
    }

    if (COURSE[row][col].goal == 0) {
      COURSE[row][col].goal = POSSIBILITY;
      COURSE[row][col].type = WATER; // avoid running through this tile
      
      // Clear wherever we were previously going and instead go to the object we found
      struct Tile* tile = ClearPath();
      SelectPath(&COURSE[row][col]);

      centering = true; // center first
      
      return true;
    }
  }


  // Left sensor
  if (left_scan_off_count > 15) {
    left_scan_off_count = 0;
    if (CURRENT_DIRECTION == NORTH) {
      if (DISTANCE_NORTH < 100) { // if we caught a reading from the previous row
        row = (*CURRENT_TILE).row + 1;
      } else {
        row = (*CURRENT_TILE).row;
      }
      col = (*CURRENT_TILE).col - (((int)(left_dist/400.0))+1);
    } else if (CURRENT_DIRECTION == EAST) {
      row = (*CURRENT_TILE).row - (((int)(left_dist/400.0))+1);
      if (DISTANCE_EAST < 100) { // if we caught a reading from the previous column
        col = (*CURRENT_TILE).col - 1;
      } else {
        col = (*CURRENT_TILE).col;
      }
    } else if (CURRENT_DIRECTION == SOUTH) {
      if (DISTANCE_NORTH > -100) { // if we caught a reading from the previous row
        row = (*CURRENT_TILE).row - 1;
      } else {
        row = (*CURRENT_TILE).row;
      }
      col = (*CURRENT_TILE).col + (((int)(left_dist/400.0))+1);
    } else if (CURRENT_DIRECTION == WEST) {
      row = (*CURRENT_TILE).row + (((int)(left_dist/400.0))+1);
      if (DISTANCE_EAST > -100) { // if we caught a reading from the previous column
        col = (*CURRENT_TILE).col + 1;
      } else {
        col = (*CURRENT_TILE).col;
      }
    }

    if (COURSE[row][col].goal == 0) {
      /*if (path_state != -1) { // if we were only travelling to an unnecessary tile
        ClearPath(); 
      }*/ // Might cause issues with centering on a tile
      //Serial.print(row);
      //Serial.println(col);
      COURSE[row][col].goal = POSSIBILITY;
      COURSE[row][col].type = WATER; // avoid running through this tile
      
      // Clear wherever we were previously going and instead go to the object we found
      struct Tile* tile = ClearPath();
      SelectPath(&COURSE[row][col]);
      centering = true; // center on the tile first
      //SelectPath(tile);
      
      return true;
    }
  }

  return false;
}

bool SetSandPath() {
  SelectPath(&COURSE[2][2]);
  SelectPath(&COURSE[2][4]);
  SelectPath(&COURSE[5][4]);
  SelectPath(&COURSE[5][3]);
  SelectPath(&COURSE[1][3]);
  SelectPath(&COURSE[1][0]);
  SelectPath(&COURSE[0][0]);
  SelectPath(&COURSE[0][2]);
  SelectPath(&COURSE[5][2]);
  return true;
}
