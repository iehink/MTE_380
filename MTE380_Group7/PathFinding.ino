#define CENTER_TOL 15.0

/* --------------------------------------------------------------------------------------------------------------------------------------------
 * ***************************************************** Pathfinding functions are below. *****************************************************
   --------------------------------------------------------------------------------------------------------------------------------------------
*/
void AddToPath(struct Tile* newTile) { // Function to add the next path point to the path list
  //struct PathPoint* nextTile = (struct PathPoint*) malloc(sizeof(struct PathPoint));
  PathPoint* nextTile = new PathPoint;
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

bool Center() { // Function to travel to the center of the current tile. Returns TRUE when the center has been reached.
  double distOff = 0, nosePosition = 260;

  if (CURRENT_DIRECTION == NORTH) {
    distOff = nosePosition - DISTANCE_NORTH;
  } else if (CURRENT_DIRECTION == EAST) {
    distOff = nosePosition - DISTANCE_EAST;
  } else if (CURRENT_DIRECTION == SOUTH) {
    distOff = nosePosition + DISTANCE_NORTH;
  } else if (CURRENT_DIRECTION == WEST) {
    distOff = nosePosition + DISTANCE_EAST;
  }

  if (distOff > CENTER_TOL) { // not far enough
    forward = true;
    reverse = false;
  } else if (-distOff > CENTER_TOL) { // overshot
    forward = false;
    reverse = true;
  } else {
    forward = false;
    reverse = false; 
    Move();
    return true;
  }

  Move();
  UpdateDistance();
  
  return false;
}

struct Tile* ClearPath() {
  struct Tile* tile = PATH_TAIL->tile;
  while (PATH_TAIL != NULL) {
    PathPointReached();
  }
  return tile;
}

int Navigate() { // Checks to verify we are on the right path towards the next pathpoint. Returns direction to head, 0 if we need to center, -1 if there is no path.
  if (PATH_HEAD == NULL) {
    return -1;
  }

  if (forward) {
    if (UpdateCourseLocation()) { // If we have identified a new tile, then check what we should do after this tile
      temporary_stop = true;
      //Serial.println("REEEEEEEEEEEEEEEEEEEEEEEE");
      return CURRENT_DIRECTION;
    }
  }

  // Determine row/column difference; note that any given next step will be a straight line from where we presently are.
  int rowDiff = (*PATH_HEAD->tile).row - (*CURRENT_TILE).row;
  int colDiff = (*PATH_HEAD->tile).col - (*CURRENT_TILE).col;

  // If we've reached the target tile, pop the target off and return 0 to indicate arrival
  if (rowDiff == 0 && colDiff == 0) {
    PathPointReached();
    return 0;
  }

  // Determine direction to head
  //if (rowDiff == 0) { // If we are already in the correct row and just need to go across a column
  if (colDiff < 0) {
    return WEST;
  } else if (colDiff > 0) {
    return EAST;
  }
  //} else if (colDiff == 0) { // If we are already in the correct column and just need to go up/down a row
  if (rowDiff < 0) {
    return NORTH;
  } else {
    return SOUTH;
  }
  //}
}

bool UpdateCourseLocation() { // Function to update the location on the course grid. Returns true if a new tile has been reached.
  UpdateDistance();

  // Determine if we are now on a new tile
  if (abs(DISTANCE_NORTH) > TILE_DISTANCE) {
    if (CURRENT_DIRECTION == NORTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row - 1][(*CURRENT_TILE).col];
      DISTANCE_NORTH -= TILE_DISTANCE;
      return true;
    } else if (CURRENT_DIRECTION == SOUTH) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row + 1][(*CURRENT_TILE).col];
      DISTANCE_NORTH += TILE_DISTANCE;
      return true;
    }
  } else if (abs(DISTANCE_EAST) > TILE_DISTANCE) {
    if (CURRENT_DIRECTION == EAST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col + 1];
      DISTANCE_EAST -= TILE_DISTANCE;
      return true;
    } else if (CURRENT_DIRECTION == WEST) {
      CURRENT_TILE = &COURSE[(*CURRENT_TILE).row][(*CURRENT_TILE).col - 1];
      DISTANCE_EAST += TILE_DISTANCE;
      return true;
    }
  }
  return false;
}

void UpdateDistance() { // Function to update DISTANCE_NORTH and DISTANCE_EAST as required
  double distanceTravelled = (millis() - time_last_called) / TIME_PER_MM;
  time_last_called = millis();

  int dir = CURRENT_DIRECTION;
  if (reverse) dir += 2;
  if (dir > WEST) dir -= 4;

  if (dir == NORTH) {
    DISTANCE_NORTH += distanceTravelled;
  } else if (dir == EAST) {
    DISTANCE_EAST += distanceTravelled;
  } else if (dir == SOUTH) {
    DISTANCE_NORTH -= distanceTravelled;
  } else if (dir == WEST) {
    DISTANCE_EAST -= distanceTravelled;
  } else {
    Serial.println("Error - no direction specified");
  }

  //front_to_wall -= distanceTravelled;
  UpdateWallDistance();

  return;
}

void PathPointReached() { // Function to pop the target off the list
  struct PathPoint* temp = new PathPoint;

  // Store that the tile is being removed from the path
  (*PATH_HEAD->tile).pathTarget = false;

  // Remove the tile from the list
  if (PATH_HEAD->next != NULL) {
    temp = PATH_HEAD;
    PATH_HEAD = PATH_HEAD->next;
    delete temp;
  } else {
    delete PATH_HEAD;
    PATH_HEAD = NULL;
    PATH_TAIL = NULL;
  }
}

void SelectPath(struct Tile* target) { // Select a path avoiding water tiles at all costs
  // If the target tile is already being targeted, leave the path alone and log to console for testing purposes #TODO: remove for production
  if ((*target).pathTarget) {
    Serial.print("Attempted to target same tile again");
    return;
  }

  // Store that the target tile is being targeted
  (*target).pathTarget = true;

  // Determine the tile it will be travelling from
  struct Tile* prevTile;
  if (PATH_TAIL == NULL) {
    prevTile = CURRENT_TILE;
    AddToPath(CURRENT_TILE); // Ensure alignment to center of tile before we begin travelling to the target
  } else {
    prevTile = PATH_TAIL->tile;
  }

  struct Tile* rowPathPt1[6] = {NULL};
  struct Tile* rowPathPt2[6] = {NULL};
  struct Tile* colPathPt1[6] = {NULL};
  struct Tile* colPathPt2[6] = {NULL};
  int rowIndex = 0, colIndex = 0;
  int rowFirst = 0, colFirst = 0;
  struct Tile* tile;

  int avoidLeft = 0, avoidRight = 0;

  int rowDiff = (*CURRENT_TILE).row - (*target).row;
  int colDiff = (*CURRENT_TILE).col - (*target).col;
  

  /* Path is broken down by directions that will need to be travelled
     Options are to start by traversing the row or the column, then checking for water tiles and tracking around them ensues
     Paths are split in half; traversing a row is pt2 of column first path, traversing a column is pt2 of row first path
  */

  // If you will need to go west
  if (colDiff >= 0) {

    for (int y = (*CURRENT_TILE).col; y > (*target).col; y--) {

      // ROW FIRST OPTIONS
      if (COURSE[(*CURRENT_TILE).row][y].type == WATER) {
        // Add the old tile to the row traversal path to make it the turning point
        rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y + 1];
        rowIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*CURRENT_TILE).row + 1 <= 5) {
            avoidLeft += COURSE[(*CURRENT_TILE).row + 1][y - i].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*CURRENT_TILE).row - 1 >= 0) {
            avoidRight += COURSE[(*CURRENT_TILE).row - 1][y - i].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          rowFirst += avoidLeft;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y + 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y - 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y - 1];
          rowIndex++;
        } else { // Skirt the water to the right
          rowFirst += avoidLeft;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y + 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y - 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y - 1];
          rowIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        rowFirst += COURSE[(*CURRENT_TILE).row][y].type;
      }

      // COLUMN FIRST OPTIONS
      if (COURSE[(*target).row][y].type == WATER) {
        // Add the old tile to the row traversal path to make it the turning point
        colPathPt2[colIndex] = &COURSE[(*target).row][y + 1];
        colIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*target).row + 1 <= 5) {
            avoidLeft += COURSE[(*target).row + 1][y - i].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*target).row - 1 >= 0) {
            avoidRight += COURSE[(*target).row - 1][y - i].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          colFirst += avoidLeft;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y + 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y - 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row][y - 1];
          colIndex++;
        } else { // Skirt the water to the right
          colFirst += avoidLeft;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y + 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y - 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row][y - 1];
          colIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        colFirst += COURSE[(*target).row][y].type;
      }
    }


    // Store ROW FIRST OPTION corner assuming you needed to head west
    if (COURSE[(*CURRENT_TILE).row][(*target).col].type == WATER) {
      int NSModifier = 0;
      if (rowDiff >= 0) { // we also head North) 
        NSModifier = 1;
      } else {
        NSModifier = -1;
      }

      
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][(*target).col + 1];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row][(*target).col + 1].type;
  
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col + 1];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col + 1].type;
  
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col].type;
    } else {
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][(*target).col];
      rowIndex++;
    }
    
    
  } else { // If you will need to go east

    for (int y = (*CURRENT_TILE).col; y < (*target).col; y++) {

      // ROW FIRST OPTIONS
      if (COURSE[(*CURRENT_TILE).row][y].type == WATER) {
        // Add the old tile to the row traversal path to make it the turning point
        rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y - 1];
        rowIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*CURRENT_TILE).row - 1 >= 0) {
            avoidLeft += COURSE[(*CURRENT_TILE).row - 1][y + i].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*CURRENT_TILE).row + 1 <= 5) {
            avoidRight += COURSE[(*CURRENT_TILE).row + 1][y + i].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          rowFirst += avoidLeft;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y - 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - 1][y + 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y + 1];
          rowIndex++;
        } else { // Skirt the water to the right
          rowFirst += avoidLeft;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y - 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row + 1][y + 1];
          rowIndex++;
          rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][y + 1];
          rowIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        rowFirst += COURSE[(*CURRENT_TILE).row][y].type;
      }

      // COLUMN FIRST OPTIONS
      if (COURSE[(*target).row][y].type == WATER) {
        // Add the old tile to the row traversal path to make it the turning point
        colPathPt2[colIndex] = &COURSE[(*target).row][y - 1];
        colIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*target).row - 1 >= 0) {
            avoidLeft += COURSE[(*target).row - 1][y + i].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*target).row + 1 <= 5) {
            avoidRight += COURSE[(*target).row + 1][y + i].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          colFirst += avoidLeft;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y - 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row - 1][y + 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row][y + 1];
          colIndex++;
        } else { // Skirt the water to the right
          colFirst += avoidLeft;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y - 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row + 1][y + 1];
          colIndex++;
          colPathPt2[colIndex] = &COURSE[(*target).row][y + 1];
          colIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        colFirst += COURSE[(*target).row][y].type;
      }
    }


    // Store ROW FIRST OPTION corner assuming you needed to head west
    if (COURSE[(*CURRENT_TILE).row][(*target).col].type == WATER) {
      int NSModifier = 0;
      if (rowDiff >= 0) { // we also head North) 
        NSModifier = 1;
      } else {
        NSModifier = -1;
      }

      
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][(*target).col - 1];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row][(*target).col - 1].type;
  
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col - 1];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col - 1].type;
  
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col];
      rowIndex++;
      rowFirst += COURSE[(*CURRENT_TILE).row - NSModifier][(*target).col].type;
    } else {
      rowPathPt1[rowIndex] = &COURSE[(*CURRENT_TILE).row][(*target).col];
      rowIndex++;
    }
  }
  
  // Reset counters
  rowIndex = 0;
  colIndex = 0;

  if (rowDiff >= 0) { // If you will need to go North

    for (int x = (*CURRENT_TILE).row; x > (*target).row; x--) {

      // ROW FIRST OPTION
      if (COURSE[x][(*target).col].type == WATER) { // Row first consideration
        // Add the old tile to the row traversal path to make it the turning point
        rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col];
        rowIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*target).col - 1 >= 0) {
            avoidLeft += COURSE[x + i][(*target).col - 1].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*target).col + 1 <= 5) {
            avoidRight += COURSE[x + i][(*target).col + 1].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          rowFirst += avoidLeft;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col];
          rowIndex++;
        } else { // Skirt the water to the right
          rowFirst += avoidRight;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col];
          rowIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        rowFirst += COURSE[x][(*target).col].type;
      }

      // COLUMN FIRST OPTION
      if (COURSE[x][(*CURRENT_TILE).col].type == WATER) { // Column first consideration
        // Add the old tile to the row traversal path to make it the turning point
        colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col];
        colIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*CURRENT_TILE).col - 1 >= 0) {
            avoidLeft += COURSE[x + i][(*CURRENT_TILE).col - 1].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*CURRENT_TILE).col + 1 <= 5) {
            avoidRight += COURSE[x + i][(*CURRENT_TILE).col + 1].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          colFirst += avoidLeft;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col];
          colIndex++;
        } else { // Skirt the water to the right
          colFirst += avoidRight;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col];
          colIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        colFirst += COURSE[x][(*CURRENT_TILE).col].type;
      }
    }


    // Store corner
    if (COURSE[(*target).row][(*CURRENT_TILE).col].type == WATER) {
      int EWModifier = 0;
      if (colDiff >= 0) { // we also head West 
        EWModifier = 1;
      } else {
        EWModifier = -1;
      }
      
      colPathPt1[colIndex] = &COURSE[(*target).row + 1][(*CURRENT_TILE).col];
      colIndex++;
      colFirst += COURSE[(*target).row + 1][(*CURRENT_TILE).col].type;
  
      colPathPt1[colIndex] = &COURSE[(*target).row + 1][(*CURRENT_TILE).col - EWModifier];
      colIndex++;
      colFirst += COURSE[(*target).row + 1][(*CURRENT_TILE).col - EWModifier].type;
  
      colPathPt1[colIndex] = &COURSE[(*target).row][(*CURRENT_TILE).col - EWModifier];
      colIndex++;
      colFirst += COURSE[(*target).row][(*CURRENT_TILE).col - EWModifier].type;
    } else {
      colPathPt1[colIndex] = &COURSE[(*target).row][(*CURRENT_TILE).col];
      colIndex++;
    }
    
  } else { // If you will need to go South
    for (int x = (*CURRENT_TILE).row; x < (*target).row; x++) {

      // ROW FIRST OPTION
      if (COURSE[x][(*target).col].type == WATER) { // Row first consideration
        // Add the old tile to the row traversal path to make it the turning point
        rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col];
        rowIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*target).col + 1 <= 5) {
            avoidLeft += COURSE[x - i][(*target).col - 1].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*target).col - 1 >= 0) {
            avoidRight += COURSE[x - i][(*target).col - 1].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          rowFirst += avoidLeft;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col + 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col];
          rowIndex++;
        } else { // Skirt the water to the right
          rowFirst += avoidRight;
          rowPathPt2[rowIndex] = &COURSE[x - 1][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col - 1];
          rowIndex++;
          rowPathPt2[rowIndex] = &COURSE[x + 1][(*target).col];
          rowIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        rowFirst += COURSE[x][(*target).col].type;
      }

      // COLUMN FIRST OPTION
      if (COURSE[x][(*CURRENT_TILE).col].type == WATER) { // Column first consideration
        // Add the old tile to the row traversal path to make it the turning point
        colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col];
        colIndex++;

        for (int i = -1; i <= 1; i++) {
          if ((*CURRENT_TILE).col + 1 <= 5) {
            avoidLeft += COURSE[x - i][(*CURRENT_TILE).col + 1].type;
          } else {
            avoidLeft = 999; // can't do it
          }
          if ((*CURRENT_TILE).col - 1 >= 0) {
            avoidRight += COURSE[x - i][(*CURRENT_TILE).col - 1].type;
          } else {
            avoidRight = 999; // can't do it
          }
        }

        if (avoidLeft < avoidRight) { // Skirt the water to the left
          colFirst += avoidLeft;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col + 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col];
          colIndex++;
        } else { // Skirt the water to the right
          colFirst += avoidRight;
          colPathPt1[colIndex] = &COURSE[x - 1][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col - 1];
          colIndex++;
          colPathPt1[colIndex] = &COURSE[x + 1][(*CURRENT_TILE).col];
          colIndex++;
        }

        avoidLeft = 0;
        avoidRight = 0;
      } else {
        colFirst += COURSE[x][(*CURRENT_TILE).col].type;
      }
    }

    // Store corner
    if (COURSE[(*target).row][(*CURRENT_TILE).col].type == WATER) {
      int EWModifier = 0;
      if (colDiff >= 0) { // we also head West 
        EWModifier = 1;
      } else {
        EWModifier = -1;
      }
      
      colPathPt1[colIndex] = &COURSE[(*target).row - 1][(*CURRENT_TILE).col];
      colIndex++;
      colFirst += COURSE[(*target).row - 1][(*CURRENT_TILE).col].type;
  
      colPathPt1[colIndex] = &COURSE[(*target).row - 1][(*CURRENT_TILE).col - EWModifier];
      colIndex++;
      colFirst += COURSE[(*target).row - 1][(*CURRENT_TILE).col - EWModifier].type;
  
      colPathPt1[colIndex] = &COURSE[(*target).row][(*CURRENT_TILE).col - EWModifier];
      colIndex++;
      colFirst += COURSE[(*target).row][(*CURRENT_TILE).col - EWModifier].type;
    } else {
      colPathPt1[colIndex] = &COURSE[(*target).row][(*CURRENT_TILE).col];
      colIndex++;
    }
  }

  rowIndex = 0;
  colIndex = 0;

  if (rowFirst < colFirst) {
    if (rowFirst > 50) { // this path will not work. Go to the center and try to get to your target again.
      struct Tile* oldTarget = ClearPath();
      if (oldTarget != &COURSE[2][2] && CURRENT_TILE != &COURSE[2][2]) {
        SelectPath(&COURSE[2][2]);
      } else {
        SelectPath(&COURSE[3][3]);
      }
      SelectPath(oldTarget);
    } else {
      for (int i = 0; i < 6; i++) {
        if (rowPathPt1[i] != NULL) {
          AddToPath(rowPathPt1[i]);
        }
      }
      for (int i = 0; i < 6; i++) {
        if (rowPathPt2[i] != NULL) {
          AddToPath(rowPathPt2[i]);
        }
      }
    }
  } else {
    if (colFirst > 50) { // this path will not work. Go to the center and try to get to your target again.
      struct Tile* oldTarget = ClearPath();
      if (oldTarget != &COURSE[2][2] && CURRENT_TILE != &COURSE[2][2]) {
        SelectPath(&COURSE[2][2]);
      } else {
        SelectPath(&COURSE[3][3]);
      }
      SelectPath(oldTarget);
    } else {
      for (int i = 0; i < 6; i++) {
        if (colPathPt1[i] != NULL) {
          AddToPath(colPathPt1[i]);
        }
      }
      for (int i = 0; i < 6; i++) {
        if (colPathPt2[i] != NULL) {
          AddToPath(colPathPt2[i]);
        }
      }
    }
  }

  AddToPath(target);
}
