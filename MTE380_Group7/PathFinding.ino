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
  forward = false;
  turn_left = false;
  turn_right = false;
  return true;
  double distN = 0, distE = 0;
  bool dirSatisfied = false;

  distN = 0.5 * TILE_DISTANCE - DISTANCE_NORTH;
  distE = 0.5 * TILE_DISTANCE - DISTANCE_EAST;

  // Adjust heading if one direction has been satisfied
  if (distN == 0 && (CURRENT_DIRECTION == NORTH || CURRENT_DIRECTION == SOUTH)) {
    if (distE > 0) {
      if (Head(EAST)) {
        dirSatisfied = true;
      }
    } else if (distE < 0) {
      if (Head(WEST)) {
        dirSatisfied = true;
      }
    } else {
      Stop();
    }
  } else if (distE == 0 && (CURRENT_DIRECTION == EAST || CURRENT_DIRECTION == WEST)) {
    if (distN > 0) {
      if (Head(NORTH)) {
        dirSatisfied = true;
      }
    } else if (distN < 0) {
      if (Head(SOUTH)) {
        dirSatisfied = true;
      }
    } else {
      Stop();
    }
  }

  // Determine what way to head to be centered
  if (dirSatisfied) {
    if (CURRENT_DIRECTION == NORTH) {
      if (distN > 0) {
        Forward(MAX_SPEED);
      } else if (distN < 0) {
        Reverse(MAX_SPEED);
      }
    } else if (CURRENT_DIRECTION == SOUTH) {
      if (distN < 0) {
        Forward(MAX_SPEED);
      } else if (distN > 0) {
        Reverse(MAX_SPEED);
      }
    } else if (CURRENT_DIRECTION == EAST) {
      if (distE > 0) {
        Forward(MAX_SPEED);
      } else if (distE < 0) {
        Reverse(MAX_SPEED);
      }
    } else if (CURRENT_DIRECTION == WEST) {
      if (distE < 0) {
        Forward(MAX_SPEED);
      } else if (distE > 0) {
        Reverse(MAX_SPEED);
      }
    }
  }

  // Are we centered?
  if (distN == 0 && distE == 0) {
    return true;
  } else {
    return false;
  }
}

int Navigate() { // Checks to verify we are on the right path towards the next pathpoint. Returns direction to head, 0 if we need to center, -1 if there is no path.
  if (PATH_HEAD == NULL) {
    return -1;
  }

  if (forward) {
    if (UpdateCourseLocation()) { // If we have identified a new tile, then check what we should do after this tile
      return CURRENT_DIRECTION;
    }
  }

  // Determine row/column difference; note that any given next step will be a straight line from where we presently are.
  int rowDiff = (*PATH_HEAD->tile).row - (*CURRENT_TILE).row;
  int colDiff = (*PATH_HEAD->tile).col - (*CURRENT_TILE).col;

  Serial.println(rowDiff);
  Serial.println(colDiff);

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
  //ReadEncoders(); // possibly useful if they ever actually work?
  double distanceTravelled = (millis() - time_last_called) / TIME_PER_MM;
  time_last_called = millis();

  if (CURRENT_DIRECTION == NORTH) {
    DISTANCE_NORTH += distanceTravelled;
  } else if (CURRENT_DIRECTION == EAST) {
    DISTANCE_EAST += distanceTravelled;
  } else if (CURRENT_DIRECTION == SOUTH) {
    DISTANCE_NORTH -= distanceTravelled;
  } else if (CURRENT_DIRECTION == WEST) {
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


void AdvancedPath(struct Tile* target) {
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

  Serial.println(rowDiff);
  Serial.println(colDiff);

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

  AddToPath(target);
}

/* maybe...
  // Maybe?
  int NorthPath(struct Tile* start, struct Tile* finish, struct Tile** pathArray, int* pathIndex) { // Tracks a path north from start to finish avoiding water and filling the path array from pathIndex. Returns the sum of the path chosen.
  int avoidRight = 0, avoidLeft = 0;
  int sum = 0;

  for (int x = (*start).row - 1; x > (*finish).row; x--) {
    if (COURSE[x][(*finish).col].type == WATER) { // Row first consideration
      // Add the old tile to the row traversal path to make it the turning point
       (*pathArray[*pathIndex]) = &COURSE[x-1][(*finish).col];
       pathIndex++;

      for (int i = -1; i <= 1; i++) {
        if ((*finish).col - 1 >= 0) {
          avoidLeft += COURSE[x+i][(*finish).col - 1].type;
        } else {
          avoidLeft = 999; // can't do it
        }
        if ((*finish).col + 1 <= 5) {
          avoidRight += COURSE[x+i][(*finish).col + 1].type;
        } else {
          avoidRight = 999; // can't do it
        }
      }

      if (avoidLeft < avoidRight) { // Skirt the water to the left
        sum += avoidLeft;
         (*pathArray[*pathIndex]) = &COURSE[x+1][(*finish).col - 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x][(*finish).col - 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x-1][(*finish).col - 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x-1][(*finish).col];
        (*pathIndex)++;
      } else { // Skirt the water to the right
        sum += avoidRight;
         (*pathArray[*pathIndex]) = &COURSE[x+1][(*finish).col + 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x][(*finish).col + 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x-1][(*finish).col + 1];
        (*pathIndex)++;
         (*pathArray[*pathIndex]) = &COURSE[x-1][(*finish).col];
        (*pathIndex)++;
      }

      avoidLeft = 0;
      avoidRight = 0;
    } else {
      sum += COURSE[x][(*finish).col].type;
    }
  }

  return sum;
  }
*/

void SelectPath(struct Tile* target) { // Function to determine the optimal path to a given target tile
  // If the target tile is already being targeted, leave the path alone and log to console for testing purposes #TODO: remove for production
  if ((*target).pathTarget) {
    Serial.print("Attempted to target same tile again");
    return;
  }

  // Store that the target tile is being targeted
  (*target).pathTarget = true;

  // Determine the tile it will be travelling from
  struct Tile* prevTile;
  bool avoidingTile = false; // indicator that we're going to need to avoid a tile

  if (PATH_TAIL == NULL) {
    prevTile = CURRENT_TILE;
    AddToPath(CURRENT_TILE); // Ensure alignment to center of tile before we begin travelling to the target
  } else {
    prevTile = PATH_TAIL->tile;
  }

  // Determine best route to get to the new target tile from the previous tile
  if ((*prevTile).col - (*target).col == 0 || (*prevTile).row - (*target).row == 0) { // if the target is in the same column as the previous target already or the same row
    AddToPath(target);
    return;
  } else { // determine potential corners and calculate best path
    struct Tile* corner[2];
    struct Tile* path1[11];
    struct Tile* path2[11];
    corner[0] = &COURSE[(*target).row][(*prevTile).col];
    corner[1] = &COURSE[(*prevTile).row][(*target).col];
    int total[2] = {0, 0};

    for (int i = 0; i < 2; i++) { // for both corner options
      // Break down into starting and ending row and column for loop usage
      int rowStart = (*corner[i]).row, colStart = (*corner[i]).col;
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

      int type = 0;

      //Sum path option
      for (int x = rowStart; x < rowEnd; x++) {
        type = COURSE[x][(*target).col].type;
        if (type == WATER) {
          int sideA = 0, sideB = 0;
          for (int i = 0; i < 3; i++) {
            sideA = COURSE[x + 1][(*target).col + i].type;
            sideB = COURSE[x - 1][(*target).col + i].type;
          }
          if (sideA < sideB) {

          }
        } else {
          total[i] += type;
        }
      }
      for (int y = colStart; y < colEnd; y++) {
        total[i] += COURSE[(*target).row][y].type;
      }

      // Add the corner tile type as well (omitted from previous calculations)
      total[i] += (*corner[i]).type;
    }

    // Make path choice #TODO: optimize to prefer direction it will be facing
    if (total[1] <= total[2]) {
      AddToPath(corner[1]);
    } else {
      AddToPath(corner[2]);
    }

    AddToPath(target);
    return;
  }
}
