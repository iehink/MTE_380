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
