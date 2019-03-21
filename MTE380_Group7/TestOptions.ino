int state = 0;
bool startup = true;
  
void NavToTile(){ // Test tile selection + navigation
  if (state == 0) {
    //SelectPath(&COURSE[5][0]);
    //COURSE[1][2].goal = POSSIBILITY;
    SelectPath(&COURSE[0][0]);
    state = 1;
  }
  
  Serial.print((*CURRENT_TILE).row);
  Serial.print(" ");
  Serial.print((*CURRENT_TILE).col);
  
  int dir = Navigate();
  Serial.print(" ");
  Serial.println(dir);
  //Serial.println(dir);
  if (temporary_stop) {
    forward = false;
    turn_left = false;
    turn_right = false;
    if (temporary_stop_counter > 30)
    {/*
      if (front_dist != -1 && front_to_wall != -1) {
        // compare front dist to expected
        Serial.println(front_to_wall);
        Serial.println(front_dist - front_to_wall);
        if (CURRENT_DIRECTION == NORTH) {
          DISTANCE_NORTH += (front_dist - front_to_wall);
        } else if (CURRENT_DIRECTION == EAST) {
          DISTANCE_EAST += (front_dist - front_to_wall);
        } else if (CURRENT_DIRECTION == SOUTH) {
          DISTANCE_NORTH -= (front_dist - front_to_wall);
        } else if (CURRENT_DIRECTION == WEST) {
          DISTANCE_EAST -= (front_dist - front_to_wall);
        }
      }*/
      time_last_called = millis();
      temporary_stop = false;
      temporary_stop_counter = 0;
      Serial.println("STOP DONE");
      centering = true;
    }
    temporary_stop_counter++;
  } else if (centering && !startup) {
    if (Center()) {
      centering = false;
      Serial.println("CENTERING DONE");
    }
  } else if (dir == -1) {
    forward = false;
    turn_left = false;
    turn_right = false;
  } else if (dir == CURRENT_DIRECTION) {
    forward = true;
  } else {
    forward = false;
    Head(dir);
  }
  
  Move();
  startup = false;
}

void Test3() {
  int testX = 0, testY = 0, testZ = 0;
  PathPoint* testPoint;
  
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

  testX = 1;
  while (testX == 1) {
    Serial.print("Pop? 1 = yes, 0 = no ");
  
    while (!Serial.available()){}
    testX = Serial.parseInt();
    Serial.read();

    if (testX == 1) {
      PathPointReached();
    }
    
    // print all
    testPoint = PATH_HEAD;
    
    while(testPoint != NULL && testPoint != PATH_TAIL) {
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
  }
}

void IMUTest() { // Can it go straight?
  int test = 0;
  CURRENT_DIRECTION = NORTH;
  
  Button();

  while (test <= 2000) {
    Forward(250);
    test++;
    Serial.print(leftMotorSpeedModifier);
    Serial.print(" ");
    Serial.println(rightMotorSpeedModifier);
  }
}

void SimpleIMUTest() {
  Serial.print("Yaw: ");
  Serial.print(ReadYaw());
  Serial.print(" Pitch: ");
  Serial.print(ReadPitch());
  Serial.print(" Roll: ");
  Serial.println(ReadRoll());
}

void SimpleDistanceSensorTest() {
  Serial.print("Left: ");
  Serial.print(left_dist);
  Serial.print(" Front: ");
  Serial.print(front_dist);
  Serial.print(" Right: ");
  Serial.println(right_dist);
}

void DistanceTest() {
  if (DISTANCE_NORTH < 200) {
    forward = true;
    UpdateDistance();
  } else {
    forward = false;
    turn_right = false;
    turn_left = false;
  }

  Move();
}

void TravelTest() {
  UpdateDistance();
  double len = FindLength();
  forward = true;

  if (scanning_complete) {
    Serial.println(len);
    forward = false;
    btnState = false;
    scanning_complete = false;
  }
  /*if (scanning_complete) {
    Serial.println(IDGoal(len));
    Stop(); 
    btnState = false;
  }
*/
  /*
  if (!scanning) {
    Navigate();
  } else {
    forward = 1;
    if (scanning_complete) {
      IDGoal(len);
      if (scan_dir == 1) {
        if (Head(CardinalToDegrees(CURRENT_DIRECTION) + 90)) {
          forward = true;
          
        }
      } else if (scan_dir == 2) {
        if (Head(CardinalToDegrees(CURRENT_DIRECTION) - 90)) {
          
        }
      }
    }
  }
  */
  Move();
}

void HeadingTest() {
  if (state == 0) {
    if (Head(EAST)) {
      state = 1;
      btnState = false;
    }
    Move();
  } else if (state == 1) {
    if (Head(WEST)) {
      state = 2;
      btnState = false;
    }
    Move();
  } else if (state == 2) {
    if (Head(SOUTH)) {
      state = 3;
      btnState = false;
    }
    Move();
  } else if (state == 3) {
    if (Head(NORTH)) {
      state = 0;
      btnState = false;
    }
    Move();
  }
}

void StructureTest() {
  if (production_state == GOAL_HANDLING) {
    GoalHandling();
  } else if (production_state == RETURNING_TO_PATH) {
    btnState = false;
    production_state = GOAL_APPROACH;
  } else {
    GoalApproach();
  }
}

void CenterTest() {
  if (Center()) {
    DISTANCE_NORTH = 0;
    btnState = false;
  }
}


bool Button() { // Swaps btnState whenever the button is pressed
  bool temp_btnState = btnState;
  while (digitalRead(4) == HIGH) {
    if (btnState) {
      temp_btnState = false;
    } else {
      temp_btnState = true;
    }
  }
  if (temp_btnState != btnState) time_last_called = millis();
  btnState = temp_btnState;
  return btnState;
}
