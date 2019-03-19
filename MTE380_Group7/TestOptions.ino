int state = 0;
  
void NavToTile(){ // Test tile selection + navigation
  if (state == 0) {
    SelectPath(&COURSE[2][4]);
    state = 1;
  }

  int dir = Navigate();

  Serial.println(dir);
  if(turnLeft) {
    Serial.println("We're actually trying to do this.");
  }

  if (dir == CURRENT_DIRECTION || dir == -1) {
    Serial.println("GO FORTH, MY ROBOT");
    forward = true;
  } else if (dir == 0) {
    //Center();
  } else {
    Head(dir);
  }
  
  Move();
}

void BoxTest() { // Test to set the robot to drive in a box (for calibrating Head and Forward functions) -> multiple button presses reqd
  double len = 210;

  Serial.println(state);

  if (state == 0) {
    if (DISTANCE_NORTH < len) {
      forward = true;
      ReadEncoders();
    } else {
      state++;
    }
  } else if (state == 1) {
    if (Head(EAST)) {
      state++;
    }
  } else if (state == 2) {
    if (DISTANCE_EAST < len) {
      forward = true;
      ReadEncoders();
    } else {
      state++;
    }
  } else if (state == 3) {
    if (Head(SOUTH)) {
      state++;
    }
  } else if (state == 4) {
    if (DISTANCE_NORTH > 0) {
      forward = true;
      ReadEncoders();
    } else {
      forward = false;
      state++;
    }
  } else if (state == 5) {
    if (Head(EAST)) {
      state++;
    }
  } else if (state == 6) {
    if (Head(WEST)) {
      state++;
    }
  } else if (state == 7) {
    if (DISTANCE_EAST > 0) {
      forward = true;
      ReadEncoders();
    } else {
      forward = false;
      state++;
    }
  } else {  
    if (Head(NORTH)) {
      state = 0;
    }
    if (DISTANCE_NORTH < 0) DISTANCE_NORTH = 0; // fixing weird bug where it goes heckin negative
    if (DISTANCE_EAST < 0) DISTANCE_EAST = 0; // fixing weird bug where it goes heckin negative
  }
  
  Move();
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

void BasicEncoderTest(){
  while(true) {
    LeftTrack(0, 250);
    //RightTrack(1, 250);
  }/*
  bool buttonPressed = false;

  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }

  buttonPressed = false;
  delay(200);

  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
    Serial.print("ENC LEFT: ");
    Serial.println(ReadEncoderLeft());
    Serial.print("ENC RIGHT: ");
    Serial.println(ReadEncoderRight());
    delay(3000);
  }
 */
}

void EncoderHighLow() { // While you hold the button, encoder digital values will be printed and the wheels will turn
  Stop();
  while(digitalRead(4) == HIGH){
    Forward(170);
    Serial.print(digitalRead(ENCODER_LEFT_PIN));
    Serial.print(" ");
    Serial.println(digitalRead(ENCODER_RIGHT_PIN));
  }
}

void EncoderTest() {
  double totalLength = 0;

  Serial.println("Right Encoder");
  Serial.println(ReadEncoderRight());
  Button();

  while(totalLength < 210) {
    Forward(MAX_SPEED);
    totalLength += ReadEncoderRight();
    Serial.println(totalLength);
  }

  totalLength = 0;
  Serial.println("Left Encoder");
  Serial.println(ReadEncoderLeft());
  Button();
   
  while(totalLength < 210) {
    Forward(MAX_SPEED);
    totalLength += ReadEncoderLeft();
    Serial.println(totalLength);
  }
}

void EncoderTurning () {
  int turnIncrement = 10; // Note there is a hard limit on how fine this can be based on the encoders
  
  Button();
  Turn(90);
  Button();
  Turn(-90);

  Button(); 
  for (int i =0; i < 90; i+=turnIncrement) {
    Turn(turnIncrement);
    Serial.println(i);
  }

  Button();
  Turn(-90);
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

void TOFTest(){
  Button();
  
  while(ReadDistanceFront() > 10) {
    Forward(175);
  }
}

void SimpleTOFTest() {
  Serial.print("Time of Flight Sensors: ");
  Serial.print(ReadDistanceLeft());
  Serial.print(" ");
  Serial.print(ReadDistanceFront());
  Serial.print(" ");
  Serial.println(ReadDistanceRight());
}

void TestStructureIDing(){
  int dist = ReadDistanceFront();
  int prevDist = ReadDistanceFront();
  int degCW = 0, degCCW = 0;
  int turnDeg = 10;
  double TOL = 15;
  double expectedHyp = 0;

  Serial.println("Testing stucture identification.");
  
  Button();

  Serial.println("GO!");
  
  while(dist > FRONT_TO_NOSE + 50) {
    Forward(MAX_SPEED);
    dist = ReadDistanceFront();
  }
  
  Button();

  prevDist = ReadDistanceFront();
  
  do {
    prevDist = ReadDistanceFront();
    Turn(turnDeg);
    degCW += turnDeg;
    expectedHyp = (FRONT_TO_NOSE + 50) / cos(PI/180*degCW);
    Serial.print(ReadDistanceFront());
    Serial.print("mm away, ");
    Serial.print(degCW);
    Serial.print(" degrees, expected hypotenuse: ");
    Serial.println(expectedHyp);
    dist = ReadDistanceFront();    
  } while (dist < expectedHyp + TOL);

  Serial.println("Press button to return to facing object");
  Button();

  Turn(-degCW);

  Button();

  prevDist = ReadDistanceFront();
  dist = ReadDistanceFront();

  do {
    prevDist = ReadDistanceFront();
    Turn(-turnDeg);
    degCCW += turnDeg;
    expectedHyp = (FRONT_TO_NOSE + 50) / cos(PI/180*degCCW);
    Serial.print(ReadDistanceFront());
    Serial.print("mm away, ");
    Serial.print(degCCW);
    Serial.print(" degrees, expected hypotenuse: ");
    Serial.println(expectedHyp);
    dist = ReadDistanceFront();
  } while (dist < expectedHyp + TOL);

  Button();

  Turn(degCCW);

  Button();

  Serial.println(degCW+degCCW);
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
  Serial.print(ReadDistanceLeft());
  Serial.print(" Front: ");
  Serial.print(ReadDistanceFront());
  Serial.print(" Right: ");
  Serial.println(ReadDistanceRight());
}

void DistanceTest() {
  if (DISTANCE_NORTH < 200) {
    forward = true;
    UpdateDistance();
  } else {
    forward = false;
    turnRight = false;
    turnLeft = false;
  }

  Move();
}

void Button() { // Swaps btnState whenever the button is pressed
  while (digitalRead(4) == HIGH) {
    if (btnState) {
      btnState = false;
    } else {
      btnState = true;
    }
    time_last_called = millis();
  }
}
