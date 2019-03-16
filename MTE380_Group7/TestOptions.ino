void Test1(){ // Test tile selection + navigation
  bool buttonPressed = false;
  SelectPath(&COURSE[0][4]);

  Button();

  while (Navigate()) {
    Serial.print(CURRENT_DIRECTION);
    Serial.print(" ");
    Serial.print((*CURRENT_TILE).row);
    Serial.print(" ");
    Serial.println((*CURRENT_TILE).col);
    //Serial.println(DISTANCE_NORTH);
    /*
    if (PATH_HEAD != NULL) {
      Serial.print((*PATH_HEAD->tile).row);
      Serial.print((*PATH_HEAD->tile).col);
      Serial.print(" ");
      if (PATH_HEAD->next != NULL) {
        Serial.print((*PATH_HEAD->next->tile).row);
        Serial.print((*PATH_HEAD->next->tile).col);
        Serial.print(" ");
        if (PATH_HEAD->next->next != NULL) {
          Serial.print((*PATH_HEAD->next->next->tile).row);
          Serial.print((*PATH_HEAD->next->next->tile).col);
          Serial.println(" ");
        }
      }
    }
    */
  }
}

void Test2() { // Test to set the robot to drive in a box (for calibrating Head and Forward functions) -> multiple button presses reqd
  DISTANCE_NORTH = 0;
  DISTANCE_EAST = 0;
  double len = 210;
  bool buttonPressed = false;

  Button();
  
  CURRENT_DIRECTION = NORTH;
  while (DISTANCE_NORTH < len) {
    Forward(200);
    ReadEncoders();
    Serial.println(DISTANCE_NORTH);
  }

  Button();

  Head(EAST);

  Button();

  while(DISTANCE_EAST < len) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }

  Button();

  Head(SOUTH);

  Button();

  while(DISTANCE_NORTH > 0) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }
  
  Button();
  
  Head(EAST); // Verify turning left is fine
  
  Button();
  
  Head(WEST);

  Button();
  
  while(DISTANCE_EAST > 0){
    Forward(MAX_SPEED);
    ReadEncoders();
  }
  
  Button();

  Head(NORTH);
  if (DISTANCE_NORTH < 0) DISTANCE_NORTH = 0; // fixing weird bug where it goes heckin negative
  if (DISTANCE_EAST < 0) DISTANCE_EAST = 0; // fixing weird bug where it goes heckin negative
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
  Serial.println(ReadDistanceFront());
}

void Button() {
  bool buttonPressed = false;

  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
}
