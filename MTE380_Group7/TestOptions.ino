void Test1(){ // Test tile selection + navigation
  bool buttonPressed = false;
  SelectPath(&COURSE[0][4]);
  
  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

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
  
  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;
  
  CURRENT_DIRECTION = NORTH;
  while (DISTANCE_NORTH < len) {
    Forward(200);
    ReadEncoders();
    Serial.println(DISTANCE_NORTH);
  }

  Stop(); 
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

  Head(EAST);

  Stop(); 
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

  while(DISTANCE_EAST < len) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }

  Stop(); 
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

  Head(SOUTH);
  
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

  while(DISTANCE_NORTH > 0) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }
  
  Stop(); 
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;
  
  Head(EAST); // Verify turning left is fine
  
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;
  
  Head(WEST);

  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;
  
  while(DISTANCE_EAST > 0){
    Forward(MAX_SPEED);
    ReadEncoders();
  }
  
  Stop(); 
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  
  buttonPressed = false;

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

void Test4() {
  bool buttonPressed = false;
  double totalLength = 0;
  
  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }
  buttonPressed = false;

  while(totalLength < 210) {
    Forward(175);
    totalLength += ReadEncoderLeft();
    Serial.println(totalLength);
  }
  Stop();
}
