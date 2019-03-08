void Test1(){
  bool buttonPressed = false;
  SelectPath(&COURSE[0][4]);

  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
      Serial.print("Go to ");
      Serial.print((*PATH_HEAD->tile).row);
      Serial.println((*PATH_HEAD->tile).col);
    }
  }

  buttonPressed = false;

  while (PATH_HEAD != NULL) {
    Navigate();
  }
}

void Test2() {
  bool buttonPressed = false;

  Stop();
  while(!buttonPressed){
    if (digitalRead(4) == HIGH) {
      buttonPressed = true;
    }
  }

  buttonPressed = false;

  CURRENT_DIRECTION = NORTH;
  while (DISTANCE_NORTH < 200) {
    Forward(200);
    ReadEncoders();
    Serial.println(DISTANCE_NORTH);
  }

  Head(EAST);
  Stop();

  while(DISTANCE_EAST < 100) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }

  Head(SOUTH);
  Stop();

  while(DISTANCE_NORTH > 0) {
    Forward(MAX_SPEED);
    ReadEncoders();
  }

  Head(WEST);
  while(DISTANCE_EAST > 0){
    Forward(MAX_SPEED);
    ReadEncoders();
  }

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
    RightTrack(1, 250);
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
