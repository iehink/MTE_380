void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  int value = analogRead(A0);
  if (value > 0) {
   Serial.println(value);
  } else {
    Serial.println("whoops");
  }
  delay(1000);
}
