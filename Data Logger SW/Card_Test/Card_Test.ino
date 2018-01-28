
void setup() {
  Serial.begin(9600);
  // initialize digital pin
  pinMode(8, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  
}

// the loop function runs over and over again forever
void loop() {
  digitalRead(8);
  if (digitalRead(8) == LOW) {
    digitalWrite(6, HIGH);
  }else{digitalWrite(6,LOW);
  }
  digitalRead(7);
  Serial.print("Card Detect = "); //for debug
  Serial.println(digitalRead(8)); //for debug
  Serial.print("Write Protect = "); //for debug
  Serial.println(digitalRead(7)); //for debug
  Serial.println("");
  delay(1000);
}
