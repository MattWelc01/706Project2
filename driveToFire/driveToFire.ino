  int IRpin = A13;

void setup() {
  pinMode(14, OUTPUT);
  pinMode(IRpin, INPUT);
  Serial.begin(9600);
}

void loop() {
  /*
  digitalWrite(14,HIGH);
  delay(500);
  digitalWrite(14,LOW);
   delay(500);
   */
   Serial.println(analogRead(IRpin));
   delay(100);
}


void driveToFire(){


  
}
