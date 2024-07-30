void setup() {
  // put your setup code here, to run once:
  pinMode(19, OUTPUT);
  
  digitalWrite(19, LOW);

  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(19, HIGH);
  delay(2500);
  digitalWrite(19, LOW);
  delay(500);
  digitalWrite(19, HIGH);
  delay(250);
  digitalWrite(19, LOW);
  delay(2500);
  digitalWrite(19, HIGH);
  delay(250);
  digitalWrite(19, LOW);
  delay(3000);
}
