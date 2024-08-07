void setup() {
  // put your setup code here, to run once:
  pinMode(20, OUTPUT);
  pinMode(27, OUTPUT);
  
  digitalWrite(20, LOW);
  digitalWrite(27, LOW);

  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(20, HIGH);
  delay(1650);
  digitalWrite(20, LOW);
  delay(100);
  digitalWrite(20, HIGH);
  delay(100);
  digitalWrite(20, LOW);
  delay(2500);
  digitalWrite(20, HIGH);
  delay(100);
  digitalWrite(20, LOW);
  delay(3000);
}
