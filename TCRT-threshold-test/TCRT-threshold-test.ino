
//right station-sensing tcrt
#define RS_TCRT 5
//left station-sensing tcrt
#define LS_TCRT 7

#define PUMP_SENSE 33

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(RS_TCRT, INPUT);
  pinMode(LS_TCRT, INPUT);
  pinMode(PUMP_SENSE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Right_station:");
  Serial.print(digitalRead(RS_TCRT));
  Serial.print(",");
  Serial.print("Left_Station:");
  Serial.print(digitalRead(LS_TCRT));
  Serial.print(",");
  Serial.print("Voltage:");
  Serial.println(analogRead(PUMP_SENSE));

  delay(50);
}
