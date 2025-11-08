
void setup() {
  //Открытие serial порта
  // на скорости 115200 Бит/сек (Бод).
  Serial.begin(115200);
}
//Бесконечный цикл loop.
void loop() {
  Serial.println(analogRead(32));
  delay(200);
}
