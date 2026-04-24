void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("BOOT OK");
}

void loop() {
  Serial.println("alive");
  delay(1000);
}
