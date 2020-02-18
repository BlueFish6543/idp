int IREmitter; // add the actual pin
int IRReceiver; // add the actual pin

void setup() {
  pinMode(IREmitter, OUTPUT);
  pinMode(IREmitter, INPUT);
}

void loop() {
  digitalWrite(IREmitter, HIGH);
  int value = digitalRead(IRReceiver);
  Serial.println(value);
}
