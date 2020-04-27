
void setup() {
Serial.begin(9600);
}

void loop() {
if (Serial.available() > 0) {
 int incoming = Serial.read();
 Serial.print("character recieved: ");
 Serial.print(incoming, DEC);
 Serial.println();
}
}
