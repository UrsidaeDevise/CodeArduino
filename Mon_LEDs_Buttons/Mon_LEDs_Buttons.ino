int ledPinGreen = 12;
int ledPinRed = 11;
int relaySSR = 10;
int inPin = 8;   // choose the input pin (for a pushbutton)

int checkGreen = 0;
int checkRed = 0;
int checkRelay = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPinGreen, OUTPUT);  // declare LED as output
  pinMode(ledPinRed, OUTPUT);  // declare LED as output
  pinMode(relaySSR, OUTPUT);  // declare LED as output
  pinMode(inPin, INPUT);    // declare pushbutton as input
  
}

void loop() {
  // put your main code here, to run repeatedly:
  checkGreen = digitalRead(inPin);  // read input value
  if (checkGreen == HIGH) {         // check if the input is HIGH (button released)
    digitalWrite(ledPinRed, LOW);  // turn LED OFF
  } else {
    digitalWrite(ledPinRed, HIGH);  // turn LED ON
  }
}
