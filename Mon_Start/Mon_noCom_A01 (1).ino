

/* Demonstration sketch for PCF8574T I2C LCD Backpack 
Uses library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads GNU General Public License, version 3 (GPL-3.0) */
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


int ledPinYellow = 13;
int ledPinGreen = 12;
int ledPinRed = 11;
int relaySSR = 10;
int inPin = 8;   // choose the input pin (for a pushbutton)

int state = LOW;      // the current state of the output pin
int stateY = LOW;      // the current state of the output pin
// buttonRead instead int reading;           // the current reading from the input pin
int previous = HIGH;    // the previous reading from the input pin
int previousY = HIGH;    // the previous reading from the input pin
int buttonRead = 0;
long time = 0;         // the last time the output pin was toggled
long debounce = 250;   // the debounce time, increase if the output flickers
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

void setup()
{
  // activate LCD module
  lcd.begin (16,2); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  // Out&In's
  pinMode(ledPinYellow, OUTPUT);  // declare LED as output
  pinMode(ledPinGreen, OUTPUT);  // declare LED as output
  pinMode(ledPinRed, OUTPUT);  // declare LED as output
  pinMode(relaySSR, OUTPUT);  // declare LED as output
  pinMode(inPin, INPUT);    // declare pushbutton as input
  digitalWrite(inPin, HIGH);       // turn on pullup resistors


  delay(10);
  
  // Set all to off
  digitalWrite(ledPinYellow, LOW);  // turn LED OFF
  digitalWrite(ledPinGreen, HIGH);  // turn LED OFF
  digitalWrite(ledPinRed, HIGH);  // turn LED OFF
  digitalWrite(relaySSR, HIGH);  // turn LED OFF
}

void loop()
{
  
  lcd.home (); // set cursor to 0,0
  lcd.print("UrsiCNC"); 

  buttonRead = digitalRead(inPin);  // read input value
  //lcd.setCursor (0,1);        // go to start of 2nd line
  //lcd.print(millis());

if (buttonRead == LOW && previous == LOW && millis() - time > debounce) {
    if (state == HIGH){
      state = LOW;
      lcd.setCursor (10,0); // set cursor to second row
      lcd.print("ssrOFF");}
    else{
      state = HIGH;
      lcd.setCursor (10,0); // set cursor to second row
      lcd.print("ssrON ");}

    time = millis();    
  }

  digitalWrite(ledPinRed, state);
  digitalWrite(ledPinGreen, !state);
  digitalWrite(relaySSR, !state);

  
  previous = buttonRead;

}
