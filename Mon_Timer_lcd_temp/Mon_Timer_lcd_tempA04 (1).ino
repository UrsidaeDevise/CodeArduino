
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
//#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS  2        // Data wire is plugged into pin 2 on the Arduino
#define currentProbe1 0        // The first current meter 
#define currentProbe2 1        // The second current meter 
#define ledYellow     13
#define ledGreen      12
#define ledRed        11
#define relaySSR      10
#define inPushButton  8        // choose the input pin (for a pushbutton)
#define SENSOR_RESOLUTION 9   // How many bits to use for temperature values: 9, 10, 11 or 12

int state = LOW;      // the current state of the output pin
int previous = HIGH;    // the previous reading from the input pin
int buttonRead = 0; 
long time = 0;         // the last time the output pin was toggled
//long time2 = 0;
long debounce = 400;   // the debounce time, increase if the output flickers
//int ledPinYellow = ledYellow;
//int ledPinGreen = ledGreen;
//int ledPinRed = ledRed;
//int relayPinSSR = relaySSR;
//int inPin = inPushButton;   // choose the input pin (for a pushbutton)
int noSensors = 0;
const int currentMean1 = 510;

volatile byte count = 0;
volatile int currentValue1 = 0;
volatile int currentValue2 = 0;
volatile byte countold = 0;
volatile boolean sendOK = false;
char TCNT2init = 177;

// Array
unsigned int current1_array[100];
unsigned int current2_array[100];

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress sensorDeviceAddress;
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack



void setup()
{

  // activate LCD module
  lcd.begin (16,2); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  
  pinMode(ledYellow, OUTPUT);     // declare LED as output
  pinMode(ledGreen, OUTPUT);      // declare LED as output
  pinMode(ledRed, OUTPUT);        // declare LED as output
  pinMode(relaySSR, OUTPUT);      // declare SSR as output
  pinMode(inPushButton, INPUT_PULLUP);          // declare pushbutton as input and make pullup resistors avalibe
  digitalWrite(inPushButton, HIGH);       // turn on pullup resistors
  //ADMUX = bit (REFS0) | bit (REFS1);  // Internal 1.1V reference

  delay(10);
  
  // Set all to off
  digitalWrite(ledYellow, LOW);   // turn LED OFF
  digitalWrite(ledGreen, HIGH);   // turn LED ON
  digitalWrite(ledRed, LOW);     // turn LED OFF
  digitalWrite(relaySSR, LOW);   // turn SSR OFF

    // start serial port
  Serial.begin(115200);
  Serial.println("UrsiCNC serial promt started");

  // Start up the library
  sensors.begin();
  // Set, show resulotion
  //sensors.getAddress(sensorDeviceAddress, 0);
  //sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);
  //Serial.println(sensorDeviceAddress,DEC);
  //int resolution = sensors.getResolution(sensorDeviceAddress);
  //Serial.println(resolution,DEC);
  
  
  noInterrupts();           // disable all interrupts
  // initialize timer1 @ 2 Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  // initialize timer2 @ 100 Hz
  
  // TIMER2_OVF_vect (Timer2 overflow) is fired with freq: 
  // Freq_OVF = 16000000/(scale*(255-TCNT2init)) Hz
  // Square wave( _-_-_ ) on pin OVF_Pin has:
  // Freq_PIN = FreqOVF/2 

  TCCR2B = 0x00;                  // No clock source (Timer/Counter stopped) 
  TCNT2 = TCNT2init;              // Register : the Timer/Counter (TCNT2) and Output Compare Register (OCR2A and OCR2B) are 8-bit // Reset Timer Count
  TCCR2A = 0x00;                  // TCCR2A – Timer/Counter Control Register A // All bits to zero -> Normal operation
 
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20); // Prescale 1024 (Timer/Counter started)
  //TCCR2B &= ~(1<<CS21);          // Turn off CS21, CS22=1 CS21=0 CS20=1 -> prescale = 128
 
  TIMSK2 |= (1<<TOIE2);          // TIMSK2 – Timer/Counter2 Interrupt Mask Register // Bit 0 – TOIE2: Timer/Counter2 Overflow Interrupt Enable
 
  interrupts();             // enable all interrupts
  noSensors = sensors.getDeviceCount();
  lcd.home (); // set cursor to 0,0
  lcd.print("UrsiCNC"); 

}



void loop()
{
  buttonRead = digitalRead(inPushButton);  // read input value
  
  if (sendOK) {
   long time2 = millis();
   //float valueVcc = readVcc();
   //float current1 = (currentValue1-currentMean1)*valueVcc/1024/100;
   float current1 = (currentValue1-currentMean1)*5/1024/100;
   //lcd.setCursor (0,1); // set cursor to 0,1
   //lcd.print(countold,DEC);
   //lcd.setCursor (3,1); // set cursor to 3,1
   //lcd.print("T1:");
   //lcd.print(sensors.getTempCByIndex(1),1);
   Serial.println("UrsiCNC");
   Serial.print("Temperature is: ");
   Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
   //Serial.print(", T2: ");
   //Serial.print(sensors.getTempCByIndex(1)); // Why "byIndex"?  
   //Serial.print(", T3: ");
   //Serial.print(sensors.getTempCByIndex(2)); // Why "byIndex"?  
   //Serial.print(", T4: ");
   //Serial.print(sensors.getTempCByIndex(3)); // Why "byIndex"?  
   //Serial.print(sensorDeviceAddress);
   Serial.print(", Time:");
   Serial.println(millis() ,DEC);
   //Serial.println(valueVcc , DEC);
   Serial.println(current1, DEC);
  
   for(int n=0; n < 10; n++){
      Serial.print(current1_array[n]);
      Serial.print(",");
      }
   Serial.println(0xFF);
   for(int n=0; n < 10; n++){
      Serial.print(current2_array[n]);
      Serial.print(",");
      }
   sendOK = false;
   Serial.println(0xFF);
   Serial.println(countold);
   time2 = millis() - time2;
   Serial.println(time2 ,DEC);
  }

  if (buttonRead == LOW && previous == HIGH && millis() - time > debounce) {
    digitalWrite(ledYellow, digitalRead(ledYellow) ^ 1);   // toggle LED pin
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

  digitalWrite(ledRed, state);
  digitalWrite(ledYellow, !state);
  digitalWrite(relaySSR, !state);
  
  previous = HIGH;
   
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine, 2Hz
{
  
  digitalWrite(ledGreen, digitalRead(ledGreen) ^ 1);   // toggle LED pin
  countold = count;
  count = 0;
  sensors.requestTemperaturesnodelay();
  currentValue1 = analogRead(currentProbe1);
  currentValue2 = analogRead(currentProbe2);
  sendOK = true;
}

ISR(TIMER2_OVF_vect) { // 100 Hz
 //https://www.arduino.cc/en/Hacking/Atmega168Hardware
 // PORTD maps to Arduino digital pins 0 to 7
 TCNT2 = TCNT2init;
 current1_array[count] = analogRead(currentProbe1);
 current2_array[count] = analogRead(currentProbe2);
 count = ++count;
 //digitalWrite(ledYellow, digitalRead(ledYellow) ^ 1);   // toggle LED pin
 
};

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  //measured 1.075-1.076V 1099725-1100748
  result = 1125300 / result; // Back-calculate AVcc in mV 1.1 * 1023 *1000 , adjust acc.
  return result;
}
