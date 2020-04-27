/* Arduino 101: timer and interrupts
   1: Timer1 compare match interrupt example 
   more infos: http://www.letmakerobots.com/node/28278
   created by RobotFreak 
*/
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

volatile byte count = 0;
volatile byte countold = 0;
volatile boolean sendOK = false;
char TCNT2init = 177;
#define ledYellow 13
#define ledGreen 12
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

void setup()
{
  // activate LCD module
  lcd.begin (16,2); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  
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

}



void loop()
{
  lcd.home (); // set cursor to 0,0
  lcd.print("UrsiCNC"); 
  // your program here...
  if (sendOK) {
   lcd.setCursor (0,1); // set cursor to 0,0
   lcd.print(countold,DEC);
  }
   
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  //lcd.setCursor (0,1); // set cursor to 0,0
  //lcd.print(count,DEC); 
  sendOK = true;
  digitalWrite(ledGreen, digitalRead(ledGreen) ^ 1);   // toggle LED pin
  countold = count;
  count = 0;
}

ISR(TIMER2_OVF_vect) { 
 //https://www.arduino.cc/en/Hacking/Atmega168Hardware
 // PORTD maps to Arduino digital pins 0 to 7
 TCNT2 = TCNT2init;
 count = ++count;
 digitalWrite(ledYellow, digitalRead(ledYellow) ^ 1);   // toggle LED pin
 
};
