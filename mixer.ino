// This is programm performs on-board Arduino-based mixing of THR and RUD sygnals for RC 
// two-engine aircraft models. The controller supports calibration procedure started 
// by button and uses LED to indicate the controller state.

#include <EEPROM.h>

boolean calibOn = false;
boolean startOn = true;
boolean calibParam1Done = false;

// Params with GAIN and THR calibration data.
const int NUM_PARAMS = 6;
const int RMIN =       0;
const int RMAX =       1;
const int TMIN =       2;
const int TMAX =       3;
const int RMID =       4;
const int RCUT =       5;

// RUDDER variables
int RUD_IN_PIN = 3;
volatile int rMax = 0;
volatile int rMin = 0;
volatile int rIn = 0;
volatile unsigned long rStart = 0;
volatile int rMid = 0;
volatile int rCut = 0;


// Throttle variables
int THR_IN_PIN = 2;
volatile int tMin = 0;
volatile int tMax = 0;
volatile int tIn = 0;
volatile unsigned long tStart = 0;

// THR1 variables
int THR1_OUT_PIN = 4;
volatile int thr1Out = 0;

// THR2 variables
int THR2_OUT_PIN = 5;
volatile int thr2Out = 0;

int BUTTON_IN_PIN = 6;
int LED_OUT_PIN = 7;

// ARDUINO-specific functions

void setup() {

  // Initialization iterrupts and timers. 
  systemInit();
  
  formatEEPROM();
  readEEPROM();

  // On start action.
  delay(100);
}

void loop() {    
  if (startOn) {
    blinkLED(3);
    startOn = false;
  }
   
  if (buttonPressed()) { 
    if (!calibOn) {
      int tMinTemp = tMin;
      
      rMin = 1500;
      rMax = 1500;
      tMin = 1500;
      tMax = 1500;
            
      calibOn = true;
      calibParam1Done = false;
      setLED(false);

      delay(100);
      
      thr1Out = tMinTemp;
      thr2Out = tMinTemp;
    }
    
    rMin = min(rMin, rIn);
    rMax = max(rMax, rIn); 
    tMin = min(tMin, tIn);
    tMax = max(tMax, tIn);
  } else {  
    if (calibOn) {
      // Check if calibration is valid, and 
      // write data of calibration in EEPROM memory. 
        if (!calibParam1Done) {
          if (abs(rMax + rMin - 2 * rMid) < 300 &&
              (tMax - tMin) > 300 && 
              (rMax - rMin) > 300) {                
            // Store new values.0.
            rCut = tIn;
            rMid = rIn;          
            
            blinkLED(3);   
            
            EEPROM.write(RMIN, rMin / 10);
            EEPROM.write(RMAX, rMax / 10);              
            EEPROM.write(TMIN, tMin / 10);
            EEPROM.write(TMAX, tMax / 10);   
            EEPROM.write(RMID, rMid / 10);
            EEPROM.write(RCUT, rCut / 10);                
          } else {
            // Read old values.
            readEEPROM();
          }
          
          calibParam1Done = true;          
      } else if (tIn - tMin < 50) {  
        calibOn = false;       
      } 
    }
  }
    
  delay(50);    
  detectInputSignals();
}

// VECTORS

// Timer interrupt processing.
ISR(TIMER1_OVF_vect)
{
  bool isElv1 = digitalRead(THR1_OUT_PIN) == HIGH;  
  bool isElv2 = digitalRead(THR2_OUT_PIN) == HIGH;
 
  TCCR1A = 0;
  digitalWrite(THR1_OUT_PIN, LOW);
  digitalWrite(THR2_OUT_PIN, LOW);
  sei();

  if (isElv1) {
    // Start THR2 output.
    digitalWrite(THR2_OUT_PIN, HIGH);    
    setTimer(thr2Out);
  } 
}

// RUD interrupt processing.
ISR(INT0_vect)
{
  if (digitalRead(RUD_IN_PIN) == LOW) {
    rIn = micros() - rStart + 20;    
  } else {
    // Start THR1 output.    
    digitalWrite(THR1_OUT_PIN, HIGH);
    rStart = micros();
    setTimer(thr1Out);
    sei();
  } 
}

// THR interrupt processing.
ISR(INT1_vect)
{
  if (digitalRead(THR_IN_PIN) == LOW) {
    tIn = micros() - tStart + 20;
  } else {
    tStart = micros();
    sei();
   
    // Mix calculation.
    float rInA = getValue(rMin, rMax, rIn);
    float tInA = getValue(tMin, tMax, tIn);
    float rCutA = getValue(tMin, tMax, rCut);
    float diffCorr = 1.0 - rCutA;
    
    float rMidA = getValue(rMin, rMax, rMid);
    
    float diffRA = 2.0 * (rInA - rMidA);    
    float thr1 = tInA * (1.0 + diffRA * diffCorr);
    float thr2 = tInA * (1.0 - diffRA * diffCorr);
    float extra1 = (thr1 > 1.0) ? thr1 - 1.0 : 0.0;
    float extra2 = (thr2 > 1.0) ? thr2 - 1.0 : 0.0;
    
    if (!calibOn) {      
      thr1Out = setValue(tMin, tMax, thr1 - extra2);
      thr2Out = setValue(tMin, tMax, thr2 - extra1);        
    }
  }
}

// UTILITY FUNCTIONS

// The function models exponent in [0 - 1.0] interval.
float myexp(float k, float x)
{
    return (2 * x + k * x * x)/(2 + k);
}

// Sets timer to fire interrupt in some period of time. 
void setTimer(int lapse)
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = - lapse * 16;

  // Enabling Timer1 overflow interrupt.
  TCCR1B |= (1 << CS10);  
}

// Converts PPM pulse length in mks
// to double value in the range [0.0 - 1.0].
float getValue(int v1, int v2, int v)
{
  if (v < v1)
    return 0.0;
  else if (v > v2)
    return 1.0;

  return (v - v1) * 1.0 / (v2 - v1);
}

// Converts double value [0.0 - 1.0] to 
// the PPM pulse length in mks.
int setValue(int v1, int v2, float value)
{
  int v = v1 + value * (v2 - v1);

  if (v < v1)
    return v1;

  if (v > v2)
    return v2;

  return v;
}

// Prints int value into serial.
void prinrInt(const char* name, int value) 
{
  Serial.print(name);
  Serial.print(": ");
  Serial.println(value);
}

// Initilizes pins, timer and interrupts.
void systemInit()
{
  pinMode(THR2_OUT_PIN, OUTPUT);
  pinMode(THR1_OUT_PIN, OUTPUT);
  pinMode(LED_OUT_PIN, OUTPUT);

  // Timer setup.  
  TCCR1A = 0;
  TIMSK1 = (1 << TOIE1);  // Set CS10 bit so timer runs at clock speed:
  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
  
  // Enabling interrupts.
  cli();
  EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20);
  EICRB |= (1 << ISC60) | (1 << ISC61);
  EIMSK |= (1 << INT0) | (1 << INT1);
  sei();
}

// First time formatting.
void formatEEPROM()
{
   if (EEPROM.read(RMIN) != 0) 
      return;
  
   EEPROM.write(RMIN, 1096 / 10);
   EEPROM.write(RMAX, 1964 / 10);
   EEPROM.write(TMIN, 1096 / 10);
   EEPROM.write(TMAX, 1964 / 10);
   EEPROM.write(RMID, 1530 / 10);
   EEPROM.write(RCUT, 1096 / 10);
}

// Reads EEPROM into memory.
void readEEPROM()
{
  rMin = EEPROM.read(RMIN) * 10;
  rMax = EEPROM.read(RMAX) * 10;  
  tMin = EEPROM.read(TMIN) * 10;
  tMax = EEPROM.read(TMAX) * 10;  
  rMid = EEPROM.read(RMID) * 10;
  rCut = EEPROM.read(RCUT) * 10;
}

// Blinks LED is one of PPM is missed.
void detectInputSignals()
{
  if (!calibOn) {
    unsigned long curTime = micros();
  
    if (curTime - tStart > 1000000 ||
        curTime - rStart > 1000000 ) {
      setLED(false);    
    } else {
      setLED(true);
    }
  }
}

// Blinks LED num times.
void blinkLED(int num)
{
  for (int idx=0; idx<num; idx++) {
    setLED(true);
    delay(150);    
    setLED(false);
    delay(150);  
  }
}

// Sets LED on.
void setLED(boolean isOn)
{
  digitalWrite(LED_OUT_PIN, isOn ? 0 : 1);    
}

// Test if the button is pressed.
boolean buttonPressed() 
{
  return !digitalRead(BUTTON_IN_PIN);
}
