#include <EEPROM.h>

boolean calibOn = false;
boolean startOn = true;
boolean calibTrap = false;
boolean calibDone = false;

// Params with GAIN and THR calibration data.
const int NUM_PARAMS = 7;
const int RMIN =       0;
const int RMAX =       1;
const int EMIN =       2;
const int EMAX =       3;
const int RMID =       4;
const int RGAIN =      5;
const int RCUT =       6;

// RUDDER variables
int RUD_IN_PIN = 3;
volatile int rMax = 0;
volatile int rMin = 0;
volatile int rIn = 0;
volatile unsigned long rStart = 0;
volatile int rMid = 0;
volatile int rExpGain = 0;
volatile int rCut = 0;


// ELEV variables
int ELEV_IN_PIN = 2;
volatile int eMin = 0;
volatile int eMax = 0;
volatile int eIn = 0;
volatile unsigned long eStart = 0;

// ELV1 variables
int ELV1_OUT_PIN = 4;
volatile int elv1Out = 0;

// ELV2 variables
int ELV2_OUT_PIN = 5;
volatile int elv2Out = 0;

int BUTTON_IN_PIN = 6;
int LED_OUT_PIN = 7;

// ARDUINO main functions

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
      int eMinTemp = eMin;
      
      rMin = 1500;
      rMax = 1500;
      eMin = 1500;
      eMax = 1500;
            
      calibOn = true;
      calibDone = false;
      calibTrap = false;
      setLED(false);

      delay(100);
      
      elv1Out = eMinTemp;
      elv2Out = eMinTemp;
    }
    
    rMin = min(rMin, rIn);
    rMax = max(rMax, rIn); 
    eMin = min(eMin, eIn);
    eMax = max(eMax, eIn);
  } else {  
    if (calibOn) {
      // Check if calibration is valid, and 
      // write data of calibration in EEPROM memory. 
      if (!calibDone) {
        if (!calibTrap) {
          if (abs(rMax + rMin - 2 * rMid) < 300 &&
              (eMax - eMin) > 300 && 
              (rMax - rMin) > 300) {                
            rExpGain =  eIn;
            rMid = rIn;          
            
            blinkLED(3); 
            
            if (buttonPressed()) {
              calibTrap = true;
            }  else {
              calibDone = true;
              rCut = rMin;              
            }            
          } else {
            // Read old values.
            readEEPROM();
            calibDone = true;
          }
        } else {
          rCut = eIn;
          calibTrap = false;
          calibDone = true;        
          blinkLED(3);
        }
      } else if (eIn - eMin < 50) {  
        calibOn = false;   
        EEPROM.write(RMIN, rMin / 10);
        EEPROM.write(RMAX, rMax / 10);              
        EEPROM.write(EMIN, eMin / 10);
        EEPROM.write(EMAX, eMax / 10);   
        EEPROM.write(RMID, rMid / 10);
        EEPROM.write(RGAIN, rExpGain / 10);      
        EEPROM.write(RCUT, rCut / 10);        
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
  bool isElv1 = digitalRead(ELV1_OUT_PIN) == HIGH;  
  bool isElv2 = digitalRead(ELV2_OUT_PIN) == HIGH;
 
  TCCR1A = 0;
  digitalWrite(ELV1_OUT_PIN, LOW);
  digitalWrite(ELV2_OUT_PIN, LOW);
  sei();

  if (isElv1) {
    // Start ELV2 output.
    digitalWrite(ELV2_OUT_PIN, HIGH);    
    setTimer(elv2Out);
  } 
}

// RUDDER interrupt processing.
ISR(INT0_vect)
{
  if (digitalRead(RUD_IN_PIN) == LOW) {
    rIn = micros() - rStart + 20;    
  } else {
    // Start ELV1 output.    
    digitalWrite(ELV1_OUT_PIN, HIGH);
    rStart = micros();
    setTimer(elv1Out);
    sei();
  } 
}

// ELEV interrupt processing.
ISR(INT1_vect)
{
  if (digitalRead(ELEV_IN_PIN) == LOW) {
    eIn = micros() - eStart + 20;
  } else {
    eStart = micros();
    sei();
   
    // Mix calculation.
    float rInA = getValue(rMin, rMax, rIn);
    float eInA = getValue(eMin, eMax, eIn);
    float rExpGainA = getValue(eMin, eMax, rExpGain) * 3.0;
    float rCutA = getValue(eMin, eMax, rCut);
    float diffCorr = 1.0 - (1.0 - eInA) * rCutA ;
    
    float rMidA = getValue(rMin, rMax, rMid);
    
    float diffRA = 2.0 * (rInA - rMidA);    
    float elv1OutA = myexp(rExpGainA, eInA * (1 + diffRA * diffCorr));
    float elv2OutA = myexp(rExpGainA, eInA * (1 - diffRA * diffCorr));
      
    if (!calibOn) {      
      elv1Out = setValue(eMin, eMax, elv1OutA);
      elv2Out = setValue(eMin, eMax, elv2OutA);        
    }
  }
}

// UTILITY FUNCTIONS

float myexp(float k, float x)
{
    return (2 * x + k * x * x)/(2 + k);
}

// Set timer to fire interrupt in some period of time. 
void setTimer(int lapse)
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = - lapse * 16;

  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
}

// Converts PPM pulse length in mks
// to double 8value (0.0 - 1.0);
float getValue(int v1, int v2, int v)
{
  if (v < v1)
    return 0.0;
  else if (v > v2)
    return 1.0;

  return (v - v1) * 1.0 / (v2 - v1);
}

// Converts double value (0.0 - 1.0) to 
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
  pinMode(ELV2_OUT_PIN, OUTPUT);
  pinMode(ELV1_OUT_PIN, OUTPUT);
  pinMode(LED_OUT_PIN, OUTPUT);

  // Timer setup.  
  TCCR1A = 0;
  TIMSK1 = (1 << TOIE1);  // Set CS10 bit so timer runs at clock speed:
  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
  
  // Enabling interrupts
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
   EEPROM.write(EMIN, 1096 / 10);
   EEPROM.write(EMAX, 1964 / 10);
   EEPROM.write(RMID, 1530 / 10);
   EEPROM.write(RGAIN, 1964 / 10);
   EEPROM.write(RCUT, 1096 / 10);
}

// Reads EEPROM into memory.
void readEEPROM()
{
  rMin = EEPROM.read(RMIN) * 10;
  rMax = EEPROM.read(RMAX) * 10;  
  eMin = EEPROM.read(EMIN) * 10;
  eMax = EEPROM.read(EMAX) * 10;  
  rMid = EEPROM.read(RMID) * 10;
  rExpGain = EEPROM.read(RGAIN) * 10;  
  rCut = EEPROM.read(RCUT) * 10;
}

// This function blinks LED is one of PPM is missed.
void detectInputSignals()
{
  if (!calibOn) {
    unsigned long curTime = micros();
  
    if (curTime - eStart > 1000000 ||
        curTime - rStart > 1000000 ) {
      setLED(false);    
    } else {
      setLED(true);
    }
  }
}

void blinkLED(int num)
{
  for (int idx=0; idx<num; idx++) {
    setLED(true);
    delay(150);    
    setLED(false);
    delay(150);  
  }
}

void setLED(boolean isOn)
{
  digitalWrite(LED_OUT_PIN, isOn ? 0 : 1);    
}

boolean buttonPressed() 
{
  return !digitalRead(BUTTON_IN_PIN);
}
