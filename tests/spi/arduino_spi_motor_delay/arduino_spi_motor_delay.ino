/*
 Control a brushless motor using a potentiometer
 Works with ARDUINO MEGA 2560 card 
 Authors : Alexandre Massot and Marc Schneider
 */

#include <SPI.h>

byte buf[2];
volatile byte pos;
volatile boolean process_motor_delay;

// Defines for setting and clearing register bits
// This is to speed up analog read
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Pins for the motor
const byte DIR_PIN = 3;
const byte ENABLE_PIN = 5;
const byte STEP_PIN = 2;

const int SENSOR_PIN_1 = A1;

const unsigned int MOTOR_MIN_DELAY = 3;
const unsigned int MOTOR_MAX_DELAY = 5000;
const unsigned int CRUISE_MOTOR_DELAY = 100; // Motor speed when moving to zero or start point

// Define a "dead band" so that the robot doesn't move within a range of values
const unsigned int DEAD_BAND_INF = 490;
const unsigned int DEAD_BAND_SUP = 530;

const char NONE = 'n'; // Do nothing
const char MOVE = 'm'; // Just move
const char RECORD = 'r'; // Move and record
const char ERASE_RECORD = 'e'; // Erase record file
const char PLAY = 'p'; // Replay movement
const char STOP = 's';
const char ZERO = 'z'; // Set "zero" point
const char GOTO_ZERO = 'h';
const char GOTO_START = 'g'; // Go to start point of last record
const char SPEED_MODE = 'v'; // We move in "speed" mode (sensor value is a speed)
const char POS_MODE = 'x'; // We move in "position" mode (sensor value is a position)

unsigned int sensorCurValue;
unsigned int motorDelay;
long curNbImpuls;

// Constants used when computing mapping between input value (sensor) and output value (motor)
const float MAP_HIGH = (float)abs(MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) / (float)abs(1023 - DEAD_BAND_SUP);
const float MAP_LOW = ((float)MOTOR_MAX_DELAY - (float)MOTOR_MIN_DELAY) / (float)(1023 - 0);

char cmdRead;

void setup() {
  // To speed up analog read, set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;

  Serial.begin(115200);

  // Default command and mode
  cmdRead = STOP;
  
  // Variable initialization
  curNbImpuls = 0; // Current motor position
  sensorCurValue = 0;
  motorDelay = 3;

  // Motor initialization
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Enable motor
  
  // SPI initialization
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_motor_delay = false;

  // now turn on interrupts
  SPI.attachInterrupt();  
}

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;  // grab byte from SPI Data Register
  // add to buffer if room

  buf [pos++] = c;
    
  if (pos == 2) // 2nd byte
    process_motor_delay = true;
}  // end of interrupt routine SPI_STC_vect

void loop() {
  mainLoop();
}

void mainLoop() {
  cmdRead = MOVE;
  if (cmdRead == MOVE) {
    moveOneStep(motorDelay, true);
  }
  else if (cmdRead == STOP) {
    stop();
  }
  else if (cmdRead == NONE) {
    //Serial.println("Waiting for a command");
  }
  
  if (process_motor_delay) {
    //buf [pos] = 0;
    //Serial.println(String(buf[0], HEX));
    //Serial.println(String(buf[1], HEX));
    motorDelay = String(buf[0]).toInt() + String(buf[1] << 8, DEC).toInt();
    Serial.println(motorDelay);
    pos = 0;
    process_motor_delay = false;
  }
}

void moveOneStep(int microSecs, boolean isForward) {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP_PIN, HIGH);
  if (isForward) {
    curNbImpuls++;
    digitalWrite(DIR_PIN, HIGH);
  }
  else {
    curNbImpuls--;
    digitalWrite(DIR_PIN, LOW);
  }
  delayMicroseconds(microSecs);
}


void stop() {
  digitalWrite(ENABLE_PIN, LOW);
}

int readValueFromSensor(int pin) {
  return analogRead(pin); // Get a value from 0 to 1023
}

boolean isMovingForward(int sensorValue) {
  if (sensorValue >= DEAD_BAND_SUP) return true;
  return false;
}

boolean isMovingBack(int sensorValue) {
  if (sensorCurValue <= DEAD_BAND_INF) return true;
  return false;
}

// Map from analog value to delay for motor when direction is "high"
float mapAnalogToDelayHigh(float x, float in_min, float in_max, float out_min, float out_max) {
  //return out_max - (x - in_min) * (abs(out_max - out_min) / abs(in_max - in_min));
  return out_max - (x - in_min) * MAP_HIGH;
} 

// Map from analog value to delay for motor when direction is "high"
float mapAnalogToDelayLow(float x, float in_min, float in_max, float out_min, float out_max) {
  //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (x - in_min) * MAP_LOW + (float)MOTOR_MIN_DELAY;
}

void displayCurInfos() {
  Serial.println("Cur pos:" + String(curNbImpuls));
  Serial.println("Ram:" + String(freeRam()));
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
