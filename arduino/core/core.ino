/*
 Maximal speed : 3 microseconds delay
 5000 microseconds is a very low speed
*/

// Defines for setting and clearing register bits
// This is to speed up analog read
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


int DIR_PIN = 3;
int ENABLE_PIN = 4;
int STEP_PIN = 2;

int MOTOR_CALIBRATE = 1750;
int MOTOR_MIN_DELAY = 3;
int MOTOR_MAX_DELAY = 5000;

int DEAD_BAND_INF = 505;
int DEAD_BAND_MID = 511;
int DEAD_BAND_SUP = 515;

int sensorCurValue = 0;
int motorDelay = 0;


void setup() {
  // To speed up analog read, set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  
  Serial.begin(9600);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  
  digitalWrite(ENABLE_PIN, HIGH);
}

void loop() {
  sensorCurValue = analogRead(A1); // Get a value from 0 to 1023
  int relativeV = sensorCurValue - DEAD_BAND_MID;
  if (sensorCurValue <= DEAD_BAND_INF) { // Define a "dead band" so that the robot doesn't move within a range of values
    digitalWrite(DIR_PIN, LOW); // Set the direction
    motorDelay = map(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    move(motorDelay);
  }
  else if (sensorCurValue >= DEAD_BAND_SUP) {
    digitalWrite(DIR_PIN, HIGH); // Set the direction
    motorDelay = map(sensorCurValue, DEAD_BAND_SUP, 1023, MOTOR_MAX_DELAY, MOTOR_MIN_DELAY);
    move(motorDelay);
  }
  else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
  //stop();
}

void stop() {
  digitalWrite(ENABLE_PIN, LOW);
}

void move(int microSecs) {
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(microSecs);
}
