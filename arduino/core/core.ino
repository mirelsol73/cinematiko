/*
 Control a brushless motor using a potentiometer
 Authors : Alexandre Massot and Marc Schneider
*/

// Defines for setting and clearing register bits
// This is to speed up analog read
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


const byte DIR_PIN = 3;
const byte ENABLE_PIN = 4;
const byte STEP_PIN = 2;

const unsigned int MOTOR_CALIBRATE = 1750;
const unsigned int MOTOR_MIN_DELAY = 3;
const unsigned int MOTOR_MAX_DELAY = 5000;

const unsigned int DEAD_BAND_INF = 505;
const unsigned int DEAD_BAND_MID = 511;
const unsigned int DEAD_BAND_SUP = 515;

unsigned int sensorCurValue = 0;
unsigned int motorDelay = 0;

const char RECORD = 'r';
const char PLAY = 'p';
const char STOP = 's';
const unsigned int MAX_RECORDS = 3500;
int nbRecords = 0;
const unsigned int RECORD_FREQ = 10; // in ms
unsigned long recordTime = 0;
unsigned long currentTime = 0;
unsigned int recordValues[MAX_RECORDS];
char cmdRead = STOP; // default action

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
  
  nbRecords = 0;
  /*
  for (int i=0; i<MAX_RECORDS; i++) {
    recordValues[i] = 0;
  }
  */
}

void loop() {
  mainLoop();
  //move(3);
}

void mainLoop() {
  if (Serial.available() > 0) {
    cmdRead = Serial.read();
    if (cmdRead == RECORD) {
      nbRecords = 0;
    }
  }
  if (cmdRead == RECORD && nbRecords < MAX_RECORDS) { //  && nbRecords < MAX_RECORDS
    sensorCurValue = analogRead(A1); // Get a value from 0 to 1023
    if (millis() - recordTime >= RECORD_FREQ) {
      recordValues[nbRecords] = sensorCurValue;
      recordTime = millis();
      nbRecords++;
    }
    move(sensorCurValue);
  }
  else if (cmdRead == PLAY) {
    Serial.print("Replaying the record : ");
    Serial.println(nbRecords);
    for (int i=0; i<nbRecords; i++) {
      Serial.println(recordValues[i]);
      unsigned long curTime = millis();
      while (millis() - curTime <= RECORD_FREQ) {
        move(recordValues[i]);
        analogRead(A1); // Just to simulate the delay introduced when recording values
      }
    }
    digitalWrite(ENABLE_PIN, HIGH);
    //nbRecords = 0;
    recordTime = 0;
    cmdRead = STOP;
  }
  else if (cmdRead == STOP) {
    Serial.println("Stopping the motor");
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.print("Values recorded : ");
    Serial.println(nbRecords);
    for (int i=0; i<nbRecords; i++) {
      Serial.println(recordValues[i]);
    }
  }
  else {
    Serial.print("Unknown command : ");
    Serial.println(cmdRead);
  }
  /*
  sensorCurValue = analogRead(A1); // Get a value from 0 to 1023
  move(sensorCurValue, 'r');
  */
  //stop();
}

void move(int sensorCurValue) {
  if (sensorCurValue <= DEAD_BAND_INF) { // Define a "dead band" so that the robot doesn't move within a range of values
    digitalWrite(DIR_PIN, LOW); // Set the direction
    motorDelay = map(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    moveOneStep(motorDelay);
  }
  else if (sensorCurValue >= DEAD_BAND_SUP) {
    digitalWrite(DIR_PIN, HIGH); // Set the direction
    motorDelay = map(sensorCurValue, DEAD_BAND_SUP, 1023, MOTOR_MAX_DELAY, MOTOR_MIN_DELAY);
    moveOneStep(motorDelay);
  }
  else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
  
}

void moveOneStep(int microSecs) {
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(microSecs);
}

void stop() {
  digitalWrite(ENABLE_PIN, LOW);
}
