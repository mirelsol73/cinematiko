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

const unsigned int MOTOR_MIN_DELAY = 3;
const unsigned int MOTOR_MAX_DELAY = 5000;

const unsigned int DEAD_BAND_INF = 505;
const unsigned int DEAD_BAND_MID = 511;
const unsigned int DEAD_BAND_SUP = 515;

unsigned int sensorCurValue = 0;
unsigned int motorDelay = 0;

boolean lastDirLow = true;

const char MOVE = 'm'; // Just move
const char RECORD = 'r'; // Move and record
const char PLAY = 'p';
const char STOP = 's';

const unsigned int MAX_RECORDS = 3500;
int nbRecords = 0;
const unsigned int RECORD_FREQ = 10; // in ms
const unsigned int READ_FREQ = 1000; // read frequency from the input device in ms
unsigned long lastReadTime = 0; // when the last reading from input device was done
unsigned long recordTime = 0;
//unsigned long currentTime = 0;
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
  //move(1020);
  //stop();
}

void mainLoop() {
  if (Serial.available() > 0) {
    cmdRead = Serial.read();
    if (cmdRead == RECORD) {
      nbRecords = 0;
    }
  }
  if (cmdRead == MOVE) {
    /*
    if (millis() - lastReadTime > READ_FREQ) {
      sensorCurValue = analogRead(A1);
      lastReadTime = millis();
    }
    move(sensorCurValue);
    */
    sensorCurValue = analogRead(A1);
    move(sensorCurValue);
  }
  else if (cmdRead == RECORD && nbRecords < MAX_RECORDS) {
    sensorCurValue = analogRead(A1); // Get a value from 0 to 1023
    if (millis() - recordTime >= RECORD_FREQ) {
      recordValues[nbRecords] = sensorCurValue;
      recordTime = millis();
      nbRecords++;
    }
    move(sensorCurValue);
  }
  else if (cmdRead == PLAY) {
    for (int i=0; i<nbRecords; i++) {
      unsigned long curTime = millis();
      while (millis() - curTime <= RECORD_FREQ) {
        move(recordValues[i]);
        analogRead(A1); // Just to simulate the delay introduced when recording values
      }
    }
    digitalWrite(ENABLE_PIN, HIGH);
    recordTime = 0;
    cmdRead = STOP;
  }
  else if (cmdRead == STOP) {
    Serial.println("Stopping the motor");
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.print("Values recorded : ");
    Serial.println(nbRecords);
  }
  else {
    Serial.print("Unknown command : ");
    Serial.println(cmdRead);
  }
}

void readValueFromMotor() {
}

void move(int sensorCurValue) {
  if (sensorCurValue <= DEAD_BAND_INF) { // Define a "dead band" so that the robot doesn't move within a range of values
    if (!lastDirLow) {
      digitalWrite(DIR_PIN, LOW); // Set the direction
    }
    lastDirLow = true;
    //motorDelay = mapAnalogToDelayLow(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    motorDelay = map(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    moveOneStep(motorDelay);
  }
  else if (sensorCurValue >= DEAD_BAND_SUP) {
    if (lastDirLow) {
      digitalWrite(DIR_PIN, HIGH); // Set the direction
    }
    lastDirLow = false;
    //motorDelay = mapAnalogToDelayHigh(sensorCurValue, DEAD_BAND_SUP, 1023, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
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

// Map from analog value to delay for motor when direction is "high"
float mapAnalogToDelayHigh(float x, float in_min, float in_max, float out_min, float out_max) {
  return out_max - (x - in_min) * (abs(out_max - out_min) / abs(in_max - in_min));
} 

// Map from analog value to delay for motor when direction is "high"
float mapAnalogToDelayLow(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 

