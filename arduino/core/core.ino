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

// Don't use pin 4 for the moment as it is used by SD card
const byte DIR_PIN = 3;
const byte ENABLE_PIN = 5;
const byte STEP_PIN = 2;

const int SENSOR_PIN_1 = A1;

const unsigned int MOTOR_MIN_DELAY = 3;
const unsigned int MOTOR_MAX_DELAY = 5000;
const unsigned int CRUISE_MOTOR_DELAY = 100; // Motor speed when moving to zero or start point

// Define a "dead band" so that the robot doesn't move within a range of values
const unsigned int DEAD_BAND_INF = 505;
const unsigned int DEAD_BAND_MID = 511;
const unsigned int DEAD_BAND_SUP = 515;

unsigned int sensorCurValue = 0;
unsigned int motorDelay = 0;

boolean lastDirLow = true;

const char NONE = 'n'; // Do nothing
const char MOVE = 'm'; // Just move
const char RECORD = 'r'; // Move and record
const char PLAY = 'p'; // Replay movement
const char STOP = 's';
const char ZERO = 'z'; // Set "zero" point
const char GOTO_ZERO = 'h';
const char GOTO_START = 'g'; // Go to start point of last record

long curNbImpuls = 0; // Current motor position
long startRecordX = 0; // Current motor position when start recording
const unsigned int MAX_RECORDS = 3500;
int nbRecords = 0;
const unsigned int RECORD_FREQ = 10; // in ms
unsigned long lastReadTime = 0; // when the last reading from input device was done
unsigned long recordTime = 0;
unsigned int recordValues[MAX_RECORDS];

// Constants used when computing mapping between input value (sensor) and output value (motor)
const float MAP_HIGH = (float)abs(MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) / (float)abs(1023 - DEAD_BAND_SUP);
const float MAP_LOW = ((float)MOTOR_MAX_DELAY - (float)MOTOR_MIN_DELAY) / (float)(1023 - 0);

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
}

void loop() {
  mainLoop();
}

void mainLoop() {
  if (Serial.available() > 0) {
    Serial.println("---");
    cmdRead = Serial.read();
    if (cmdRead == RECORD) {
      startRecordX = curNbImpuls;
      nbRecords = 0;
      Serial.println("Recording movement");
    }
  }
  if (cmdRead == MOVE) {
    sensorCurValue = readValueFromSensor(SENSOR_PIN_1);
    move(sensorCurValue);
  }
  else if (cmdRead == RECORD) {
    if (nbRecords < MAX_RECORDS) {
      sensorCurValue = readValueFromSensor(SENSOR_PIN_1);
      if (millis() - recordTime >= RECORD_FREQ) {
        recordValues[nbRecords] = sensorCurValue;
        recordTime = millis();
        nbRecords++;
      }
      move(sensorCurValue);
    }
    else {
      Serial.println("Stop : max nb of records reached!");
      cmdRead = STOP;
    }
  }
  else if (cmdRead == GOTO_START) {
    moveToStartRecord();
    cmdRead = NONE;
  }
  else if (cmdRead == PLAY) {
    moveToStartRecord();
    Serial.println(curNbImpuls);
    Serial.println("Mvt replay");
    long nbImpuls = 0;
    for (int i=0; i<nbRecords; i++) {
      unsigned long curTime = millis();
      // Replay sampling
      while (millis() - curTime < RECORD_FREQ) {
        move(recordValues[i]);
        readValueFromSensor(SENSOR_PIN_1); // Just to simulate the delay introduced when recording values
        nbImpuls++;
      }
    }
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("Ok");
    Serial.println(nbImpuls);
    recordTime = 0;
    cmdRead = STOP;
  }
  else if (cmdRead == ZERO) {
    Serial.println("Zero set");
    curNbImpuls = 0;
  }
  else if (cmdRead == GOTO_ZERO) {
    moveToZero();
    cmdRead = NONE;
  }
  else if (cmdRead == STOP) {
    digitalWrite(ENABLE_PIN, HIGH);
    cmdRead = NONE;
    Serial.println("Stop");
    Serial.println("Cur pos:" + String(curNbImpuls));
    Serial.println("Records#:" + String(nbRecords));
    Serial.println("Ram:" + String(freeRam()));
  }
  else if (cmdRead == NONE) {
    //Serial.println("Waiting for a command");
  }
  else {
    Serial.print("Unknown cmd:" + cmdRead);
    Serial.println(cmdRead);
  }
}

void move(int sensorCurValue) {
  if (isMovingBack(sensorCurValue)) {
    if (!lastDirLow) {
      digitalWrite(DIR_PIN, LOW); // Set the direction
    }
    lastDirLow = true;
    motorDelay = mapAnalogToDelayLow(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    //motorDelay = map(sensorCurValue, 0, DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    moveOneStep(motorDelay, false);
  }
  else if (isMovingForward(sensorCurValue)) {
    if (lastDirLow) {
      digitalWrite(DIR_PIN, HIGH); // Set the direction
    }
    lastDirLow = false;
    motorDelay = mapAnalogToDelayHigh(sensorCurValue, DEAD_BAND_SUP, 1023, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY);
    //motorDelay = map(sensorCurValue, DEAD_BAND_SUP, 1023, MOTOR_MAX_DELAY, MOTOR_MIN_DELAY);
    moveOneStep(motorDelay, true);
  }
  else {
    digitalWrite(ENABLE_PIN, HIGH);
  }
}

void moveOneStep(int microSecs, boolean isForward) {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP_PIN, HIGH);
  if (isForward) curNbImpuls++; else curNbImpuls--;
  delayMicroseconds(microSecs);
}

void moveRelative(long nbImpuls) {
  Serial.println("moveRelative");
  Serial.println(nbImpuls);
  if (nbImpuls > 0) {
    digitalWrite(DIR_PIN, LOW); // We must go back
  }
  else {
    digitalWrite(DIR_PIN, HIGH); // We must go forward
  }
  nbImpuls = abs(nbImpuls);
  for (long i=0; i<nbImpuls; i++) {
    moveOneStep(CRUISE_MOTOR_DELAY, (nbImpuls > 0));
  }
  Serial.println("Ok");
}

void moveToZero() {
  Serial.println("moveToZero");
  moveRelative(curNbImpuls);
  curNbImpuls = 0;
}

void moveToStartRecord() {
  Serial.println("moveToSR");
  //Serial.println(curNbImpuls);
  //Serial.println(startRecordX);
  moveRelative(curNbImpuls - startRecordX);
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
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

