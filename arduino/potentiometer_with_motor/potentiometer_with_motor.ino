int dirpin = 3;
int enablepin = 4;
int steppin = 12;

void setup() {
    Serial.begin(9600);
	pinMode(dirpin, OUTPUT);
	pinMode(enablepin, OUTPUT);
	pinMode(steppin, OUTPUT);

	digitalWrite(enablepin, HIGH);
}

void loop() {
    int motorDelay;
    int sensorValue = analogRead(A0);
    Serial.println(sensorValue, DEC);
    int relV = sensorValue / 2;
    if (sensorValue <= 611) {
      digitalWrite(dirpin, LOW); // Set the direction
      motorDelay = 611 - (relV * 3 / 611);
    }
    else {
      digitalWrite(dirpin, HIGH); // Set the direction
      motorDelay = relV * 3 / 611;
    }
    
    move(motorDelay);
  
	//stop();
}

void stop() {
    digitalWrite(enablepin, LOW);
}

void move(int microSecs) {
	digitalWrite(steppin, LOW);
	delayMicroseconds(microSecs);
	digitalWrite(steppin, HIGH);
	//delayMicroseconds(microSecs);
}
