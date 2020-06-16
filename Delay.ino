```

const int REFRESH_RATE_MS=1;
const int DELAYED_SENSOR_DELAY_MS=10;
const int RING_BUFFER_SIZE=DELAYED_SENSOR_DELAY_MS/REFRESH_RATE_MS;

unsigned long lastSensorUpdate = 0;
float delayedSensorValue = 0;
float liveSensorValue = 0;
float liveSensorRingBuffer[RING_BUFFER_SIZE];
int ringBufferPosition=0;


unsigned long lastOutput = 0;

boolean readSensors() {
  unsigned long currentMs=millis();
  if (currentMs-lastSensorUpdate<REFRESH_RATE_MS || lastSensorUpdate>currentMs)
    return false; // this function returns false unless we need a refresh of our sensor values

  lastSensorUpdate+=REFRESH_RATE_MS;

  delayedSensorValue=currentMs; // TODO: SET TO THE VALUE OF THE SENSOR WHO IS DELAYED

  liveSensorValue=liveSensorRingBuffer[ringBufferPosition];

  liveSensorRingBuffer[ringBufferPosition]=currentMs; // TODO: SET TO THE VALUE OF THE SENSOR WHO HAS LIVE DATA
  
  ringBufferPosition++;
  if (ringBufferPosition>=RING_BUFFER_SIZE)
    ringBufferPosition=0;

  return true;
}

void loop() {
  if (readSensors() && millis()-lastOutput>100) {
    Serial.print(millis()); Serial.print(" | READ SENSORS: ");Serial.print(delayedSensorValue);Serial.print(" / ");Serial.print(liveSensorValue);Serial.println("");
    lastOutput=millis();
  }
}
```
