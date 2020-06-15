/* STEERING_ANGLE_CALCULATION SENDER 313 */
#include <Arduino_LSM6DS3.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

/* WIFI CONNECTION */
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

/* STEERING_ANGLE_CALCULATION */
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


/* STEERING_ANGLE_CALCULATION */
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
float steering_angle;

/* WIFI CONNECTION */
int status = WL_IDLE_STATUS;
char ssid[] = "FRITZ!Box 7490";
char pass[] = "90346904906149662937";
unsigned int localPort = 2390; 
char packetBuffer[256]; //buffer to hold incoming packet
long previousMillis = 0;
WiFiUDP Udp;


void setup() {
  Serial.begin(9600);
  
/* WIFI CONNECTION */  
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("connecting to: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // wait 1 seconds for connection:
    delay(1000);
  }
  Serial.print("connected to: ");
  Serial.println(ssid);

  
  
/* STEERING_ANGLE_CALCULATION */
  IMU.begin();
  if (IMU.accelerationAvailable()) {
  IMU.readAcceleration(accX, accY, accZ);
  }
 // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();

  Udp.begin(localPort);
}


void loop() {
  
steering_angle_calculation();
steering_angle = compAngleY;


// send a Message evry 100ms
long currentMillis = millis();        
      if (currentMillis - previousMillis >= 10) {

        
        IPAddress IP(192, 168, 178, 163);
        
        Udp.beginPacket(IP, 2390);
        Udp.print(steering_angle);
        Udp.endPacket();
        Serial.println(steering_angle);
    
        previousMillis = currentMillis;
      }

// receive a Message
  // if there's data available, read a packet
int packetSize = Udp.parsePacket();
  if (packetSize) {
  int len = Udp.read(packetBuffer, 255);
  if (len > 0) {
      packetBuffer[len] = 0;
    }
  Serial.println(packetBuffer);
  }


delay(1);

}

void steering_angle_calculation() {
 
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


return kalAngleY;
return compAngleY;
delay(1);
}
