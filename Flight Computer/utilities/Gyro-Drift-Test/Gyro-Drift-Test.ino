#include <Arduino_LSM9DS1.h> 
#include <SPI.h> 

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started Serial communication");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  Serial.print(IMU.gyroscopeSampleRate());
  }
  delay(1000);
}

float DRIFT_X, DRIFT_Y, DRIFT_Z; 
void loop() {
  calibrateIMU(IMU, 10, DRIFT_X, DRIFT_Y, DRIFT_Z);
  Serial.print("\t X: "); Serial.print(DRIFT_X); 
  Serial.print("\t Y: "); Serial.print(DRIFT_Y); 
  Serial.print("\t Z: "); Serial.print(DRIFT_Z); Serial.print("\n");
  // while (1);
}


void calibrateIMU(LSM9DS1Class &imu_device, int loops, float &driftX, float &driftY, float &driftZ) {
  driftX = 0;
  driftY = 0;
  driftZ = 0; 
  float raw_x, raw_y, raw_z;
  float driftSumX = 0, driftSumY = 0, driftSumZ = 0;
  int i = 0;
  while (i<loops) {
    i++; 
    imu_device.readGyroscope(raw_x, raw_y, raw_z);   // get drift rates in degrees/second 
    driftSumX += raw_x; 
    driftSumY += raw_y; 
    driftSumZ += raw_z; 
    driftX = driftSumX / i;   // average it out 
    driftY = driftSumY / i;
    driftZ = driftSumZ / i;
  }
} 

