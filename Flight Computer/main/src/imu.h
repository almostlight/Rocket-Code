#pragma once 
#include "config.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>


//  glorified tuple 
struct ThreeAxes {
  double x, y, z;
}; 


class MPU6050_Wrapper {       //    GO WRAPPERS 
  public:
    Adafruit_MPU6050 sensor;

  private:  
    sensors_event_t a, g, t;  
    Adafruit_Sensor *mpu_temp = sensor.getTemperatureSensor();
    Adafruit_Sensor *mpu_accel = sensor.getAccelerometerSensor();
    Adafruit_Sensor *mpu_gyro = sensor.getGyroSensor();

    ThreeAxes AccBias, GyroBias; 

  public: 
    ThreeAxes acceleration, gyro; 

    void setAccBias ( ThreeAxes bias ) { AccBias = bias; }
    void setGyroBias ( ThreeAxes bias ) { GyroBias = bias; }

    void refresh() {
      sensor.getEvent(&a, &g, &t);
    }

    //  uncalibrated gyro rates 
    void getGyroscope ( ThreeAxes &data ) {
      //if ( mpu_gyro->getEvent(&g) ) return true; else return false; 
      data.x = (g.gyro.x * 57.29578) - GyroBias.x;     // rad to deg/second 
      data.y = (g.gyro.y * 57.29578) - GyroBias.y;
      data.z = (g.gyro.z * 57.29578) - GyroBias.z; 
      gyro.x = data.x; 
      gyro.y = data.y;
      gyro.z = data.z;
    }
    //  uncalibrated accelerations  
    void getAcceleration ( ThreeAxes &data ) {
      //if ( mpu_accel->getEvent(&a) ) return true; else return false; 
      data.x = (a.acceleration.x) - AccBias.x; 
      data.y = (a.acceleration.y) - AccBias.y; 
      data.z = (a.acceleration.z) - AccBias.z; 
      acceleration.x = data.x; 
      acceleration.y = data.y; 
      acceleration.z = data.z; 
    } 
    void getAccelerationMagnitude ( double &magnitude ) {
      magnitude = sqrt(sq(acceleration.x) + sq(acceleration.y) + sq(acceleration.z)); 
    }
    void getTemperature(double &temperature_deg) {
      //if ( mpu_temp->getEvent(&t) ) return true; else return false; 
      temperature_deg = t.temperature; 
    } 
} IMU;


bool initIMU(MPU6050_Wrapper &imu_device) {
  if (IMU.sensor.begin()) {
    IMU.sensor.setAccelerometerRange(MPU6050_RANGE_4_G); 
    IMU.sensor.setGyroRange(MPU6050_RANGE_500_DEG); 
    IMU.sensor.setFilterBandwidth(MPU6050_BAND_44_HZ);
    return true; 
  } else {
    debugln(F("Failed to initialize IMU"));
    return false; 
  }
}


void calibrateIMU(MPU6050_Wrapper &imu_device) {
  ThreeAxes accel_bias, gyro_drift; 
  int initial_millis = millis(); // iteration index  
  int i = 0; 
  ThreeAxes gyro, accelerometer; // measurements 
  double gyro_x = 0, gyro_y = 0, gyro_z = 0, acc_x = 0, acc_y = 0, acc_z = 0; // sums 
  while ( (millis() - initial_millis) <= CALIBRATION_TIME_SEC*1000 ) {
    i++; 
    IMU.refresh(); 
    imu_device.getGyroscope(gyro);   // get drift rates in degrees/second 
    imu_device.getAcceleration(accelerometer);  // get acceleration in g  
    gyro_x += gyro.x; 
    gyro_y += gyro.y; 
    gyro_z += gyro.z;
    acc_x += accelerometer.x; 
    acc_y += accelerometer.y; 
    acc_z += accelerometer.z;
  }
  accel_bias.x = acc_x / i; 
  accel_bias.y = ((acc_y / i) - (GRAVITATIONAL_ACCEL)); // should be -9.81 before launch 
  accel_bias.z = acc_z / i; 
  gyro_drift.x = gyro_x / i; 
  gyro_drift.y = gyro_y / i; 
  gyro_drift.z = gyro_z / i; 

  imu_device.setAccBias(accel_bias); 
  imu_device.setGyroBias(gyro_drift); 

  debugln(F("\nFinished Calibration")); 
} 


bool checkLanded( ThreeAxes gyro ) { 
  double margin = 1.0; 
  if (fabs(gyro.x) < margin && fabs(gyro.y) < margin && fabs(gyro.z) < margin) {
    return true; 
  }
  return false; 
}

