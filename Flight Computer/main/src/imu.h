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

  public: 
    struct acceleration { 
      double x, y, z; 
    } acceleration;

    struct gyro {
      double x, y, z;
    } gyro;

    void refresh() {
      sensor.getEvent(&a, &g, &t);
    }

    void getGyroscope ( double &x, double &y, double &z ) {
      //if ( mpu_gyro->getEvent(&g) ) return true; else return false; 
      x = g.gyro.x * 57.29578;     // rad to deg/second 
      y = g.gyro.y * 57.29578;
      z = g.gyro.z * 57.29578; 
      gyro.x = x; 
      gyro.y = y;
      gyro.z = z;
    }
    void getAcceleration ( double &x, double &y, double &z ) {
      //if ( mpu_accel->getEvent(&a) ) return true; else return false; 
      x = a.acceleration.x; 
      y = a.acceleration.y; 
      z = a.acceleration.z; 
      acceleration.x = x; 
      acceleration.y = y; 
      acceleration.z = z; 
    } 
    void getTemperature(double &temperature_deg) {
      //if ( mpu_temp->getEvent(&t) ) return true; else return false; 
      temperature_deg = t.temperature; 
    } 
} IMU;


bool initIMU(MPU6050_Wrapper &imu_device) {
  if (IMU.sensor.begin()) {
    IMU.sensor.setAccelerometerRange(MPU6050_RANGE_8_G); 
    IMU.sensor.setGyroRange(MPU6050_RANGE_500_DEG); 
    IMU.sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true; 
  } else {
    debugln(F("Failed to initialize IMU"));
    return false; 
  }
}


void calibrateIMU(MPU6050_Wrapper &imu_device, ThreeAxes &accel_avg, ThreeAxes &gyro_drift) {
  int initial_millis = millis(); // iteration index  
  int i = 0; 
  double gx, gy, gz, ax, ay, az; // measurements 
  double gyro_x = 0, gyro_y = 0, gyro_z = 0, acc_x = 0, acc_y = 0, acc_z = 0; // sums 
  while ( (millis() - initial_millis) <= CALIBRATION_TIME_SEC*1000 ) {
    IMU.refresh(); 
    i++; 
    imu_device.getGyroscope(gx, gy, gz);   // get drift rates in degrees/second 
    imu_device.getAcceleration(ax, ay, az);  // get acceleration in g  
    gyro_x += gx; 
    gyro_y += gy; 
    gyro_z += gz;
    acc_x += ax; 
    acc_y += ay; 
    acc_z += az;
  }
  accel_avg.x = acc_x / i; 
  accel_avg.y = acc_y / i; // should be -9.81 before launch 
  accel_avg.z = acc_z / i; 
  gyro_drift.x = gyro_x / i; 
  gyro_drift.y = gyro_y / i; 
  gyro_drift.z = gyro_z / i; 

  debugln(F("\nFinished Calibration")); 
} 


double getAccelerationMagnitude (double ax, double ay, double az) {
  double magnitude = sqrt(sq(ax) + sq(ay) + sq(az)); 
  return magnitude; 
}


bool checkLanded(double gx, double gy, double gz) { 
  double margin = 1.0; 
  if (fabs(gx) < margin && fabs(gy) < margin && fabs(gz) < margin) {
    return true; 
  }
  return false; 
}

