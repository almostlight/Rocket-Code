#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h> 
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <SD.h>
#include <SerialFlash.h> 

//  Setup devices 
Servo servo_x; 
Servo servo_z; 
SerialFlashFile signature;  
SerialFlashFile FLASH_log; 
File SD_log; 

