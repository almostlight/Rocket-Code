#pragma once
#include <Wire.h>
#include <SPI.h> 
#include <Servo.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Arduino_LSM9DS1.h>
#include <SD.h>
#include <SerialFlash.h> 

//  Setup devices 
Servo servo_x; 
Servo servo_z; 
Adafruit_BMP3XX BMP; 
SerialFlashFile signature;  
SerialFlashFile FLASH_log; 
File SD_log; 

