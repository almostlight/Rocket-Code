/*
 *
 * Created by almostlight on Nov 1, 2022 
 *
 * https://github.com/almostlight 
 * 
 * Pins configured for the Pitch v1.0 flight computer 
 * 
 */

#pragma once 
#include <Arduino.h>
#include <stdint.h>

/*
* These are preset on the Teensy. Do not redefine. 
* UART TX     0
* UART RX     1
* SPI MOSI    11
* SPI MISO    12
* SPI SCK     13
* I2C SCL     19
* I2C SDA     18
*/

//  Toggle for serial communication 
#define DEBUG 
#ifdef DEBUG 
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else 
#define debug(x)
#define debugln(x)
#endif

////  PIN ALIASES
#define OUT1    0
#define OUT2    1
#define OUT3    2
#define LED_RED 3
#define LED_BLU 4
#define LED_GRN 5
#define BUZZER  6
#define PYRO_1  7
#define PYRO_2  8
#define PYRO_3  9
#define IMU_INT 10
#define VOLTAGE 23
#define BMP_INT 22
#define CS_SD   21
#define FC_CS   20
#define TVC_X   17
#define TVC_Y   16
#define PWM_1   15
#define PWM_2   14

////  PID SETTINGS 
#define PID_SETPOINT    0.0 
#define KP    0.4
#define KI    0.2
#define KD    0.1
#define GEAR_RATIO  4.0    //  how many degrees on servo per degree on mount 
#define MOUNT_DEG_MIN   -6.0   //  from 90 
#define MOUNT_DEG_MAX   6.0   
#define SERVO_DEG_MIN   -24.0
#define SERVO_DEG_MAX   24.0
double SERVO_X_HOME = 81.0;    // adjust in case of offsets 
double SERVO_Z_HOME = 84.0; 

#define CALIBRATION_TIME_SEC  10  // Set to at least 10 secs    
#define COUNTDOWN_TIME_SEC       10 
#define SERVO_MIN     1000      //  chrono values 
#define SERVO_MAX     2000      //  use only for attaching 
////  GLOBAL CONSTANTS  
#define BATTERY_MIN_VOLTS       10.8        //  3S at 3.6V per cell
#define BATTERY_MAX_VOLTS       12.6        //  3S at 4.2V per cell
#define MOTOR_BURN_TIME_LIMIT_MILLIS     5000     //  change to actual value before flight 
#define EJECTION_DELAY_MILLIS      4000     //  in miliseconds 
#define ABORT_DEGREE_THRESHOLD        40.0 
#define PARACHUTE_DEGREE_THRESHOLD    80.0 
#define IDLE_ACCELERATION_MARGIN     0.5    //    m/s^2
#define SEALEVELPRESSURE_HPA (1013.25) 
////  TOGGLES 
const bool CHECK_BATTERY = true;
const bool LOGGING_ENABLE = true;
const bool BUZZER_ENABLE = true;
const bool WIPE_FLASH = false;
//    the WIPE_FLASH toggle will wipe the flash chip at startup regardless of signature 
const bool PYROS_ENABLE = true;
const bool SUSPEND_ENABLE = true;
//    this will only run WAIT, POWERED_ASCENT, and LANDED 
const bool STATIC_FIRE = true;
//    this will count down and fire chutes 
const bool PARACHUTE_TEST = false; 
