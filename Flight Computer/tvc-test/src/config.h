/*
 *
 * Created by almostlight on Nov 1, 2022 
 *
 * https://github.com/almostlight 
 * 
 * Pins configured for the Pitch v.0.1 flight computer 
 * 
 */

#pragma once 
/*
* These are preset on the Arduino. Do not redefine. 
* UART TX     0
* UART RX     1
* SPI MOSI    11
* SPI MISO    12
* SPI SCK     13
*/
////  PIN ALIASES
#define VOLTAGE A7
#define TVC_X   5
#define TVC_Y   4
#define FLASH_CS   6
#define SD_CS   7
#define LED_RED 10
#define LED_GRN 9
#define LED_BLU 8
#define BUZZER  A7
#define PYRO_1  A3
#define PYRO_2  A2
////  PID SETTINGS 
#define PID_TARGET    0.0 
#define KP    0.8
#define KI    0.0
#define KD    0.2 

#define GEAR_RATIO  4.0    //  how many degrees on servo per degree on mount 
#define SERVO_DEG_MIN -24.0   //  from home  
#define SERVO_DEG_MAX 24.0   
float SERVO_X_HOME = 92.0;    // adjust in case of offsets 
float SERVO_Z_HOME = 83.0; 

#define COUNTDOWN_SEC   10 
#define CALIBRATION_TIME_MILLIS  20000  // Set to at least 20 000  
#define SERVO_MIN     1000      //  chrono values 
#define SERVO_MAX     2000      //  use only for attaching 
////  GLOBAL CONSTANTS  
#define MAX_BURN_TIME_LIMIT_MILLIS     4000     //  change to actual value before flight 

////  TOGGLES 
const bool INTEGRAL_ENABLE = false;     // this is problematic 
const bool SERIAL_ENABLE = true;      // set to false before flight 

////  FLIGHT TOGGLE 
const bool FLIGHT = true;     //  when enabled, will enter infinite loops at startup errors 

