/* THESE ARE PREDEFINED FOR THE PI PICO
  I2C_SDA   4 
  I2C_SCL   5 
  SPI_MOSI  16 
  SPI_MISO  19 
  SPI_SCK   18 
  SPI_CS    17 
DO NOT CHANGE */

//  PIN ALIASES
#define IGNITER_POWER   9 
#define SWITCH_OUTPUT   1
#define SWITCH_INPUT    0
#define BUZZER    15 
#define CLAMP_SERVO_PIN  14
#define EXTENDED    180
#define RETRACTED   80

//  GLOBAL CONSTANTS 
#define COUNTDOWN_LENGTH 20    // countdown in seconds - set to at least 30 before launch 
#define IGNITION_LENGTH 2000   // pyro ignition duration in miliseconds 
const bool PYROS_ENABLE = true; 

