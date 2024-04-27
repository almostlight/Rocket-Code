/*
 *
 * Created by almostlight on Nov 1, 2022 
 * 
 * https://github.com/almostlight 
 *
 * Written for the Pitch v1.0 flight computer 
 * 
 */


//  config
#include "config.h"
//  libraries 
#include <Arduino.h>
#include <fcntl.h> 
#include <stdint.h>
#include <vector> 
#include <numeric> 
//  headers 
#include "bmp.h"
#include "buzzer.h"
#include "filters.h"
#include "imu.h"
#include "led.h"
#include "logger.h"
#include "pid.h"
#include "power.h"
#include "servos.h"
#include "states.h"


using std::vector; 

////    SETUP FUNCTION    //// 

bool STARTUP_ERROR = false;
bool APOGEE = false; 
bool PARACHUTE_EJECTED = false; 

ThreeAxes GyroRate, Acceleration;  
ThreeAxes Angle, AdjustedAcceleration; 
double zero_altitude; 

vector<double> pressure_change_vector; 
unsigned long execution_time, loop_index = 0; 
unsigned long start_millis_main, end_millis_main; double delta_millis_main; 
double pressure, altitude; 
double smooth_pressure, pressure_change, pressure_change_avg; 
double acc_magnitude; 
double voltage; 
StateWrapper CurrentState(StateName::WAITING); 
PID pidX(&Record.angleX); 
PID pidZ(&Record.angleZ); 

  //  Initialize filters 
MovingAverageFilter Pressure_MovAvgFilter(20); 
LowPassFilter AccX_LowPass(0.6), AccY_LowPass(0.6), AccZ_LowPass(0.6); 
LowPassFilter Pressure_LowPass(0.6), Altitude_LowPass(0.6); 


void setup() {
  pinMode(VOLTAGE, INPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);
  pinMode(OUT1, OUTPUT); 
  pinMode(OUT2, OUTPUT); 
  pinMode(OUT3, OUTPUT); 
  
  setColor (Black); noBuzzerTone();  
  setColor (Red); buzzerTone(250); delay(333); 
  setColor (Green); buzzerTone(500); delay(333); 
  setColor (Blue); buzzerTone(1000); delay(333);
  setColor (Black); noBuzzerTone(); 

  Serial.begin(9600);
  //while (!Serial);     
  debugln(F("Started Serial communication"));

  if (CHECK_BATTERY) { 
    if (readBatteryVoltage() < BATTERY_MIN_VOLTS) {
      debugln(F("Battery near depletion. Suspending.")); 
      powerDown(); 
    } else {
      int dec = map(readBatteryVoltage(), BATTERY_MIN_VOLTS, BATTERY_MAX_VOLTS, 0, 10); 
      if (dec > 6) { 
        setColor(Green); 
      } else if (dec > 3) {
        setColor(Blue);
      } else {
        setColor(Orange); 
      }
      for (int i=0; i<dec; i++) {
        //  beep for every 10% charge 
        delay(400); buzzerTone(2000); delay(100); noBuzzerTone();
      }
      setColor(Black); 
      noBuzzerTone(); 
    }
  }

  if (PARACHUTE_TEST) {
    delay(1000); 
    countdown(COUNTDOWN_TIME_SEC); 
    setColor(Yellow); 
    if (PYROS_ENABLE) {
      ejectParachute(PYRO_1); 
    }
    setColor(Black); 
    //  stop here 
    while (true); 
  }

  initTvcServos(); 
  testTvcServos(1); 

  //  Initialize sensors  
  if (!initIMU(IMU)) {
    STARTUP_ERROR = true; 
  }
  if (!initBMP(BMP)) {
    STARTUP_ERROR = true; 
  }

  //  Initialize data logger if enabled 
  if (LOGGING_ENABLE) { 
    if (!initSDCard()) {
      STARTUP_ERROR = true; 
    }
    if (!createSDLogfile()) {
      STARTUP_ERROR = true; 
    }
    if (!initFlash()) {
      STARTUP_ERROR = true; 
    }
    //  Create recovery file of previous flight on SD 
    copyRecordsToSD(); 

    if (!createFlashLogfile(FILE_SIZE_4M)) {
      STARTUP_ERROR = true; 
    }
    openRecord();   //  opens logfile 
  }

  if (STARTUP_ERROR) {
    setColor (Red); 
    buzzerAlarmLoop(3000, 3000); 
  }

  //  Calibrate IMU 
  setColor(White); 
  calibrateIMU(IMU); 
  zero_altitude = getAverageAltitude(BMP); 
  IMU.refresh(); 
  IMU.getAcceleration(Acceleration); 
  
  AccX_LowPass.setInitialEstimate(Acceleration.x); 
  AccY_LowPass.setInitialEstimate(Acceleration.y); 
  AccZ_LowPass.setInitialEstimate(Acceleration.z); 

  debug(F("\nAll devices initialized successfully")); 
  setColor(Green);  
  Record.specialEventIndex = static_cast<int>(SpecialEvents::STARTUP); 
}

////    END OF SETUP FUNCTION    ////

void loop() {
  //  Delta is converted to seconds to save time   
  delta_millis_main = (end_millis_main - start_millis_main)/1000.0; 
  start_millis_main = millis();

  if ( !BMP.performReading() ) debugln(F("Failed to perform BMP reading"));

  voltage = readBatteryVoltage(); 
  altitude = (Altitude_LowPass.getEstimate(BMP.readAltitude( SEALEVELPRESSURE_HPA )) - zero_altitude);    //  in meters 
  pressure = Pressure_LowPass.getEstimate(BMP.pressure);      //  in Pascals 
  smooth_pressure = Pressure_MovAvgFilter.getEstimate(BMP.pressure); 

  IMU.refresh(); 
  IMU.getGyroscope( GyroRate ); 
  IMU.getAcceleration( Acceleration ); 

  AdjustedAcceleration.x = AccX_LowPass.getEstimate(Acceleration.x); 
  AdjustedAcceleration.y = AccY_LowPass.getEstimate(Acceleration.y); 
  AdjustedAcceleration.z = AccZ_LowPass.getEstimate(Acceleration.z); 

  //  start calculating angles and logging data at liftoff 
  if ( CurrentState.state_name != StateName::WAITING ) {
    Angle.x += (GyroRate.x) * delta_millis_main;    //  multiply angular rates by time delta 
    Angle.y += (GyroRate.y) * delta_millis_main;    //  in deg/sec * seconds 
    Angle.z += (GyroRate.z) * delta_millis_main;    //  let's get some roll data! 
  } 

////    STATE ROUTINES    ////

  switch ( CurrentState.state_name ) 
  { 
    case StateName::WAITING:     //  Pre-launch routine 
      if (CHECK_BATTERY) { 
        if (voltage < BATTERY_MIN_VOLTS) {
          debugln(F("Battery near depletion. Suspending.")); 
          powerDown(); 
        } 
      } 

      debug(F("diff-Y: ")); debug(abs(AdjustedAcceleration.y - GRAVITATIONAL_ACCEL)); 

      if ( abs(AdjustedAcceleration.y - GRAVITATIONAL_ACCEL) > IDLE_ACCELERATION_MARGIN ) {      //    detect liftoff at upward acceleration 
        CurrentState.setState(StateName::POWERED_ASCENT); 
        Record.specialEventIndex = static_cast<int>(SpecialEvents::LIFTOFF); 
        RECORDING = true; 

      } else if ( STATIC_FIRE ) {
        countdown(COUNTDOWN_TIME_SEC); 
        CurrentState.setState(StateName::POWERED_ASCENT); 
        Record.specialEventIndex = static_cast<int>(SpecialEvents::LIFTOFF); 
        RECORDING = true; 
        //  signal to light motor 
        analogWrite(PYRO_3, 255); 
        debugln("Motor ignition.");
      }
    break;


    case StateName::POWERED_ASCENT:     //  TVC routine 

      servo_x.write(SERVO_X_HOME - (pidX.compute() * GEAR_RATIO));
      servo_z.write(SERVO_Z_HOME + (pidZ.compute() * GEAR_RATIO));   //    the positive direction is flipped 

      IMU.getAccelerationMagnitude(acc_magnitude); 

      debug(F("X-Axis: ")); debug(Angle.x); debug(F(" deg \t")); debugln(pidX.getCorrection()); 
      debug(F("Z-Axis: ")); debug(Angle.z); debug(F(" deg \t")); debugln(pidZ.getCorrection());    
      debug(F("Pressure: ")); debug(pressure); debug(F(" hPa \t")); debug(F("Altitude: ")); debug(altitude); debugln(F(" m"));
      debug(F("Acc magnitude: ")); debug(acc_magnitude); debugln(F(" m/s^2"));

      //  this needs to be tested as a special event first 
      if (AdjustedAcceleration.y > (GRAVITATIONAL_ACCEL)) {
        Record.specialEventIndex = static_cast<int>(SpecialEvents::ACCEL_BURNOUT); 
      }

      if ( (millis() > (CurrentState.state_change_timestamp + MOTOR_BURN_TIME_LIMIT_MILLIS )) ) { 
        Record.specialEventIndex = static_cast<int>(SpecialEvents::TIME_BURNOUT); 

        if ( STATIC_FIRE ) {
          CurrentState.setState(StateName::DESCENT); 
          analogWrite(PYRO_3, 0); 
        } else {
          CurrentState.setState(StateName::UNPOWERED_ASCENT); 
        }
        //  center servos 
        servo_x.write(SERVO_X_HOME); 
        servo_z.write(SERVO_Z_HOME); 
        delay(200); 

      } else if ( fabs(Record.angleX) > ABORT_DEGREE_THRESHOLD || fabs(Record.angleZ) > ABORT_DEGREE_THRESHOLD ) {
        CurrentState.setState(StateName::ABORT); 
      }
    break;


    case StateName::UNPOWERED_ASCENT:     //  Coast routine 
      //  Save power by disabling TVC servos 
      detachTvcServos(); 
      
      pressure_change = Pressure_MovAvgFilter.getChange(); 
      debug ("Pressure change: \t"); debugln (pressure_change); 
      pressure_change_vector.push_back(pressure_change); 
      if (pressure_change_vector.size() > 40) { 
        pressure_change_vector.erase(pressure_change_vector.begin()); 
        //  check if total change from previous n measurements is positive 
        if ( accumulate(pressure_change_vector.begin(), pressure_change_vector.end(), 0.0) > 0.0 ) {  
          APOGEE = true; debugln(F("Ejection triggered by pressure"));  
          Record.specialEventIndex = static_cast<int>(SpecialEvents::PRESSURE_EJECT); 
        } else if ( millis() - CurrentState.state_change_timestamp > EJECTION_DELAY_MILLIS ) {
          APOGEE = true; debugln(F("Ejection triggered by delay")); 
          Record.specialEventIndex = static_cast<int>(SpecialEvents::TIME_EJECT); 
        }
      } 

      if ( APOGEE ) {   
        if (altitude > 10.0) {
          ejectParachute(PYRO_1); 
          ejectParachute(PYRO_2); 
        } 
        PARACHUTE_EJECTED = true; 
      } 

      if ( PARACHUTE_EJECTED ) {
        CurrentState.setState(StateName::DESCENT); 
      }
    break; 


    case StateName::DESCENT:     //  Parachute descent routine 
      if ( checkLanded(GyroRate) || (millis() - CurrentState.state_change_timestamp) > MAX_DESCENT_DURATION_MILLIS ) { 
        CurrentState.setState(StateName::LANDED); 
        Record.specialEventIndex = static_cast<int>(SpecialEvents::LANDING); 
      }
    break;
    
    
    case StateName::LANDED:     //  Landed routine 
      buzzerTone(2000);  
      if (LOGGING_ENABLE) { 
        RECORDING = false; 
        closeRecord();    //  closes logfile 
        copyRecordsToSD(); 
      }
      delay(1000); 
      setColor(Black); noBuzzerTone(); 
      if (SUSPEND_ENABLE) { 
        powerDown(); 
      } else { 
        debugln(F("Suspend disabled in config!")); while ( true ); 
      }
    break; 


    case StateName::ABORT: 
      debugln(F("ABORTED")); 
      servo_x.write(SERVO_X_HOME); 
      servo_z.write(SERVO_Z_HOME); 
      setColor(Red); buzzerTone(3000); 
      delay(100);
      setColor(Black); noBuzzerTone(); 
      delay(100);

      if ( millis() > (CurrentState.state_change_timestamp + MOTOR_BURN_TIME_LIMIT_MILLIS) ) {     //  check if motor has burned out 
        CurrentState.setState(StateName::UNPOWERED_ASCENT); 
      }
    break; 
  } 

////    END OF STATE ROUTINES   ////

  if ( RECORDING ) {
    //  Update  record 
    Record.recordNumber++; 
    Record.timeStamp = millis(); 
    Record.machineStateIndex = CurrentState.state_index; 
    Record.voltage = voltage; 
    Record.pressure = pressure; 
    Record.altitude = altitude; 
    Record.accelerationX = AdjustedAcceleration.x;
    Record.accelerationY = AdjustedAcceleration.y;
    Record.accelerationZ = AdjustedAcceleration.z;
    Record.rotationRateX = GyroRate.x;
    Record.rotationRateY = GyroRate.y;
    Record.rotationRateZ = GyroRate.z;
    Record.angleX = Angle.x;
    Record.angleY = Angle.y;
    Record.angleZ = Angle.z;
    Record.pidCorrectionX = pidX.getCorrection();
    Record.XpidP = pidX.getP();
    Record.XpidI = pidX.getI();
    Record.XpidD = pidX.getD();
    Record.pidCorrectionZ = pidZ.getCorrection();
    Record.ZpidP = pidZ.getP();
    Record.ZpidI = pidZ.getI();
    Record.ZpidD = pidZ.getD();

    //  Write struct to local memory 
    if (LOGGING_ENABLE) { 
      if ( !logRecordToFlash(Record) ) { debugln("Failed to write data"); }
    }
  }

  end_millis_main = millis();
  execution_time = (end_millis_main - start_millis_main); 
  loop_index ++; 
  
  //  print battery charge in % 
/*
  debug(F("Battery charge: ")); debug(map(voltage, BATTERY_MIN_VOLTS, BATTERY_MAX_VOLTS, 0, 100)); debugln(F("%")); 
  debug(F("execution time (ms): ")); debugln(execution_time);
*/ 
  debugln(); 
}

