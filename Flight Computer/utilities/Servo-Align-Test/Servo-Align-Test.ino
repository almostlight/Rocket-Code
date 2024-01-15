/*
 *
 * Created by almostlight on Nov 1, 2022 
 *
 * https://github.com/almostlight 
 * 
 */

#define SERVO_MIN     1000      //  chrono values 
#define SERVO_MAX     2000 
#define TVC_X   5
#define TVC_Y   4
#include <Servo.h>
Servo servo_x; 
Servo servo_z; 

float angle_x = 92.0; 
float angle_z = 82.5; 

void setup() {
  servo_x.attach(TVC_X, SERVO_MIN, SERVO_MAX);
  servo_z.attach(TVC_Y, SERVO_MIN, SERVO_MAX);  
  servo_x.write( angle_x ); 
  servo_z.write( angle_z ); 
  Serial.begin(9600);
  while (!Serial);
}

int n = 3; 
void loop() {
  for ( int i=-n; i<=n; i++ ) {
    float angle = angle_x+i; 
    servo_x.write( angle ); 
    Serial.print( "x: " ); Serial.println( angle ); 
    delay(2000); 
  }
  servo_x.write( angle_x ); 
  servo_z.write( angle_z ); 

  for ( int i=-n; i<=n; i++ ) {
    float angle = angle_z+i; 
    servo_z.write( angle ); 
    Serial.print( "z: " ); Serial.println( angle ); 
    delay(2000); 
  }
  servo_x.write( angle_x ); 
  servo_z.write( angle_z ); 
  
  //while (true); 
}

