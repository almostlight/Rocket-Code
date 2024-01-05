#include <Wire.h> 
#include <SPI.h> 
#include <Servo.h>
#include <Adafruit_GFX.h> 
#include "Adafruit_LEDBackpack.h" 
#include "config.h"

//  Setup devices 
Adafruit_AlphaNum4 alphanum = Adafruit_AlphaNum4();
bool ignited = false; 
bool countdown = false;
Servo clamp_servos;

void setup() {
  pinMode(CLAMP_SERVO_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(IGNITER_POWER, OUTPUT);
  pinMode(SWITCH_OUTPUT, OUTPUT);
  pinMode(SWITCH_INPUT, INPUT);

  noTone(BUZZER); 
  digitalWrite(SWITCH_OUTPUT, HIGH);        //  set switch power to high 
  while (digitalRead(SWITCH_INPUT)==1);     //  if swith is high at startup, wait for state change 
  Serial.begin(9600);
  alphanum.begin(0x70); 

  tone(BUZZER, 2800); 
  alphanum.writeDigitRaw(0, 0xFFFF);
  alphanum.writeDigitRaw(1, 0xFFFF);
  alphanum.writeDigitRaw(2, 0xFFFF);
  alphanum.writeDigitRaw(3, 0xFFFF);
  alphanum.writeDisplay();
  delay(2000);
  alphanum.clear();
  alphanum.writeDisplay();
  noTone(BUZZER); 

  clamp_servos.attach(CLAMP_SERVO_PIN); 
  clamp_servos.write(EXTENDED); 
  delay(800); 
  clamp_servos.detach();      //  stop servos from wasting power by trying to correct 
  delay(200); 
  clamp_servos.attach(CLAMP_SERVO_PIN);        //  reattach 
}

void displayWrite(Adafruit_AlphaNum4 displayName, String text) {
  if(isprint(text[0])) displayName.writeDigitAscii(0, text[0]); 
  if(isprint(text[1])) displayName.writeDigitAscii(1, text[1]); 
  if(isprint(text[2])) displayName.writeDigitAscii(2, text[2]); 
  if(isprint(text[3])) displayName.writeDigitAscii(3, text[3]);
  displayName.writeDisplay();
}

void displayClear(Adafruit_AlphaNum4 displayName) {
  displayName.clear();
  displayName.writeDisplay(); 
}


int readVoltage(int pin) {
  int converted = map(analogRead(pin), 0, 1023, 0, 3300);
  return converted; 
}

void loop() {
  int i = COUNTDOWN_LENGTH; 
  int a = 500; 
  Serial.print("\nswitch: "); Serial.print(digitalRead(SWITCH_INPUT));

  while(digitalRead(SWITCH_INPUT)==1 && ignited==false) { 
    countdown = true; 
    Serial.print("\nT - "); Serial.print(i);
    displayWrite(alphanum, String(i));

    tone(BUZZER, (1800+a)); 
    a=-a;

    if (i==0) {
      if (PYROS_ENABLE) {
        Serial.println("\nMOTOR IGNITION"); 
        digitalWrite(IGNITER_POWER, HIGH); 
        clamp_servos.write(RETRACTED); 
        delay(IGNITION_LENGTH); 
        digitalWrite(IGNITER_POWER, LOW);
      }
      ignited=true; 
      noTone(BUZZER);
      displayWrite(alphanum, String("LIFT")); delay(2000); 
      displayWrite(alphanum, String("OFF")); delay(2000);
      displayClear(alphanum); 
      break;
    }
    delay(1000); 
    i--; 
    Serial.print("\nswitch: "); Serial.print(digitalRead(SWITCH_INPUT));
  }
  noTone(BUZZER); 
  if (ignited==false && countdown==true) {
    Serial.println("\nLAUNCH ABORTED"); 
    displayWrite(alphanum, String("ABRT"));
    while (digitalRead(SWITCH_INPUT)==0);     //  wait for switch high state 
  } else if (ignited==true){
    Serial.println("\nLAUNCHED. Suspending to save power."); 
    digitalWrite(SWITCH_OUTPUT, LOW); 
    clamp_servos.detach();      //  turn off servos 
    while (1);      //  simulate program end 
  }
  delay(500);
}

