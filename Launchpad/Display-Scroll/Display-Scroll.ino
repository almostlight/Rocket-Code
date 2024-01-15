#include <Wire.h> 
#include <Adafruit_GFX.h> 
#include "Adafruit_LEDBackpack.h" 

//  Setup devices 
Adafruit_AlphaNum4 alphanum = Adafruit_AlphaNum4();

void setup() {
  Serial.begin(9600);
  alphanum.begin(0x70); 
  alphanum.writeDigitRaw(0, 0xFFFF);
  alphanum.writeDigitRaw(1, 0xFFFF);
  alphanum.writeDigitRaw(2, 0xFFFF);
  alphanum.writeDigitRaw(3, 0xFFFF);
  alphanum.writeDisplay();
  delay(5000);
  alphanum.clear();
  alphanum.writeDisplay();
}

// scroll from serial 
char displaybuffer[4] = {' ', ' ', ' ', ' '};
void displayFromSerial(Adafruit_AlphaNum4 displayName) {
  if(Serial.available()>0) {
    char c = Serial.read(); 
    while(isprint(c)) {
      //  scroll down display
      displaybuffer[0] = displaybuffer[1];
      displaybuffer[1] = displaybuffer[2];
      displaybuffer[2] = displaybuffer[3];
      displaybuffer[3] = c;
      //  set every digit to the buffer
      displayName.writeDigitAscii(0, displaybuffer[0]);
      displayName.writeDigitAscii(1, displaybuffer[1]);
      displayName.writeDigitAscii(2, displaybuffer[2]);
      displayName.writeDigitAscii(3, displaybuffer[3]);
      //  print  
      displayName.writeDisplay();
      delay(200);
      c = Serial.read();
    }
    Serial.println("Printed\n");
    delay(1000);
    displayName.clear();
    displayName.writeDisplay();
  }
}

void loop() {
  displayFromSerial(alphanum); 
}
