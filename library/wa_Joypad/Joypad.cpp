#include <Joypad.h>
#include <Firmata.h>

Joypad::Joypad() {
}

void Joypad::init(byte p0,byte p1){
  pin0 = p0;
  pin1 = p1;
  pin2 = -1;
  pin3 = -1;
}
void Joypad::init(byte p0,byte p1,byte p2,byte p3){
  pin0 = p0;
  pin1 = p1;
  pin2 = p2;
  pin3 = p3;
}

void Joypad::start(){
  isRunning = true;
}

void Joypad::stop(){
  isRunning = false;
}

boolean Joypad::state(){
  return isRunning;
}

void Joypad::loop() {
  detect();
}  

void Joypad::detect() {
  int xVal = 0, yVal = 0;
  unsigned long nowTime = millis();
  if (nowTime - lastTime > 10) {
    lastTime = nowTime;
    xVal = (analogRead(pin0) >> 8) - 1;
    switch (xVal) {
      case 0:
        bits = bits & B11111100;
        break;
      case -1:
        bits = bits | B00000001;
        break;
      case 1:
      case 2:
        bits = bits | B00000010;
    }
    yVal = (analogRead(pin1) >> 8) - 1;
    switch (yVal) {
      case 0:
        bits = bits & B11110011;
        break;
      case -1:
        bits = bits | B00000100;
        break;
      case 1:
      case 2:
        bits = bits | B00001000;
    }
    //btn
    if(pin2>=0){
      if (analogRead(pin2) >> 8 == 0) {
        bits = bits & B00101111;
      } else {
        bits = bits | B00010000;
      }
    }
    //btn
    if(pin3>=0){
      if ( analogRead(pin3) >> 8 == 0) {
        bits = bits & B00011111;
      } else {
        bits = bits | B00100000;
      }
    }
    if (lastBits != bits) {
      lastBits = bits;
      output(lastBits);
    }
  }
}

void Joypad::output(byte lastBits){
  Firmata.write(START_SYSEX);
  Firmata.write(4);
  Firmata.write(20);
  Firmata.write(pin0);
  Firmata.write(pin1);
  Firmata.write(lastBits);
  Firmata.write(END_SYSEX);
}
