#ifndef joypad_h
#define joypad_h
#include "Arduino.h"

class Joypad {
public:
	Joypad();
	void init(byte pin0,byte pin1);
	void init(byte pin0,byte pin1,byte pin2,byte pin3);
	void start();
	void stop();
	void loop();
	boolean state();
private:
	void detect();
	void output(byte data);
	byte pin0 = -1 , pin1 = -1,pin2 = -1,pin3 = -1;
	boolean isRunning = false;
	byte lastBits = 0, bits = 0;
	unsigned long lastTime = 0;
};

#endif
