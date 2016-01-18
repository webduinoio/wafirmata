#ifndef waeeprom_h
#define waeeprom_h

#include "Arduino.h"

class WAEEPROM {

public:
	String readString(int addr);
	void writeString(int addr, String str);
	byte readByte(int addr);
	void writeByte(int addr, byte b);
	int readInt(int addr);
	void writeInt(int addr, int b);
	void clearROMBuffer();
private:
	void writeToROM(int idx, String str);
	String readFromROM(int idx);
	byte eeprom_read_string(int addr, char* buffer, int bufSize);
};

#endif
//
// END OF FILE
//
