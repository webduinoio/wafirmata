#include "WAEEPROM.h"
#include "EEPROM.h"

const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;
const int BUFSIZE = 64;
const int SSID_ADDR = 1;
const int PWD_ADDR = 50;
const int SERVER_ADDR = 100;
const int SERVER_PORT_ADDR = 150;
char buf[BUFSIZE];

byte WAEEPROM::eeprom_read_string(int addr, char* buffer, int bufSize) {
  byte ch;
  int bytesRead;
  if (bufSize == 0) {
    return 0;
  }
  if (bufSize == 1) {
    buffer[0] = 0;
    return 1;
  }
  bytesRead = 0;
  ch = EEPROM.read(addr + bytesRead);
  buffer[bytesRead] = ch;
  bytesRead++;
  while ( (ch != 0x00) && (ch != 0xFF) && (bytesRead < bufSize) && ((addr + bytesRead) <= EEPROM_MAX_ADDR) ) {
    ch = EEPROM.read(addr + bytesRead);
    buffer[bytesRead] = ch;
    bytesRead++;
  }
  if ((ch != 0x00) && (bytesRead >= 1)) {
    buffer[bytesRead - 1] = 0;
  }
  if (ch == 0xFF) {
    return 0;
  }
  return bytesRead;
}

void WAEEPROM::clearROMBuffer() {
  int i;
  for (i = 0; i < 512; i++) {
    EEPROM.write(i, 0xFF);
  }
}

String WAEEPROM::readFromROM(int idx) {
  byte len = eeprom_read_string(idx, buf, BUFSIZE);
  String str = String(buf);
  return str.substring(0, len);
}

void WAEEPROM::writeToROM(int idx, String str) {
  int i;
  int len = str.length();
  for (i = 0; i < len; i++) {
    EEPROM.write(i + idx, str.charAt(i));
  }
  EEPROM.write(i + idx, 0);
}

String WAEEPROM::readString(int addr) {
  return readFromROM(addr);
}

void WAEEPROM::writeString(int addr, String str) {
  writeToROM(addr, str);
}

byte WAEEPROM::readByte(int addr) {
  return EEPROM.read(addr);
}

void WAEEPROM::writeByte(int addr, byte b) {
  EEPROM.write(addr,b);
}

int WAEEPROM::readInt(int addr) {
  int i = EEPROM.read(addr);
  i |= EEPROM.read(addr+1) << 8;
  return i;
}

void WAEEPROM::writeInt(int addr, int i) {
  EEPROM.write(addr,i&0xff);
  EEPROM.write(addr+1,i >> 8 & 0xff);
}

//
// END OF FILE
//