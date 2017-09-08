
#include <Firmata.h>
#include <waLCD1602.h>

#define SENSOR_LCD1602  0x18

waLCD1602* lcd;

void sensorsSetup(){
    setup_LCD1602();
}
void setup_LCD1602() {

}

void sensorsLoop() {
    loop_LCD1602();
}
void loop_LCD1602() {

}

void waSysexCallback(byte sensorType, byte argc, byte *argv) {
  switch (sensorType) {      
    case SENSOR_LCD1602:
      case_LCD1602(argc, argv);
    break;

  }
}

void publish(String topic, String strData) {
  byte topicLen = topic.length();
  byte strLen = strData.length();
  Firmata.write(START_SYSEX);
  Firmata.write(0x03);
  Firmata.write(0x06 /* publishTo */);
  for (byte i = 0; i < topicLen; i++) {
    Firmata.write(topic.charAt(i));
  }
  Firmata.write(0x20);
  for (byte i = 0; i < strLen; i++) {
    Firmata.write(strData.charAt(i));
  }
  Firmata.write(END_SYSEX);
}

void case_LCD1602(byte argc, byte *argv) {
  switch (argv[1]) {
  case 0: /* LCD begin */
    if (lcd == NULL) {
      lcd = new waLCD1602(argv[2], argv[3], 0x3F);
      lcd->begin();
    }
    break;
  case 1: /* setCursor */
    lcd->setCursor(argv[2], argv[3]);
    break;
  case 2: /* print */
    lcd->print(toString(argv + 2, argc));
    break;
  case 3: /* clear */
    lcd->clear();
    break;
  }
}

String toString(byte *array, byte argc) {
  String data = "";
  for (int i = 0; i < argc - 2 ; i += 2) {
    data.concat((char)asc2hex(array + i));
  }
  return data;
}

byte asc2hex(byte * array) {
  byte b = 0;
  if (*array >= 0x41 && *array <= 0x66) {
    b = (*array & 0x0F) + 9 << 4;
  } else if (*array >= 0x30 && *array <= 0x39) {
    b = (*array - 0x30) << 4;
  }
  array++;
  if (*array >= 0x41 && *array <= 0x66) {
    b |= (*array & 0x0F) + 9;
  } else if (*array >= 0x30 && *array <= 0x39) {
    b |= (*array - 0x30);
  }
  return b;
}

