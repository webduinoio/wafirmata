//Burn-Key 5O7D
#include <Firmata.h>

#define RST_PIN         9
#define SS_PIN          10

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL 10
// pin configured to be ignored by digitalWrite and capabilityResponse
#define PIN_MODE_IGNORE         0x7F


/*==============================================================================
   GLOBAL VARIABLES
  ============================================================================*/
#define MAXPULSE 65000
#define RESOLUTION 20
uint16_t pulsesStartPos = 0, pulsesEndPos;
const uint16_t pulsesSize = 250;
uint16_t pulses[pulsesSize][2];

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

boolean isResetting = false;


/*==============================================================================
   FUNCTIONS
  ============================================================================*/

void readAndReportData(byte address, int theRegister, byte numBytes) {
  theRegister = 0;  // fill the register with a dummy value
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
   check all the active digital inputs for change of state, then add any events
   to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
     The compiler will apply substantial optimizations if the inputs
     to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
   two bit-arrays that track Digital I/O and PWM status
*/
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == IGNORE)
    return;

  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        }
        pinConfig[pin] = ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        pinConfig[pin] = INPUT;
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    case I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = I2C;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
*/
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
   SYSEX-BASED commands
  ============================================================================*/

void pulseIR(byte pin, long microsecs) {
  cli();
  while (microsecs > 0) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);
    delayMicroseconds(10);
    microsecs -= 26;
  }
  sei();
}

boolean readSignal(int IRpin, int state) {
  uint16_t readPulse = 0;
  while (digitalRead(IRpin) == state) {
    readPulse++;
    delayMicroseconds(RESOLUTION);
    if ((readPulse >= MAXPULSE) && (pulsesStartPos != 0)) {
      return false;
    }
  }
  pulses[pulsesStartPos][state] = readPulse;
  return true;
}

void sendPulses() {
  Firmata.write(START_SYSEX);
  Firmata.write(0x04);
  Firmata.write(0x09);
  Firmata.write(0x0D);
  for (uint8_t i = 0; i < pulsesEndPos; i++) {
    String strData = toHEX(pulses[i][1] * RESOLUTION);
    strData.concat(toHEX(pulses[i][0] * RESOLUTION));
    int strLen = strData.length();
    for (int i = 0; i < strLen; i++) {
      Firmata.write(strData.charAt(i));
    }
  }
  Firmata.write(END_SYSEX);
}

String toHEX(uint16_t i) {
  if (i < 0xF) {
    return "000" + String(i, HEX);
  } else if (i < 0xFF) {
    return "00" + String(i, HEX);
  } else if (i < 0xFFF) {
    return "0" + String(i, HEX);
  } else  {
    return String(i, HEX);
  }
}

void sysexCallback(byte command, byte argc, byte *argv) {
  byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;
  byte strLen;
  String strData;
  //ir send
  byte codeType, bits;
  unsigned long sendCode = 0;
  switch (command) {
    case 4:
      switch (argv[0]) {
        case 7: //Buzzer f004070b131805f7
          pinMode(argv[1], OUTPUT);
          tone(argv[1], (String(argv[2], HEX)).toInt() * 100 +
               (String(argv[3], HEX)).toInt(), argv[4] * 100);
          break;
        case 9: //IRremote
          switch (argv[1]) {
            case 0x0A: // set & reset StartPos
              pulsesStartPos = fourChar2uint16((char*)argv, 2);
              break;
            case 0x0B: // store IR Data
              for (uint8_t i = 2; i < argc + 2; i = i + 8) {
                pulses[ pulsesStartPos + (i - 2) / 8][0] = fourChar2uint16((char*)argv, i); //puseIR
                pulses[ pulsesStartPos + (i - 2) / 8][1] = fourChar2uint16((char*)argv, i + 4); //delayTime
              }
              // fourChar2uint16
              pulsesEndPos = pulsesStartPos + (argc - 2) / 8;
              //Firmata.sendString(String(pulsesEndPos, DEC).c_str());
              Firmata.write(START_SYSEX);
              Firmata.write(0x04);
              Firmata.write(0x09);
              Firmata.write(0x0B);
              Firmata.write(END_SYSEX);
              break;
            case 0x0C: // send IR pulse
              pinMode(argv[2], OUTPUT);
              strData = "send IR signal...";
              strData.concat(pulsesEndPos);
              //Firmata.sendString(strData.c_str());
              for (uint16_t i = 0; i < pulsesEndPos; i++) {
                pulseIR(argv[2], pulses[i][0]);
                delayMicroseconds(pulses[i][1]);
              }
              Firmata.write(START_SYSEX);
              Firmata.write(0x04);
              Firmata.write(0x09);
              Firmata.write(0x0C);
              Firmata.write(END_SYSEX);
              break;
            case 0x0D:
              Firmata.sendString("wait IR signal...");
              pinMode(argv[2], INPUT);
              while (true) {
                if (!readSignal(argv[2], 1) || !readSignal(argv[2], 0)) {
                  String info = String("record:");
                  pulsesEndPos = pulsesStartPos + 1;
                  info.concat(pulsesEndPos);
                  //Firmata.sendString(info.c_str());
                  pulsesStartPos = 0;
                  sendPulses();
                  break;
                }
                pulsesStartPos++;
              }
              Firmata.sendString("wait IR signal...OK");
              break;
          }
          break;
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PWM);
          Firmata.write(8); // 8 = 8-bit resolution
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write((byte)pinConfig[pin]);
          Firmata.write((byte)pinState[pin] & 0x7F);
          if (pinState[pin] & 0xFF80) Firmata.write((byte)(pinState[pin] >> 7) & 0x7F);
          if (pinState[pin] & 0xC000) Firmata.write((byte)(pinState[pin] >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;
  }
}


/*==============================================================================
   SETUP()
  ============================================================================*/

void systemResetCallback()
{
  isResetting = true;
  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;


  /* send digital inputs to set the initial state on the host computer,
     since once in the loop(), this firmware will only send on change */
  /*
    TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
    for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
    }
  */
  isResetting = false;
}

void setup()
{
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  // to use a port other than Serial, such as Serial1 on an Arduino Leonardo or Mega,
  // Call begin(baud) on the alternate serial port and pass it to Firmata to begin like this:
  // Serial1.begin(57600);
  // Firmata.begin(Serial1);
  // then comment out or remove lines 701 - 704 below

  Firmata.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Only needed for ATmega32u4-based boards (Leonardo, etc).
  }
  systemResetCallback();  // reset to default config
}

/*==============================================================================
   LOOP()
  ============================================================================*/
void loop()
{
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
     FTDI buffer using Serial.print()  */
  checkDigitalInputs();
  /* STREAMREAD - processing incoming messagse as soon as possible, while still
     checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
  }
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

uint16_t fourChar2uint16(char* x, uint16_t startPos) {
  uint16_t v = 0;
  uint16_t endPos = startPos + 4;
  for (uint16_t i = startPos; i < endPos ; i++) {
    uint16_t now = x[i] > 57 ? x[i] - 87 : x[i] - 48;
    v = v | now << ((startPos + 3 - i) << 2);
  }
  return v;
}
