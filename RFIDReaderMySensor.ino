/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 nonobike and pipersw from Jeedom forum

   DESCRIPTION
   RFID Reader

   Use the SPI or I2C wiring option for your RFID module

   Use soft spi wiring for NRF24L01 radio if RFID module in SPI wiring

   http://www.mysensors.org/build/rfid
*/
// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// because the SPI configuration for NRF and RFID is different
//(NRF : SPI_MODE0, MSB first and 2MHz, PN532: SPI_MODE0, LSB first and 1MHz)

// The W5100 ethernet module allow built-in soft spi changes
// so we call it event if we don't use it.
//#if !defined(MY_W5100_SPI_EN) && !defined(ARDUINO_ARCH_SAMD)
//#define MY_SOFTSPI
//#define MY_SOFT_SPI_SCK_PIN 14
//#define MY_SOFT_SPI_MISO_PIN 16
//#define MY_SOFT_SPI_MOSI_PIN 15
//#endif

// When W5100/PN532 is connected we have to move CE/CSN pins for NRF radio
//#ifndef MY_RF24_CE_PIN
//#define MY_RF24_CE_PIN 5
//#endif
//#ifndef MY_RF24_CS_PIN
//#define MY_RF24_CS_PIN 6
//#endif


#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  13
#define PN532_MOSI 11
#define PN532_SS   10
#define PN532_MISO 12

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
// Use of HW WIRE I2C pins A4 SCL and A5 SDA
#define PN532_IRQ   9
#define PN532_RESET 8  //Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a SPI connection:
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// Add your valid rfid keys here. To find you your key just run sketch; hold your new RFID tag in fron ot the reader;
// and copy the key from serial output of this sketch.
//const uint8_t maxKeyLength = 7;
//uint8_t validKeys[][maxKeyLength] = {
//  { 0xB3, 0xC6, 0xD9, 0x80, 0x00, 0x00, 0x00 },
//  { 0, 0, 0, 0, 0, 0, 0 },    // ADD YOUR KEYS HERE!
//  { 0, 0, 0, 0, 0, 0, 0 }
//};
//int keyCount = sizeof validKeys / maxKeyLength;

#define CHILD_ID0 1
#define CHILD_ID1 2
#define CHILD_ID2 3
#define CHILD_ID3 4
#define CHILD_ID4 5

MyMessage msg0(CHILD_ID0, V_VAR1);
MyMessage msg1(CHILD_ID1, V_VAR2);
MyMessage msg2(CHILD_ID2, V_VAR3);
MyMessage msg3(CHILD_ID3, V_VAR4);
MyMessage msg4(CHILD_ID4, V_VOLTAGE);

const int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
const unsigned long SLEEP_TIME = 1000;  // sleep time between reads in ms

void before() {

}

void setup() {

  // use the 1.1 V internal reference
  analogReference(INTERNAL);

  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to one.
  DIDR0 = DIDR0 | 0x3F;

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print(F("Couldn't find PN53x board"));
    while (1); // halt
  }
  Serial.print(F("Found NFC chip PN5")); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print(F("Firmware ver. ")); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print(F(".")); Serial.println((versiondata >> 8) & 0xFF, DEC);
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);

  // configure board to read RFID tags
  nfc.SAMConfig();

  Serial.println(F("Waiting for an ISO14443A card"));
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("RFID Reader", "1.0");

  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID0, S_CUSTOM);
  present(CHILD_ID1, S_CUSTOM);
  present(CHILD_ID2, S_CUSTOM);
  present(CHILD_ID3, S_CUSTOM);
  present(CHILD_ID4, S_POWER);
}

void loop() {

  boolean success;
  uint8_t key[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t currentKeyLength;
  // Length of the UID (4 or 7 bytes depending on ISO14443A card type)


  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &key[0], &currentKeyLength);

  if (success) {
#ifdef MY_DEBUG
    Serial.println(F("Found tag id: "));
#endif
    for (uint8_t i = 3; i < currentKeyLength; i++)
    {
#ifdef MY_DEBUG
      Serial.println(key[0]);
#endif
      send(msg0.set(key[0]));
      delay(3);
#ifdef MY_DEBUG
      Serial.println(key[1]);
#endif
      send(msg1.set(key[1]));
      delay(3);
#ifdef MY_DEBUG
      Serial.println(key[2]);
#endif
      send(msg2.set(key[2]));
      delay(3);
#ifdef MY_DEBUG
      Serial.println(key[3]);
#endif
      send(msg3.set(key[3]));
    }
#ifdef MY_DEBUG
    Serial.println(F("Battery voltage: "));
#endif

    //measure battery voltage with internal ref: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
    float Vbattery = readVcc();
#ifdef MY_DEBUG
    Serial.println(Vbattery);
#endif
    send(msg4.set(Vbattery, 3));

    // Wait for card/tag to leave reader
    while (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &key[0], &currentKeyLength));
  }
  else
  {
    // PN532 probably timed out waiting for a card
    Serial.println(F("Timed out waiting for a card"));
  }

#ifdef MY_DEBUG
  Serial.println(F("Go to sleep"));
#endif
  //begin to sleep PN532:
  /* not implemented:
    In case LowVbat functionality is required
    The interface pins will be used to achieve LowVbat mode. Therefore they must be connected to the host
    controller.
    Before switching off the host controller, change I0 to 1 and I1 to 0 (this put the PN532 in I2C
    configuration)
    Host sends a reset pulse (minimum 20ns, see datasheet p209) to PN532 via RSTPD_N
    Wait a time off (2ms, see datasheet p209)
    The PN532 will go in LowVbat mode and stays in this mode (25µA)
  */
  //begin to sleeping cpu:
  //disable ADC by setting ADEN bit to 0
  ADCSRA &= ~(1 << ADEN); // adc off

  // modification of internal clock : 16/8 MHz --> 1 MHz (see datasheet ATMEL section 9.12.2)
  int oldClkPr = CLKPR;  // save the clock prescaler register CLKPR
  CLKPR = 0x80;          // bit 7 of CLKPR set to 1 and others to 0 for signal a prescaler change
#if F_CPU == 8000000UL //for Arduino 8Mhz
  CLKPR = 0x03;          // clock divider by 8 of 8MHz clock --> 1MHz
#elif F_CPU == 16000000UL //for Arduino 16MHz
  CLKPR = 0x04;          // clock divider by 16 of 16MHz clock --> 1MHz
#endif
  // go to sleep cpu
  sleep(SLEEP_TIME);

  //wake up cpu:
  // Restauration of original frequency after wake up
  CLKPR = 0x80;          // bit 7 of CLKPR to 1 and others to 0 for signaling a prescaler change
  CLKPR = oldClkPr;      // restore the old prescaler
  ADCSRA |= (1 << ADEN); // adc on
  //wake up PN532:
  /* not implemented
    To wake up the PN532 (to exit LowVbat mode) and recover SPI communication
    Host controller change I0 to 0 and I1 to 1 (restore SPI configuration)
    Host controller sends a reset pulse (minimum 20ns) to PN532 via RSTPD_N
    Wait a time off (2 ms)
    Host controller sets NSS wake-up (high to low, CSN)
  */
#ifdef MY_DEBUG
  Serial.println(F("Wake Up"));
#endif
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  //  if (message.type == V_LOCK_STATUS) {
  //  }
}

float readVcc(void) {

#define ADMUX_VCCWRT1V1 (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))

  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  if (ADMUX != ADMUX_VCCWRT1V1)
  {
    ADMUX = ADMUX_VCCWRT1V1;

    // Bandgap reference start-up time: max 70us
    // Wait for Vref to settle.
    delayMicroseconds(350);
  }

  // Start conversion and wait for it to finish.
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)) {}; // measuring

  // Result is now stored in ADC.

  // Calculate Vcc (in V)
  float vcc = 1.1 * 1023.0 / ADC;

  return vcc; //in mV
}


