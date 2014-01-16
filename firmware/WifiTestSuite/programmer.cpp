// Atmega chip programmer
// Author: Nick Gammon
// Date: 22nd May 2012
// Version: 1.19

// Version 1.1: Reset foundSig to -1 each time around the loop.
// Version 1.2: Put hex bootloader data into separate files
// Version 1.3: Added verify, and MD5 sums
// Version 1.4: Added signatures for ATmeag8U2/16U2/32U2 (7 May 2012)
// Version 1.5: Added signature for ATmega1284P (8 May 2012)
// Version 1.6: Allow sketches to read bootloader area (lockbyte: 0x2F)
// Version 1.7: Added choice of bootloaders for the Atmega328P (8 MHz or 16 MHz)
// Version 1.8: Output an 8 MHz clock on pin 9
// Version 1.9: Added support for Atmega1284P, and fixed some bugs
// Version 1.10: Corrected flash size for Atmega1284P.
// Version 1.11: Added support for Atmega1280. Removed MD5SUM stuff to make room.
// Version 1.12: Added signatures for ATtiny2313A, ATtiny4313, ATtiny13
// Version 1.13: Added signature for Atmega8A
// Version 1.14: Added bootloader for Atmega8
// Version 1.15: Removed extraneous 0xFF from some files
// Version 1.16: Added signature for Atmega328
// Version 1.17: Allowed for running on the Leonardo, Micro, etc.
// Version 1.18: Added timed writing for Atmega8
// Version 1.19: Changed Atmega1280 to use the Optiboot loader.

/*

 Copyright 2012 Nick Gammon.


 PERMISSION TO DISTRIBUTE

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.


 LIMITATION OF LIABILITY

 The software is provided "as is", without warranty of any kind, express or implied,
 including but not limited to the warranties of merchantability, fitness for a particular
 purpose and noninfringement. In no event shall the authors or copyright holders be liable
 for any claim, damages or other liability, whether in an action of contract,
 tort or otherwise, arising from, out of or in connection with the software
 or the use or other dealings in the software.

 */

#include <SPI.h>
#include <avr/pgmspace.h>
#include "programmer.h"

//#define AVR_PROGRAMMER_DEBUG
#ifdef AVR_PROGRAMMER_DEBUG
#  define PD(x) x
#else
#  define PD(x)
#endif

// see Atmega datasheet for values
signatureType signatures[] =
{
  //     signature          description   flash size  bootloader size
  // ATtiny13 family
  {
    {
      0x1E, 0x90, 0x07     }
    , "ATtiny13A",    1 * kb,   0,
    (byte*)attiny13a_flash,// loader image
    0x0000,      // start address
    sizeof attiny13a_flash,
    16,           // page size (for committing)
    0x21,         // fuse low byte:
    0xFB,         // fuse high byte:
    0xFF,         // fuse extended byte:
    0xFF   }
  ,       // lock bits: SPM is not allowed to write to the Boot Loader section.

};

// if signature found in above table, this is its index
AVRProgrammer::AVRProgrammer(int CS, SPIClass &SPIDriver, int clockDivider) :
chipSelectPin(CS), SPI(SPIDriver), spiSpeed(clockDivider) {
  foundSig = -1;
  lastAddressMSB = 0;

  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);

  this->SPI.setClockDivider(spiSpeed);
  begin();
}

void AVRProgrammer::begin() {
  this->SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, LOW);
}


void AVRProgrammer::end() {
  this->SPI.end();
  digitalWrite(chipSelectPin, HIGH);
}

void AVRProgrammer::startProgramming() {
  byte confirm;
  pinMode(chipSelectPin, OUTPUT);

  // we are in sync if we get back programAcknowledge on the third byte
  do {
    // ensure SCK low
    digitalWrite(SCK, LOW);
    // then pulse reset, see page 309 of datasheet
    digitalWrite(chipSelectPin, HIGH);
    delay(1);  // pulse for at least 2 clock cycles
    digitalWrite(chipSelectPin, LOW);
    delay(25);  // wait at least 20 mS
    SPI.transfer(programEnable);
    SPI.transfer(programAcknowledge);
    confirm = SPI.transfer(0);
    SPI.transfer(0);
  }
  while (confirm != programAcknowledge);
  PD(Serial.println("Entered programming mode OK"));

  // we must set extended address byte to 0 to start, otherwise our writes start at 128k, not 0
  program(loadExtendedAddressByte, 0, lastAddressMSB);
}

void AVRProgrammer::getSignature() {
  foundSig = -1;
  lastAddressMSB = 0;

  byte sig[3];
  PD(Serial.print("Signature = "));
  for (byte i = 0; i < 3; i++) {
    sig[i] = program(readSignatureByte, 0, i);
    PD(showHex(sig[i]));
  }
  PD(Serial.println());

  for (int j = 0; j < NUMITEMS(signatures); j++) {
    if (memcmp(sig, signatures[j].sig, sizeof sig) == 0) {

      foundSig = j;
      PD(Serial.print("Processor = "));
      PD(Serial.println(signatures[j].desc));
      PD(Serial.print("Flash memory size = "));
      PD(Serial.print(signatures[j].flashSize, DEC));
      PD(Serial.println(" bytes."));
      if (signatures[foundSig].timedWrites) {
        PD(Serial.print("Writes are timed, not polled."));
      }
      if (strncmp(signatures[j].desc, "ATtiny", 6) == 0) {
        SPI.setClockDivider(SPI_CLOCK_DIV128); // slow down SPI for the tinys
      }
      return;
    }
  }

  PD(Serial.print("Unrecogized signature."));
}

void AVRProgrammer::getFuseBytes() {
  PD(Serial.print("LFuse = "));
  PD(showHex(program(readLowFuseByte, readLowFuseByteArg2), true));
  PD(Serial.print("HFuse = "));
  PD(showHex(program(readHighFuseByte, readHighFuseByteArg2), true));
  PD(Serial.print("EFuse = "));
  PD(showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2), true));
  PD(Serial.print("Lock byte = "));
  PD(showHex(program(readLockByte, readLockByteArg2), true));
  PD(Serial.print("Clock calibration = "));
  PD(showHex(program(readCalibrationByte), true));
}

bool AVRProgrammer::writeFuseBytes(const byte lowFuse, const byte highFuse, const byte extendedFuse, const byte lockFuse) {
  PD(Serial.println("Writing fuses..."));

  writeFuse(lowFuse, writeLowFuseByte);
  if (program(readLowFuseByte, readLowFuseByteArg2) != lowFuse) {
    PD(Serial.print("Low fuse failed to write. Expected "));
    PD(showHex(lowFuse, false, true));
    PD(Serial.print("Got "));
    PD(showHex(program(readLowFuseByte, readLowFuseByteArg2)));
    PD(Serial.println());
    return false;
  }
  else {
    PD(Serial.print("Successfully wrote low fuse: "));
    PD(showHex(program(readLowFuseByte, readLowFuseByteArg2)));
    PD(Serial.println());
  }

  writeFuse(highFuse, writeHighFuseByte);
  if (program(readHighFuseByte, readHighFuseByteArg2) != highFuse) {
    PD(Serial.print("High fuse failed to write. Expected "));
    PD(showHex(highFuse, false, true));
    PD(Serial.print("Got "));
    PD(showHex(program(readHighFuseByte, readHighFuseByteArg2)));
    PD(Serial.println());
    return false;
  }
  else {
    PD(Serial.print("Successfully wrote high fuse: "));
    PD(showHex(program(readHighFuseByte, readHighFuseByteArg2)));
    PD(Serial.println());
  }

  writeFuse(extendedFuse, writeExtendedFuseByte);
  if (program(readExtendedFuseByte, readExtendedFuseByteArg2) != extendedFuse) {
    PD(Serial.print("Extended fuse failed to write. Expected "));
    PD(showHex(extendedFuse, false, true));
    PD(Serial.print("Got "));
    PD(showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2)));
    PD(Serial.println());
    return false;
  }
  else {
    PD(Serial.print("Successfully wrote extended fuse: "));
    PD(showHex(program(readExtendedFuseByte, readExtendedFuseByteArg2)));
    PD(Serial.println());
  }

  writeFuse(lockFuse, writeLockByte);
  if (program(readLockByte, readLockByteArg2) != lockFuse) {
    PD(Serial.print("Lock fuse failed to write. Expected "));
    PD(showHex(lockFuse, false, true));
    PD(Serial.print("Got "));
    PD(showHex(program(readLockByte, readLockByteArg2)));
    PD(Serial.println());
    return false;
  }
  else {
    PD(Serial.print("Successfully wrote lock fuse: "));
    PD(showHex(program(readLockByte, readLockByteArg2)));
    PD(Serial.println());
  }

  // confirm them
  getFuseBytes();
  return true;
}

// burn the bootloader to the target device
bool AVRProgrammer::writeProgram(unsigned long loaderStart, const byte *image, const int length) {

  bool ret = true;

  if (image == 0) {
    PD(Serial.println("No bootloader support for this device."));
    return 1;
  }

  int i;
  byte lFuse = program(readLowFuseByte, readLowFuseByteArg2);

  byte newlFuse = signatures[foundSig].lowFuse;
  byte newhFuse = signatures[foundSig].highFuse;
  byte newextFuse = signatures[foundSig].extFuse;
  byte newlockByte = signatures[foundSig].lockByte;

  //unsigned long addr = signatures[foundSig].loaderStart;
  unsigned long addr = loaderStart;
  unsigned int  len = length;
  unsigned long pagesize = signatures[foundSig].pageSize;
  unsigned long pagemask = ~(pagesize - 1);
  const byte * flash = image;

  PD(Serial.print("Bootloader page size = "));
  PD(Serial.println(pagesize));
  PD(Serial.print("Bootloader address = 0x"));
  PD(Serial.println(addr, HEX));
  PD(Serial.print("Bootloader length = "));
  PD(Serial.print(len));
  PD(Serial.println(" bytes."));

  byte subcommand = 'U';

  unsigned long oldPage = addr & pagemask;

  PD(Serial.println("Writing program..."));

  for (i = 0; i < len; i += 2) {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage) {
      commitPage(oldPage);
      oldPage = thisPage;
    }
    writeFlash(addr + i, pgm_read_byte(flash + i));
    writeFlash(addr + i + 1, pgm_read_byte(flash + i + 1));
  }

  // commit final page
  commitPage(oldPage);
  PD(Serial.println("Written."));
  PD(Serial.println("Verifying ..."));

  // count errors
  unsigned int errors = 0;

  // check each byte
  for (i = 0; i < len; i++) {
    byte found = readFlash(addr + i);
    byte expected = pgm_read_byte(flash + i);
//    Serial.print("Found at 0x");
//    Serial.print(i, HEX);
//    Serial.print(": ");
//    Serial.println(readFlash(addr + i), HEX);
//    Serial.print("Expected at 0x");
//    Serial.print(i, HEX);
//    Serial.print(": ");
//    Serial.println(pgm_read_byte(flash + i), HEX);
    if (found != expected) {
      if (errors <= 100) {
        PD(Serial.print("Verification error at address ");
        PD(showHex(addr + i, false, true)));
        PD(Serial.print(": Got: "));
        PD(showHex(found));
        PD(Serial.print(" Expected: "));
        PD(showHex(expected, true));
      }
      errors++;
    }
  }

  if (errors == 0) {
    PD(Serial.println("No errors found."));
    ret = true;
  } else {
    ret = false;
    PD(Serial.print(errors, DEC));
    PD(Serial.println(" verification error(s)."));
    if (errors > 100) {
      PD(Serial.print("First 100 shown."));
    }
    //return;  // don't change fuses if errors
  }
  PD(Serial.println("Done."));
  return ret;
}

bool AVRProgrammer::writeProgramFromSerialFlash(uint32_t loaderStart, FlashClass *flash, const uint32_t flashAddress, const uint32_t length) {

  bool ret = true;

  uint32_t i;
  uint32_t addr = loaderStart;
  uint32_t  len = length;
  uint32_t pagesize = signatures[foundSig].pageSize;
  uint32_t pagemask = ~(pagesize - 1);

  uint32_t timesThrough = 0;
  uint16_t bufCtr = 0;
  uint16_t verifyCtr = 0;
  uint16_t bufLen = 16384;
  byte flashBuffer[bufLen];

  // count errors
  unsigned int errors = 0;

  PD(Serial.print("Bootloader page size = "));
  PD(Serial.println(pagesize));
  PD(Serial.print("Bootloader address = 0x"));
  PD(Serial.println(addr, HEX));
  PD(Serial.print("Bootloader length = "));
  PD(Serial.print(len));
  PD(Serial.println(" bytes"));

  byte subcommand = 'U';

  unsigned long oldPage = addr & pagemask;

  PD(Serial.println("Writing program..."));

  // load each word
  for (i=0; i<len; i+=2, bufCtr+=2) {

    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage) {
      commitPage(oldPage);
      oldPage = thisPage;
    }

    if (i % bufLen == 0) {
      // read new chunk of bytes into buffer
      bufCtr = 0;
      flash->read(flashAddress + i, &flashBuffer, bufLen);
      startProgramming();

      timesThrough++;
      PD(Serial.print("Writing "));
      if ((timesThrough * bufLen) > len) {
        PD(Serial.print(len));
      }
      else {
        PD(Serial.print(timesThrough * bufLen));
      }
      PD(Serial.print(" bytes of a total of "));
      PD(Serial.println(len));
    }

    writeFlash(addr + i, flashBuffer[bufCtr]);
    writeFlash(addr + i + 1, flashBuffer[bufCtr + 1]);
  }

  // commit final page
  commitPage(oldPage);

  PD(Serial.println("Written."));

  // check each byte
  for (i=0; i<len; i++, bufCtr++) {
    if (i % bufLen == 0) {
      PD(Serial.print("Verifying flash at address: 0x"));
      PD(Serial.println(loaderStart + i, HEX));
      bufCtr = 0;
      flash->read(flashAddress + i, &flashBuffer, bufLen);
      startProgramming();
    }

    byte found = readFlash(addr + i);
    if (found != flashBuffer[bufCtr]) {
      if (errors <= 10) {
        PD(Serial.print("Verification error at address 0x"));
        PD(Serial.print(addr + i, HEX));
        PD(Serial.print(": Got: "));
        PD(showHex(found, false, true));
        PD(Serial.print("Expected: "));
        PD(showHex(flashBuffer[bufCtr], false, true));
        PD(Serial.println(bufCtr));
      }
      errors++;
    }
  }

  if (errors == 0) {
    PD(Serial.println("No errors found."));
    ret = true;
  }
  else {
    ret = false;
    PD(Serial.print(errors, DEC));
    PD(Serial.println(" verification error(s)."));
    if (errors > 100) {
      PD(Serial.print("First 100 shown."));
    }
  }
  flash->end();
  PD(Serial.print("Done."));
  return ret;
}

void AVRProgrammer::readProgram(uint32_t address, uint32_t length) {
  PD(Serial.println());
  PD(Serial.print("Reading "));
  PD(Serial.print(length));
  PD(Serial.print(" bytes of program memory starting at address 0x"));
  PD(Serial.println(address, HEX));

  for (uint32_t i = 0; i < length; i++) {
    // show address
    if (i % 16 == 0) {
      PD(Serial.print(address + i, HEX));
      PD(Serial.print(": "));
    }
    PD(showHex(readFlash(address + i)));
    // new line every 16 bytes
    if (i % 16 == 15) {
      PD(Serial.println());
    }
  }
  PD(Serial.println());
}

uint8_t AVRProgrammer::readEeprom(unsigned long addr) {
  PD(Serial.println("Reading EEPROM..."));
  return program(readEepromMemory, highByte(addr), lowByte(addr));
}

void AVRProgrammer::writeEeprom(unsigned long addr, byte value) {
  PD(Serial.println("Writing EEPROM..."));
  program(writeEepromMemory, highByte(addr), lowByte(addr), value);
  pollUntilReady();
}

void AVRProgrammer::eraseChip() {
  PD(Serial.println("Erasing chip..."));
  program(programEnable, chipErase);
  delay(20);  // for Atmega8
  pollUntilReady();
}

bool AVRProgrammer::foundSignature() {
  return foundSig;
}

// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte AVRProgrammer::program(const byte b1, const byte b2, const byte b3, const byte b4) {
  byte ret;
  this->SPI.transfer(b1);
  this->SPI.transfer(b2);
  this->SPI.transfer(b3);
  ret = this->SPI.transfer(b4);
  return ret;
}

// read a byte from flash memory
byte AVRProgrammer::readFlash(unsigned long addr) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

  return program(readProgramMemory | high, highByte (addr), lowByte (addr));
}

// write a byte to the flash memory buffer (ready for committing)
byte AVRProgrammer::writeFlash(unsigned long addr, const byte data) {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  //  Serial.println("writeFlash: ");
  //  Serial.print("high byte: ");
  //  Serial.println(high);
  //  Serial.print("low byte: ");
  //  Serial.println(lowByte(addr));
  //  Serial.print("sending the following command: 0x");
  //  Serial.print(loadProgramMemory | high, HEX);
  //  Serial.print(", 0, 0x");
  //  Serial.print(lowByte(addr), HEX);
  //  Serial.print(", 0x");
  //  Serial.println(data, HEX);
  program(loadProgramMemory | high, 0, lowByte (addr), data);
}

// convert a boolean to Yes/No
void AVRProgrammer::showYesNo(const boolean b, const boolean newline) {
  if (b) {
    PD(Serial.print("Yes"));
  }
  else {
    PD(Serial.print("No"));
  }
  if (newline) {
    PD(Serial.println());
  }
}

// poll the target device until it is ready to be programmed
void AVRProgrammer::pollUntilReady() {
  if (signatures[foundSig].timedWrites) {
    delay (10);  // at least 2 x WD_FLASH which is 4.5 mS
  }
  else {
    while ((program(pollReady) & 1) == 1) {
    }  // wait till ready
  }
}

// commit page
void AVRProgrammer::commitPage(unsigned long addr) {
  //Serial.print("Committing page starting at 0x");
  //Serial.print(addr, HEX);

  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB) {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }

  //  Serial.println("commitPage: ");
  //  Serial.print("sending the following command: 0x");
  //  Serial.print(writeProgramMemory, HEX);
  //  Serial.print(", 0, 0x");
  //  Serial.print(highByte(addr), HEX);
  //  Serial.print(", 0x");
  //  Serial.println(lowByte(addr), HEX);

  program(writeProgramMemory, highByte(addr), lowByte(addr));
  pollUntilReady();
}

// write specified value to specified fuse/lock byte
void AVRProgrammer::writeFuse(const byte newValue, const byte instruction) {
  if (newValue == 0) {
    return;  // ignore
  }

  program(programEnable, instruction, 0, newValue);
  pollUntilReady();
}

// show a byte in hex with leading zero and optional newline
void AVRProgrammer::showHex(const byte b, const boolean newline, const boolean show0x) {
  if (show0x) {
    PD(Serial.print("0x"));
  }
  // try to avoid using sprintf
  char buf[4] = {
    ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', ' ' , 0     };
  if (buf[0] > '9') {
    buf[0] += 7;
  }
  if (buf[1] > '9') {
    buf[1] += 7;
  }
  PD(Serial.print(buf));
  if (newline) {
    PD(Serial.println());
  }
}



