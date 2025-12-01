#ifndef RFID_READER_H
#define RFID_READER_H

#include <Arduino.h>

bool compareUID(byte *uid, byte *knownUID, byte size);
void rfid_setup();
uint8_t getRFIDTagValue(void);

#endif
