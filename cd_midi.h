#ifndef _CD_MIDI_H_
#define _CD_MIDI_H_
#pragma once

HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} __packed midiPacket_t;

const uint8_t CINToLenTable[] =
{
  0, // 0X00 Miscellaneous function codes. Reserved for future extensions.
  0, // 0X01 Cable events.Reserved for future expansion.
  2, // 0x02 Two-byte System Common messages like  MTC, SongSelect, etc.
  3, // 0x03 Three-byte System Common messages like SPP, etc.
  3, // 0x04 SysEx starts or continues
  1, // 0x05 Single-byte System Common Message or SysEx ends with following single byte.
  2, // 0x06 SysEx ends with following two bytes.
  3, // 0x07 SysEx ends withfollowing three bytes.
  3, // 0x08 Note-off
  3, // 0x09 Note-on
  3, // 0x0A Poly-KeyPress
  3, // 0x0B Control Change
  2, // 0x0C Program Change
  2, // 0x0D Channel Pressure
  3, // 0x0E PitchBend Change
  1  // 0x0F Single Byte
};

void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo)
{
  uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F];
  serialHw[serialNo]->write(&pk->packet[1], msgLen);
}

#endif
