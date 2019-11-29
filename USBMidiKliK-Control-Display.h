#ifndef _USBMIDIKLIK4X4_H_
#define _USBMIDIKLIK4X4_H_
#pragma once

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

char hexaKeys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

#endif
