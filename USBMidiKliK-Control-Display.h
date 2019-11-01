#ifndef _USBMIDIKLIK4X4_H_
#define _USBMIDIKLIK4X4_H_
#pragma once

typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} midiPacket_t;

static const uint8_t CINToLenTable[] = {
  0,
  0,
  2, // 0x02
  3, // 0x03
  3, // 0x04
  1, // 0x05
  2, // 0x06
  3, // 0x07
  3, // 0x08
  3, // 0x09
  3, // 0x0A
  3, // 0x0B
  2, // 0x0C
  2, // 0x0D
  3, // 0x0E
  1  // 0x0F
};

char hexaKeys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};




#endif
