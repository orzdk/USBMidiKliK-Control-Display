// Use this structure to send and receive packet to/from USB
typedef union  {
    uint32_t i;
    uint8_t  packet[4];
} midiPacket_t;