#include <JC_Button.h>
#include "Keypad.h"
#include "usb_midi.h"
#include "usb_midi_device.h"
#include "hardware_config.h"
#include "USBMidiKliK-Control-Display.h"
#include "RingBuffer.h";

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

//char hexaKeys[4][4] = {
//  {'1','2','3','A'},
//  {'4','5','6','B'},
//  {'7','8','9','C'},
//  {'*','0','#','D'}
//};
//
//byte rowPins[4] = {3, 2, 1, 0};
//byte colPins[4] = {7, 6, 5, 4};
//
//Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4); 

//#define B_RING_BUFFER_PACKET_SIZE  16*sizeof(midiPacket_t)
//RingBuffer<uint8_t,B_RING_BUFFER_PACKET_SIZE> serialRingBuffer;
//serialRingBuffer.write((uint8_t *)&pk,sizeof(midiPacket_t));
//serialRingBuffer.available();
//serialRingBuffer.readBytes(pk.packet,sizeof(midiPacket_t));

char keyPress;
char lastKeyPress;
char dialBuffer[6];
char rawBuffer[256]; 

int d;
uint8_t dialBufferPos = 0, rawBufferPos = 0, tags = 0, nums = 0;
uint8_t mode = 2; 

HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};
USBMidi MidiUSB;

const int B_RED = 7;  
const int L_RED = 6; 
const int B_BLACK = 8;  
const int L_BLACK = 9; 

ToggleButton bRed(B_RED, false, 250, false, true);
ToggleButton bBlack(B_BLACK, false, 250, false, true);

uint8_t sysexID = 3;
byte currentFlowDataType = 0x0;
char serialBuffer[128];
uint8_t serialBufferIDX = 0;
int ired,iblack;
uint8_t pkLen = 0;
byte inByte;
midiPacket_t pk { .i = 0 };

static void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo) 
{
    uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F];
    serialHw[serialNo]->write(&pk->packet[1],msgLen);
}

void SendSysexToSerialAndUSB(const uint8_t *data, size_t size)
{
    if (data == NULL || size == 0) return;

    size_t midiDataSize = (size+2)/3*4;
    uint8_t midiData[midiDataSize];
    const uint8_t *d = data;
    uint8_t *p = midiData;
    
    size_t bytesRemaining = size;

    while (bytesRemaining > 0) {
      switch (bytesRemaining) {
      case 1:
          *p++ = 5;   // SysEx ends with following single byte
          *p++ = *d;
          *p++ = 0;
          *p = 0;
          bytesRemaining = 0;
          break;
      case 2:
          *p++ = 6;   // SysEx ends with following two bytes
          *p++ = *d++;
          *p++ = *d;
          *p = 0;
          bytesRemaining = 0;
          break;
      case 3:
          *p++ = 7;   // SysEx ends with following three bytes
          *p++ = *d++;
          *p++ = *d++;
          *p = *d;
          bytesRemaining = 0;
          break;
      default:
          *p++ = 4;   // SysEx starts or continues
          *p++ = *d++;
          *p++ = *d++;
          *p++ = *d++;
          bytesRemaining -= 3;
          break;
      }
    }

    for (int i=0;i<midiDataSize;i+=4){
      uint32_t package = (midiData[i+3] << 24) | (midiData[i+2] << 16)| (midiData[i+1] << 8) | (midiData[i] | (1 << 4));
      midiPacket_t lpk = { .i = package };
      
      MidiUSB.writePacket(&(lpk.i));
      SerialWritePacket(&(lpk),0);
    }
}

void processSysexFlow(byte dataByte)
{

  Serial.print("dataByte: ");
  Serial.println(dataByte, HEX);

  pk.packet[0] = dataByte == 0xF7 ? 4+pkLen : 4;
  
  if (dataByte == 0xF7){
    
    pk.packet[0] = 5 + pkLen;  
    pk.packet[pkLen+1] = dataByte;
    Serial.print("EndPacket: ");
    Serial.println(pk.i,HEX);
    //MidiUSB.writePacket(&pk.i);
    pkLen=0;pk.i = 0;
  
  } else {
  
      pk.packet[++pkLen] = dataByte;
      
      if (pkLen == 3) {
          pk.packet[0] = 4;
          Serial.print("RunningPacket: ");
          Serial.println(pk.i, HEX);
          //MidiUSB.writePacket(&pk.i);
          pkLen=0;pk.i = 0;
      }
     
  } 
  
}

void processSerialInput(byte dataByte)
{
    
  if (dataByte == 0xFF){

    if (currentFlowDataType == 0xF0){
      
      for (int i = 0; i < serialBufferIDX; i++) Serial.print((char)serialBuffer[i]);
      Serial.println("");     
       
      memset(serialBuffer, 0, sizeof(serialBuffer));
      serialBufferIDX = 0;
      currentFlowDataType = 0x0;
    }    
   
  } else if (dataByte == 0xF0 || dataByte == 0xF1){
     currentFlowDataType = dataByte;
  
  } else{
      if (currentFlowDataType == 0xF0){
        serialBuffer[serialBufferIDX++] = dataByte;
      }
      else if (currentFlowDataType == 0xF1){
        processSysexFlow(dataByte);
      }
  }

}

void processDialBuffer()
{
   uint16_t cableMask = 0x0; /*Get from MidiKlik */
   uint16_t jackMask = 0x0; /*Get from MidiKlik */
   
   uint8_t sysex[14] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x01,};
   
   int styp = dialBuffer[0] == '#' ? 0 : 1;
   int ttyp = dialBuffer[3] == '#' ? 0 : 1;
   
   int src_id = ((int)(dialBuffer[1]) * 10) + (int)(dialBuffer[2]); 
   int tgt_id = ((int)(dialBuffer[4]) * 10) + (int)(dialBuffer[4]); 
  
   cableMask ^= !ttyp * (1UL << tgt_id);
   jackMask ^= ttyp * (1UL << tgt_id);
   
   sysex[6] = 0xFF & styp;
   sysex[7] = 0xFF & src_id;
   sysex[8] = 0x0F;
   sysex[9] = cableMask >> 8;
   sysex[10] = cableMask & 0xFF;
   sysex[11] = jackMask >> 8;
   sysex[12] = jackMask & 0xFF;
   sysex[13] = 0xF7;

   int i;
   for (i=0; i<sizeof(sysex)/sizeof(sysex[0]);i++){
      Serial.print(sysex[i], HEX);
      Serial.print(" ");
   }
   Serial.println(".......................");
   
   SendSysexToSerialAndUSB(sysex, 14);
   resetBuffers();
}

void processKeypress()
{
    if (mode==2){
      dialBuffer[dialBufferPos++] = '#';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '#';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '1';
      processDialBuffer();
    }
    else if (mode==1){
      char key = keyPress == '*'?'E':keyPress == '#'?'F':keyPress;
      rawBuffer[rawBufferPos++] = key;
    } 
    else if (mode==0){
      
        switch (keyPress)
        {
          case NO_KEY:
            break;
          
          case '0': case '1': case '2': case '3': case '4':
          case '5': case '6': case '7': case '8': case '9':
            nums++;
            dialBuffer[dialBufferPos++] = keyPress;  
            if (nums == 3 || (nums > 0 && tags == 0)) resetBuffers();
            if (tags == 2 && nums == 2) processDialBuffer();
            break;
    
          case '#': case '*':
            if (lastKeyPress == '#' || lastKeyPress == '*') {
                resetBuffers();
            } else {
               tags++;nums=0;
               dialBuffer[dialBufferPos++] = keyPress;
            }
            break;
    
        }

        lastKeyPress = keyPress;   
    }
}

void resetBuffers()
{
  dialBufferPos = 0;
  rawBufferPos = 0;
  tags=0;
  nums=0;
  keyPress = 'R';
  memset(dialBuffer, 0, sizeof(dialBuffer));
  memset(rawBuffer, 0, sizeof(rawBuffer));
}

void setup() 
{
  pinMode(B_RED, INPUT);
  pinMode(L_RED, OUTPUT); 
 
  pinMode(B_BLACK, INPUT);
  pinMode(L_BLACK, OUTPUT);
  
  bRed.begin();
  bBlack.begin();
  
  Serial.begin(9600);
  Serial3.begin(9600);
  
//  usb_midi_set_vid_pid(0x2710,0x1973);
//  usb_midi_set_product_string("SysexBOX");
//  MidiUSB.begin();
//  delay(4000);

//  serialHw[0]->begin(31250);
   
}

void loop() 
{
  delay(10);
   
//  keyPress = customKeypad.getKey();  
//  if (keyPress) processKeypress(); 

  bRed.read();
  if (bRed.changed()) {

      ired = bRed.toggleState();
      Serial.println("RED");     
      Serial.println(ired);
      digitalWrite(L_RED, ired);
      Serial3.print(0x1);
  }    

  bBlack.read();
  if (bBlack.changed()) {
    
      iblack = bBlack.toggleState();
      Serial.println("BLACK");
      Serial.println(iblack);
      digitalWrite(L_BLACK, iblack);
      Serial3.print(0x2);
  }    
  
  if (Serial3.available() > 0)
  {
    inByte = Serial3.read();
    Serial.print("inbyte: ");
    Serial.println(inByte, HEX);
    processSerialInput(inByte);   
  }

}
        



  
