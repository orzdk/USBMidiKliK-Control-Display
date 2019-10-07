#include <LiquidCrystal.h> 

#include "Keypad.h"
#include "usb_midi.h"
#include "usb_midi_device.h"
#include "USBMidiKliK-Control-Display.h"

#define buttonRed 8
#define buttonBlack 9

const int rs = PB11, en = PB10, d4 = PB0, d5 = PB1, d6 = PC13, d7 = PC14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

char hexaKeys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[4] = {3, 2, 1, 0};
byte colPins[4] = {7, 6, 5, 4};

Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, 4, 4); 

char keyPress;
char lastKeyPress;
char dialBuffer[6];
char rawBuffer[256]; 

int dialBufferPos = 0, rawBufferPos = 0, tags = 0, nums = 0;
int mode = 0; /* 0 = dial-mode, 1 = raw sysex hex, 2 = raw sysex dec */

USBMidi MidiUSB;

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

void MidiUSB_sendSysEx(const uint8_t *data, size_t size)
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
      uint32_t package = ((midiData[i] | (0 << 4)) << 24) | (midiData[i+1] << 16) | (midiData[i+2] << 8) | midiData[i+3];
      midiPacket_t lpk = { .i = package };
      MidiUSB.writePacket(&(lpk.i));
      
      Serial.print("lpk.i : "); 
      Serial.print(lpk.i, HEX); 
      Serial.println("**********");      
   }  
}

void processDialBuffer()
{
   uint16_t cableMask = 0xFFFF; /*Get from MidiKlik */
   uint16_t jackMask = 0xFFFF; /*Get from MidiKlik */

   int styp = dialBuffer[0] == '#' ? 0 : 1;
   int ttyp = dialBuffer[3] == '#' ? 0 : 1;
   
   int src_id = ((int)(dialBuffer[1]) * 10) + (int)(dialBuffer[2]); 
   int tgt_id = ((int)(dialBuffer[4]) * 10) + (int)(dialBuffer[4]); 
  
   cableMask ^= !ttyp * (1UL << tgt_id);
   jackMask ^= ttyp * (1UL << tgt_id);

   uint8_t sysex[14] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x01,};
   
   sysex[6] = 0xFF & styp;
   sysex[7] = 0xFF & src_id;
   sysex[8] = 0xFF;
   sysex[9] = cableMask >> 8;
   sysex[10] = cableMask & 0xFF;
   sysex[11] = jackMask >> 8;
   sysex[12] = jackMask & 0xFF;
   sysex[13] = 0xF7;
    
   MidiUSB_sendSysEx(sysex, 14);
   resetBuffers();
}

void processKeypress()
{
    if (mode==1){
      int key = keyPress == '*'?'E':keyPress == '#'?'F':keyPress;
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

void setup() 
{
  
  //usb_midi_set_vid_pid(0x2710,0x1973);
  //usb_midi_set_product_string("Sysex-BOX");
  //MidiUSB.begin();
  
  //pinMode(buttonRed, INPUT);
  //pinMode(buttonBlack, INPUT);
 
  //lcd.begin(16, 2); 
  //lcd.setCursor(0, 1); 
  //lcd.print("Reset");  

  Serial.begin(9600);
  
}

int d;

void loop() 
{
  //delay(10);
  //d=digitalRead(8);
  //if(d==1) resetBuffers();
   
  keyPress = customKeypad.getKey();  
  if (keyPress) processKeypress(); 
}






  
