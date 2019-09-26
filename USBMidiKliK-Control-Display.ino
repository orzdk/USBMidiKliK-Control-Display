/* 
   This is the control & display slave for USBMidiKliK 
   Unit display data and sends sysex from keypad
   Unit will set PA8 (pin 29) high when it has something to say, and master will poll this
   Information will then be requested by the master
*/

#include <LiquidCrystal.h> 
#include <Wire_slave.h> 
#include "Keypad.h"

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {3, 2, 1, 0};
byte colPins[COLS] = {7, 6, 5, 4};

Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

const int rs = PB11, en = PB10, d4 = PB0, d5 = PB1, d6 = PC13, d7 = PC14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

char keyBuffer[6]; 
char keyPress;
char lastKeyPress;
char sysex[14] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x01,};

int keyBufferPos = 0, tags = 0, nums = 0;

#define interruptPin 8
int interruptPinValue;

void setup() 
{
  Wire.begin(8);  
  Wire.onReceive(onReceiveEvent);              
  Wire.onRequest(onRequestEvent);             
  
  Serial.begin(9600);
  lcd.begin(16, 2);  

  pinMode(interruptPin, OUTPUT);
  interruptPinValue = LOW;

  resetBuffer();
}

void onRequestEvent()                   
{
    Wire.write(sysex);  
    interruptPinValue = LOW;                   
}

void onReceiveEvent(int howMany) {

}

void resetBuffer()
{
  keyBufferPos = 0;
  tags=0;
  nums=0;
  memset(keyBuffer, 0, sizeof(keyBuffer));
  lcd.setCursor(0, 0); 
  lcd.print("");
}

void processBuffer()
{
   lcd.setCursor(0, 1); 
   lcd.print("Processing");

   uint16_t cableMask = 0xFFFF; /*Get from MidiKlik */
   uint16_t jackMask = 0xFFFF; /*Get from MidiKlik */

   int src_cableserial_id = ((keyBuffer[1] - '0') * 10) + (keyBuffer[2] - '0'); 
   int tgt_cableserial_id = ((keyBuffer[4] - '0') * 10) + (keyBuffer[5] - '0'); 
  
   cableMask ^= !(keyBuffer[3] - '0') * (1UL << tgt_cableserial_id);
   jackMask ^= (keyBuffer[3] - '0') * (1UL << tgt_cableserial_id);
 
   sysex[6] = 0xFF & (keyBuffer[0] - '0');
   sysex[7] = 0xFF & src_cableserial_id;
   sysex[8] = 0xFF;
   sysex[9] = cableMask >> 8;
   sysex[10] = cableMask & 0xFF;
   sysex[11] = jackMask >> 8;
   sysex[12] = jackMask & 0xFF;
   sysex[13] = 0xF7;
  
   currentPinValue = HIGH; 
   delay(500);  
}

void loop() 
{
   delay(10);
   digitalWrite(interruptPin, interruptPinValue); 
    
   keyPress = customKeypad.getKey();
   
   if (keyPress) {
      Serial.print(keyPress);
      switch (keyPress)
      {
        case NO_KEY:
          break;
        
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
          nums++;
          keyBuffer[keyBufferPos++] = keyPress;
          
          lcd.setCursor(0, 0); 
          lcd.print(keyBuffer);

          if (nums == 1 && tags == 0) resetBuffer();
          if (nums == 3) resetBuffer();
          if (tags == 2 && nums == 2) processBuffer();
          break;

        case '#':
          if (lastKeyPress == '#' || lastKeyPress == '*') {
              resetBuffer();
          } else {
             tags++;nums=0;
             keyBuffer[keyBufferPos++] = '0';
          }
          break;

        case '*':
          if (lastKeyPress == '#' || lastKeyPress == '*') {
              resetBuffer();
          } else {
            tags++;nums=0;
            keyBuffer[keyBufferPos++] = '1';
          }
     }

     lastKeyPress = keyPress;

    }

}






  
