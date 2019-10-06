/* The midiklik sysex box */

#include <LiquidCrystal.h> 
#include "Keypad.h"

const int rs = PB11, en = PB10, d4 = PB0, d5 = PB1, d6 = PC13, d7 = PC14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

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

char keyPress;
char lastKeyPress;
char sysex[14] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x01,};
char dialBuffer[6];
char rawBuffer[256]; 

int dialBufferPos = 0, rawBufferPos = 0, tags = 0, nums = 0;

#define buttonRed 10
#define buttonBlack 11
#define ledRed 12
#define ledGreen 13

int mode = 0; /* 0 = dial-mode, 1 = raw sysex hex, 2 = raw sysex dec */

void setup() 
{
  Serial.begin(9600);
  lcd.begin(16, 2);  

  pinMode(buttonRed, INPUT);
  pinMode(buttonBlack, INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  
  resetBuffer();
}

void resetBuffer()
{
  dialBufferPos = 0;
  rawBufferPos = 0;
  tags=0;
  nums=0;
  memset(dialBuffer, 0, sizeof(dialBuffer));
  memset(rawBuffer, 0, sizeof(rawBuffer));
  lcd.setCursor(0, 1); 
  lcd.print("Reset");
}

void processDialBuffer()
{
   lcd.setCursor(0, 1); 
   lcd.print("Processing");

   uint16_t cableMask = 0xFFFF; /*Get from MidiKlik */
   uint16_t jackMask = 0xFFFF; /*Get from MidiKlik */

   int styp = dialBuffer[0] == '#' ? 0 : 1;
   int ttyp = dialBuffer[3] == '#' ? 0 : 1;
   
   int src_id = ((int)(dialBuffer[1]) * 10) + (int)(dialBuffer[2]); 
   int tgt_id = ((int)(dialBuffer[4]) * 10) + (int)(dialBuffer[4]); 
   //int tgt_id = ((dialBuffer[4] - '0') * 10) + (dialBuffer[5] - '0'); 
  
   cableMask ^= !ttyp * (1UL << tgt_id);
   jackMask ^= ttyp * (1UL << tgt_id);
 
   sysex[6] = 0xFF & styp;
   sysex[7] = 0xFF & src_id;
   sysex[8] = 0xFF;
   sysex[9] = cableMask >> 8;
   sysex[10] = cableMask & 0xFF;
   sysex[11] = jackMask >> 8;
   sysex[12] = jackMask & 0xFF;
   sysex[13] = 0xF7;
  
   delay(500);  
}

void processKeypress(keyPress)
{
  
    // Start = F0, 240
    // End = F7, 247
       
    if (mode==1){
      int key = keyPress == '*'?'E':keyPress == '#'?'F':keyPress;
      rawBuffer[rawBufferPos++] = key;
    } 
    else if (mode==2){

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
    
            if (nums == 3 || (nums == 1 && tags == 0)) resetBuffers();
            if (tags == 2 && nums == 2) processDialBuffer();
            break;
    
          case '#': case '*':
            if (lastKeyPress == '#' || lastKeyPress == '*') {
                resetBuffer();
            } else {
               tags++;nums=0;
               dialBuffer[dialBufferPos++] = keyPress;
            }
            break;
    
        }

    }

    Serial.print("DIAL:");
    Serial.println(dialBuffer);
    Serial.print("RAW:");
    Serial.println(rawBuffer);
    
}

void loop() 
{
   delay(5);
   
   keyPress = customKeypad.getKey();
   
   if (keyPress){
    processKeypress(keyPress);
    lastKeyPress = keyPress; 
   }

}






  
