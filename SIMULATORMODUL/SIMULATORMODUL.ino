
//#include <avr/io.h>
//#include <avr/interrupt.h>
#include "PinChangeInterrupt.h"

/*
 * SOFT DIGITALE + IMPULSE TIME COUTER ON LAST INPUT 
 * 
 * FUSES
 * --------
 * EFUSE = 0xff     // = 0x07
 * HFUSE = 0xde
 * LFUSE = 0xe2
 * 
 */
 
#define ID1   "DA9"
#define ID2   "DC9"

#define LEDON    PORTB |= (1<<PB5)
#define LEDOFF   PORTB &= ~(1<<PB5)

#define pinImpulse 12   // PB4 - MOSI
volatile uint16_t timeCounter = 4123; // start from somewere
volatile uint8_t flagCounterStarted = 0;
volatile uint32_t timeStamp = 0;



#define TO_HEX(i)   (i <= 9 ? '0' + i : 'A' - 10 + i)
#define HI(x)       TO_HEX(((x & 0xF0) >> 4))
#define LO(x)       TO_HEX((x & 0x0F))

uint16_t u16CRC;

String str;

/*
CRC Generation from the MODBUS Specification V1.02: 
1. Load a 16bit register with FFFF hex (all 1’s). Call this the CRC register. 
2. Exclusive OR the first 8bit byte of the message with the low order byte of the 16 bit CRC register, putting the result in the 
CRC register. 
3. Shift the CRC register one bit to the right (toward the LSB), zero filling the MSB. 
Extract and examine the LSB. 
4. (If the LSB was 0): Repeat Step 3 (another shift). 
(If the LSB was 1): Exclusive OR the CRC register with the polynomial value 0xA001 (1010 0000 0000 0001). 
5. Repeat Steps 3 and 4 until 8 shifts have been performed. When this is done, a complete 8bit byte will have been processed. 
6. Repeat Steps 2 through 5 for the next 8bit byte of the message. Continue doing this until all bytes have been processed. 
7. The final content of the CRC register is the CRC value. 
8. When the CRC is placed into the message, its upper and lower bytes must be swapped.
*/
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

void setup() {
  // initialize serial port
  Serial.begin(19200);

  // led pin as output
  DDRB |= (1<<PB5);
  LEDOFF;

  // activate pull-ups 
  PORTB |= (1<<PB7)|(1<<PB6)|(1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);  
  PORTC |= (1<<PC5)|(1<<PC4)|(1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
  PORTD |= (1<<PD7)|(1<<PD6)|(1<<PD5);   

  // initialize pins as input
  DDRB &= ~((1<<PB7)&(1<<PB6)&(1<<PB4)&(1<<PB3)&(1<<PB2)&(1<<PB1)&(1<<PB0));  
  DDRC &= ~((1<<PC5)&(1<<PC4)&(1<<PC3)&(1<<PC2)&(1<<PC1)&(1<<PC0));
  DDRD &= ~((1<<PD7)&(1<<PD6)&(1<<PD5));
   
  for (int i=0; i < 10; i++)
  {
    delay(30);    
    LEDON;
    delay(30);
    LEDOFF;
  }            

  // Attach the new PinChangeInterrupt and enable event function below
  //attachPCINT(digitalPinToPCINT(pinImpulse), impulseTimeCounter, CHANGE);
}

/*
void impulseTimeCounter(void)
{
  // If impulse detected
  if (!(PINB & (1<<PB4)))
  {
    // If first time
    if (flagCounterStarted == 0)
    { 
      // Save time stamp
      timeStamp = millis();
      // Disable this block until impulse end
      flagCounterStarted = 1;
    }
    //LEDON; ==========================
  }
  else
  {
    // Calculate impulse duration
    timeCounter = millis() - timeStamp;
    // Reset flag for the next impulse
    flagCounterStarted = 0;
    //LEDOFF; ===================
  }
}
*/

int TRIGGER = 0;
uint32_t millis_counter = millis();
unsigned long now;

void loop() 
{
  now = millis();
  if (now - millis_counter > 30000) // SWITCH DATA TRIGGER AFTER 30 SECONDS
  {                           // TO HAVE TIME WATCH RMMS
    millis_counter = now;
    TRIGGER++;
    if (TRIGGER > 2) TRIGGER = 0;  
    PORTB ^= (1<<PB5);   
  }
  
  // if we receive one char then enter this block
  if (Serial.available())
  {    
    // if we have '/' char then proceed processing more incoming chars
    if (Serial.read() == '/')
    {      
      str = "";
      Serial.setTimeout(100);      
      str = Serial.readStringUntil('\\');
      
      now = millis ();
      while (millis () - now < 50)
        if (Serial.available())
          Serial.read();  // read and discard any input        


      //if (str == "DA0") send_inputsD();
      //if (str == "TA0") send_inputsT();
      //if (str == "IA0") send_inputsI(); 


      if (TRIGGER == 0)
      {  
          if (str == "DA0") { LEDON; Serial.print((char*)";D4688982?"); LEDOFF; }
          if (str == "TA0") { LEDON; Serial.print((char*)";00EB035F0002006559E7?"); LEDOFF; } 
          if (str == "IA0") { LEDON; Serial.print((char*)";0B15FF107DC81628A9EC?"); LEDOFF; } 
      }

      if (TRIGGER == 1)
      {  
          if (str == "DA0") { LEDON; Serial.print((char*)";DFA3A544?"); LEDOFF; }
          if (str == "TA0") { LEDON; Serial.print((char*)";000A0056007D00B6D453?"); LEDOFF; } 
          if (str == "IA0") { LEDON; Serial.print((char*)";18FCFF0B6FDE2A1CC044?"); LEDOFF; } 
      }

      if (TRIGGER == 2)
      {  
          if (str == "DA0") { LEDON; Serial.print((char*)";18E976BC?"); LEDOFF; }
          if (str == "TA0") { LEDON; Serial.print((char*)";006E0001004E000B0274?"); LEDOFF; } 
          if (str == "IA0") { LEDON; Serial.print((char*)";F687DDA1475DEC3A515F?"); LEDOFF; } 
      }

/*    
/DA0\;D4688982?   ===================   1001, 1100, 1110, 1010
/TA0\;00EB035F0002006559E7?   =======   235, 863, 2, 101
/IA0\;0B15FF107DC81628A9EC?   =======   11, 21, 255, 16, 125, 200, 22, 40
**************************************************************************
/DA0\;DFA3A544?   ===================   1101, 1111, 1010, 0011
/TA0\;000A0056007D00B6D453?   =======   10, 86, 125, 182
/IA0\;18FCFF0B6FDE2A1CC044?   =======   24, 252, 255, 11, 111, 222, 42, 28
**************************************************************************
/DA0\;18E976BC?   ===================   0001, 1000, 1110, 1001
/TA0\;006E0001004E000B0274?   =======   110, 1, 78, 11
/IA0\;F687DDA1475DEC3A515F?   =======   246, 135, 221, 161, 71, 93, 236, 58
*/

    }
  }
}

/*
void send_inputsD(void)
{
  char buf[30];  
  
  LEDON;

  unsigned long now = millis();

  uint8_t i = 0;
  buf[i++] = ';';
 
  buf[i++] = TO_HEX(0b00000001);
  buf[i++] = TO_HEX(0b00001000);
  buf[i++] = TO_HEX(0b00001110);
  buf[i++] = TO_HEX(0b00001001);

  u16CRC = 0xFFFF;
  for (uint8_t j=0; j<i; j++)
    u16CRC = crc16_update(u16CRC, buf[j]);

  buf[i++] = TO_HEX(((u16CRC & 0xF000) >> 12));
  buf[i++] = TO_HEX(((u16CRC & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((u16CRC & 0x00F0) >> 4));
  buf[i++] = TO_HEX( (u16CRC & 0x000F));

  buf[i++] = '?';
    
  buf[i++] = 0;   // string ending
  
  Serial.print((char*)buf);

  // wait at least 12 ms
  while (millis() - now < 12);
  
  LEDOFF;   
}

void send_inputsI(void)
{
  uint16_t I0, I1, I2, I3, I4, I5, I6, I7;
  char buf[30];  
  
  LEDON;
  
  I0 = 246;
  I1 = 135;
  I2 = 221;
  I3 = 161;

  I4 = 71;
  I5 = 93;
  I6 = 236;
  I7 = 58;

  // module top side
  uint8_t i = 0;
  buf[i++] = ';';
  buf[i++] = HI(I0);  buf[i++] = LO(I0);
  buf[i++] = HI(I1);  buf[i++] = LO(I1);
  buf[i++] = HI(I2);  buf[i++] = LO(I2);
  buf[i++] = HI(I3);  buf[i++] = LO(I3);

  // module bottom side
  buf[i++] = HI(I4);  buf[i++] = LO(I4);
  buf[i++] = HI(I5);  buf[i++] = LO(I5);
  buf[i++] = HI(I6);  buf[i++] = LO(I6);
  buf[i++] = HI(I7);  buf[i++] = LO(I7);

  u16CRC = 0xFFFF;
  for (uint8_t j=0; j<i; j++)
    u16CRC = crc16_update(u16CRC, buf[j]);

  buf[i++] = TO_HEX(((u16CRC & 0xF000) >> 12));
  buf[i++] = TO_HEX(((u16CRC & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((u16CRC & 0x00F0) >> 4));
  buf[i++] = TO_HEX((u16CRC & 0x000F));;

  buf[i++] = '?';
    
  buf[i++] = 0;   // string ending
  
  Serial.print((char*)buf);
  
  LEDOFF;   
}

void send_inputsT(void)
{
  uint16_t T0, T1, T2, T3;
  uint8_t stat;
  char buf[30];  
  
  LEDON;
         
  T0 = 110;
  T1 = 1;
  T2 = 78;
  T3 = 11;
  
  if (T0 > 1000) T0 = 1000; // 100 grade max
  if (T1 > 1000) T1 = 1000; // 100 grade max
  if (T2 > 1000) T2 = 1000; // 100 grade max
  if (T3 > 1000) T3 = 1000; // 100 grade max
    
    

  // module top side
  uint8_t i = 0;
  buf[i++] = ';';
  
  buf[i++] = TO_HEX(((T0 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T0 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T0 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T0 & 0x000F));
  
  buf[i++] = TO_HEX(((T1 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T1 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T1 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T1 & 0x000F));
  
  buf[i++] = TO_HEX(((T2 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T2 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T2 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T2 & 0x000F));
  
  buf[i++] = TO_HEX(((T3 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T3 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T3 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T3 & 0x000F));
  
  u16CRC = 0xFFFF;
  for (uint8_t j=0; j<i; j++)
    u16CRC = crc16_update(u16CRC, buf[j]);

  buf[i++] = TO_HEX(((u16CRC & 0xF000) >> 12));
  buf[i++] = TO_HEX(((u16CRC & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((u16CRC & 0x00F0) >> 4));
  buf[i++] = TO_HEX((u16CRC & 0x000F));;

  buf[i++] = '?';
    
  buf[i++] = 0;   // string ending
  
  Serial.print((char*)buf);
 
  LEDOFF;
}
*/
