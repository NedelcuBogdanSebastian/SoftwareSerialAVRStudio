/*
 * SoftwareSerialAVRStudio.c
 *
 * Created: 2/2/2022 1:14:51 AM
 *  Author: Bogdan
 *
 * Pentru portare au fost modificate:
 * portserial.c
 * porttimer.c
 * mbport.h
 * mbcrc.c
 * port.h
 *
 */ 


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "port.h"
#include "main.h"

/************************************************
*   HERE ARE MODBUS DEFINES AND VARIABLES   *
*************************************************/

eMBErrorCode    eStatus;

/*****************************************************
*   HERE ARE ALL SOFT SERIAL DEFINES AND VARIABLES   *
******************************************************/
#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

uint16_t _rx_delay_centering;
uint16_t _rx_delay_intrabit;
uint16_t _rx_delay_stopbit;
uint16_t _tx_delay;

uint16_t _buffer_overflow = 0;

// static data
static char _receive_buffer[_SS_MAX_RX_BUFF];
static volatile uint8_t _receive_buffer_tail = 0;
static volatile uint8_t _receive_buffer_head = 0;

const int XMIT_START_ADJUSTMENT = 5;

/***********************^*************************
*   HERE ARE ALL MODULES DEFINES AND VARIABLES   *
*************************^************************/
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
#define TO_DEC(i) (i <= '9'? i - '0': i - 'A' + 10)
#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))

uint8_t currentModuleType = 0;        // here we store to what type of module we've sent the ID
uint8_t currentModuleDataLength = 0;
uint8_t holdingRegistersIndex = 3;    // in the holding register number 2 we store ambiant temperature (1 in the RMMS)
uint8_t coilsIndex = 2;               // coils start from position 2 (1 in RMMS)

uint8_t errBuffer[10] = {0};          // buffer where we keep error counter for each of the modules
	
unsigned long volatile millis_count = 0;
unsigned long volatile last_count = 0;
uint8_t volatile module_counter=0;
uint8_t volatile triggerSendID=0;
uint8_t volatile triggerReceiveData=0;
uint8_t volatile stateToggler=0;

#define NUMBER_OF_MODULES 6           // HOW MANY MODULES DO WE HAVE

/***********************^********************
*   TIMER 0 INTERRUPT HAS 2 FUNCIONS:       *
*   1.TO TRIGGER: - MODULE ID SENDING       *
*                 - PROCESS REVEIVED DATA   * 
*   2.TO INCREMENT MODULE COUNTER           *     
*************************^*******************/
ISR(TIMER0_COMPA_vect)
{
	// increment milliseconds counter
	millis_count++;
	
	// test if 100 milliseconds have passed since last time
	if (millis_count - last_count >= 100)
	{
		// save current milliseconds counter value
		last_count = millis_count;
		
		// toggle state (send id or process received data)
		stateToggler ^= 1;
		
		if (stateToggler == 1)
		{
			// increment module counter
			module_counter++;
			// activate trigger for module ID sending
			triggerSendID = 1;
			// module counter start over
			if (module_counter == NUMBER_OF_MODULES) module_counter = 0;
		}
		else
		{
			// activate trigger for received data processing
			triggerReceiveData = 1;
		}
		//PORTB ^= (1<<PB5);   // toggle SCK led
	}	
}

/* static */
inline void ssDelay(uint16_t delay)
{
	uint8_t tmp=0;

	asm volatile("sbiw    %0, 0x01 \n\t"
	"ldi %1, 0xFF \n\t"
	"cpi %A0, 0xFF \n\t"
	"cpc %B0, %1 \n\t"
	"brne .-10 \n\t"
	: "+w" (delay), "+a" (tmp)    
	: "0" (delay)
	);
	// ATENTIE AM SCHIMBAT +r IN +w
}

void ss_flush()
{
	uint8_t oldSREG = SREG;
	cli();
	_receive_buffer_head = _receive_buffer_tail = 0;
	SREG = oldSREG;
}

void ss_write(uint8_t b)
{
	uint8_t oldSREG = SREG;
	cli();  // turn off interrupts for a clean txmit

	// Write the start bit
	PORTD &= ~(1 << PD4);
	
	ssDelay(_tx_delay + XMIT_START_ADJUSTMENT);

	for (uint8_t mask = 0x01; mask; mask <<= 1)
	{
		if (b & mask) PORTD |= (1 << PD4); // send 1
		else PORTD &= ~(1 << PD4); // send 0
		
		ssDelay(_tx_delay);
	}
	PORTD |= (1 << PD4); // restore pin to natural state

	SREG = oldSREG; // turn interrupts back on
	ssDelay(_tx_delay);
}

// Read data from buffer
int ss_read()
{
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail) return -1;

	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
	_receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
	return d;
}

int ss_available()
{
	return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

void ss_begin(void)
{
	//  baud    rxcenter   rxintra    rxstop      tx
	//  19200,    54,        117,       117,      114
	_rx_delay_centering = 50;
	_rx_delay_intrabit = 110;
	_rx_delay_stopbit = 117;
	_tx_delay = 114;

	// init RX TX pins and RX pinchange interrupt
	DDRD |= (1<<PD4); // PD4 as TX
	PORTD |= (1<<PD4); // set as high

	DDRD &= ~(1<<PD3); // PD3 as RX
	PORTD |= (1<<PD3); // activate pullup for normal logic!

	// RX on pin PD3, PCINT19, PCMSK2 bit 3, PCICR bit 2 (PCIE2)
	// Set up RX interrupt
	PCICR |= (1<<2);
	PCMSK2 |= (1<<3);

	ssDelay(_tx_delay); // if we were low this establishes the end

	// init TIMER0
	TCCR0A = 0b10;
	TCCR0B = 0b11;  //Timer settings for interrupt at every millisecond
	OCR0A = 249;
	TIMSK0 |= 0b10;
	TIMSK0 |= (1<<TOIE0);
	
	sei();
}

void ss_recv()
{
	#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Preserve the registers that the compiler misses
	asm volatile(
	"push r18 \n\t"
	"push r19 \n\t"
	"push r20 \n\t"
	"push r21 \n\t"
	"push r22 \n\t"
	"push r23 \n\t"
	"push r26 \n\t"
	"push r27 \n\t"
	::);
	#endif
    
	uint8_t d = 0;
	
	// If RX line is high, then we don't see any start bit
	// so interrupt is probably not for us
	if (!(PIND & (1<<PD3)))
	{
		// Wait approximately 1/2 of a bit width to "center" the sample
		ssDelay(_rx_delay_centering);
		
		// Read each of the 8 bits
		for (uint8_t i=0x1; i; i <<= 1)
		{
			ssDelay(_rx_delay_intrabit);
			uint8_t noti = ~i;
			if (PIND & (1<<PD3)) d |= i;
			else d &= noti;   // else clause added to ensure function timing is ~balanced
		}
		
		// skip the stop bit
		ssDelay(_rx_delay_stopbit);
		
		// if buffer full, set the overflow flag and return
		if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head)
		{
			// save new data in buffer: tail points to where byte goes
			_receive_buffer[_receive_buffer_tail] = d; // save new byte
			_receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
		}
		else _buffer_overflow = 1;
	}

	#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Restore the registers that the compiler misses
	asm volatile(
	"pop r27 \n\t"
	"pop r26 \n\t"
	"pop r23 \n\t"
	"pop r22 \n\t"
	"pop r21 \n\t"
	"pop r20 \n\t"
	"pop r19 \n\t"
	"pop r18 \n\t"
	::);
	#endif
}

inline void cacat()
{
	ss_recv();
}

ISR(PCINT2_vect)
{
	cacat();
}


#define UART_BAUD 19200
void init_uart(void)
{
	UBRR0H = 0;
	UBRR0L = (F_CPU / (16UL * UART_BAUD)) - 1;
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);  //enable transmitter and receiver
	UCSR0C = (3<<UCSZ00); // 8 bits, 1 stop bit, no parity
}
void uart_putc(char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}
void uart_puts( const unsigned char *string )
{
	while (*string != 0)  uart_putc( *string++ );
}

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
uint16_t crcErrorFlag;
uint16_t u16CRC_computed;

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	uint8_t i;

	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
		if (crc & 1) crc = (crc >> 1) ^ 0xA001;
		else crc = (crc >> 1);
	}

	return crc;
}

void writeCoil(uint8_t coil_index, uint8_t state)
{
	uint8_t coil_offset=coil_index/8;
	if (state == 1)
	usRegCoilBuf[coil_offset] |= (1<<(coil_index%8));
	else usRegCoilBuf[coil_offset] &= ~(1<<(coil_index%8));
}

uint8_t getCoil(uint8_t coil_index)
{
	uint8_t coil_byte=usRegCoilBuf[coil_index/8];
	if (coil_byte && (1<<(coil_index%8))) return 1;
	else return 0;
}

void writeHoldingRegister(uint8_t reg_index, uint16_t reg_val)
{
	usRegHoldingBuf[reg_index] = reg_val;
}

uint16_t readHoldingRegister(uint8_t reg_index)
{
	return usRegHoldingBuf[reg_index];
}

// https://www.h-schmidt.net/FloatConverter/IEEE754.html
//
// from : http://www.chipkin.com/how-real-floating-point-and-32-bit-data-is-encoded-in-modbus-rtu-messages/
// result that the encoding scheme is:
// 2.i16-1.ifloat-sb	byte swap	[ a b ] [ c d ]	=> [ b a d c ]
// [ d c ] - first modbus register
// [ b a ] - second modbus register
void writeFloat2HoldingRegisters(uint8_t reg_index, float f)
{
	uint8_t * v = (uint8_t *) &f;

	uint8_t a, b, c, d;
	uint16_t val16;

	a = v[0];
	b = v[1];
	c = v[2];
	d = v[3];

	// to change encoding scheme to
	// 2.i16-1.ifloat-sw	byte and word swap	[ a b ] [ c d ]	=> [ d c b a ]
	// just change as commented below

	val16 = (b & 0xFF);
	val16 = (val16 << 8) | (a & 0xFF);
	writeHoldingRegister(reg_index, val16);   // reg_index + 1

	val16 = (d & 0xFF);
	val16 = (val16 << 8) | (c & 0xFF);
	writeHoldingRegister(reg_index+1, val16);   // reg_index
}

float get_float(uint8_t reg_index)
{
	float f = 0.0;
	uint8_t * v = (uint8_t *) &f;
	uint8_t a, b, c, d;
	uint16_t val16;

	val16 = usRegHoldingBuf[reg_index];
	b = (val16 >> 8) & 0xFF;
	a = val16 & 0xFF;

	val16 = usRegHoldingBuf[reg_index+1];
	d = (val16 >> 8) & 0xFF;
	c = val16 & 0xFF;

	v[0] = a;
	v[1] = b;
	v[2] = c;
	v[3] = d;

	writeFloat2HoldingRegisters(reg_index+4, f);   // to test
	return f;
}

uint8_t hex2dec(char c)
{
	uint8_t data = 0;
	
    // find the decimal representation of hex
    if(c >= '0' && c <= '9')
    {
	    data = c- 48;
    }
    else if(c >= 'A' && c <= 'F')
    {
	    data = c - 65 + 10;
    }	
	
	return data;
}

int main(void)
{	
	uint8_t k;
	uint16_t tmp;
	char buf[10]; 
	
	
	
	// RX - 3 - PD3
	// TX - 4 - PD4
	// Baud 19200
    // software serial
	ss_begin();
	
	// used to debug module reading and data integrity testing
	// the serial port is the same used by modbus
	// use this only when modbus if not started	
	//init_uart();

	// init SCK led pin
	DDRB |= (1<<PB5);

    eStatus = eMBInit(MB_RTU, 2, 1, 19200, MB_PAR_NONE);
    sei(  );
    // Enable the Modbus Protocol Stack
    eStatus = eMBEnable(  );

    // enable watchdog at 250 milliseconds
    wdt_enable(WDTO_250MS);
		
    for (;;)
	{
        ( void )eMBPoll(  );
        // Here we simply count the number of poll cycles.
        usRegHoldingBuf[2]++;
			
		// feed the watchdog so we don't get bitten	
		wdt_reset();
		
		if (triggerSendID == 1)
		{
			// reset send id trigger 
			triggerSendID = 0;
			
			/*****************************
			 *        BELOW IS WERE      *
			 *   WE DEFINE NEW MODULES   *
			 *         NEW MODULES       * 
			 *****************************/
			
			if (module_counter == 0)
			{
				currentModuleType = 2;   // PT100RTD MODULE
				ss_write('/');
				ss_write('T');
				ss_write('A');
				ss_write('0');
				ss_write('\\');
			}
			if (module_counter == 1)
			{
				currentModuleType = 1;   // 4-20mA MODULE
				ss_write('/');
				ss_write('I');
				ss_write('A');
				ss_write('0');
				ss_write('\\');
			}
			if (module_counter == 2)
			{
				currentModuleType = 0;   // DIGITAL MODULE
				ss_write('/');
				ss_write('D');
				ss_write('A');
				ss_write('0');
				ss_write('\\');
			}
			if (module_counter == 3)
			{
				currentModuleType = 0;   // DIGITAL MODULE
				ss_write('/');
				ss_write('D');
				ss_write('A');
				ss_write('1');
				ss_write('\\');
			}
			if (module_counter == 4)
			{
				currentModuleType = 0;   // DIGITAL MODULE
				ss_write('/');
				ss_write('D');
				ss_write('A');
				ss_write('2');
				ss_write('\\');
			}
			if (module_counter == 5)
			{
				currentModuleType = 0;   // DIGITAL MODULE
				ss_write('/');
				ss_write('D');
				ss_write('A');
				ss_write('5');
				ss_write('\\');
			}




		}

		if (triggerReceiveData == 1)
		{
			// reset received data trigger
			triggerReceiveData = 0;
					
			// all analog values are represented by 2 octets, except PT100 on 4 octets
			// for 1 = digital, length of data is 10,
			//     '?' + 4 octets DATA + 4 octets CRC + ';',
			//     16 values as 1 bit each
			// for 2 = 4-20mA, length of data is 22,
			//     '?' + 16 octets DATA + 4 octets CRC + ';',
			//     8 values as 2 octets each
			// for 3 = 0-10V, length of data is 22,
			//     '?' + 16 octets DATA + 4 octets CRC + ';',
			//     8 values as 2 octets each
			// for 4 = PT100RTD cu MAX31865, length of data is 22,
			//     '?' + 16 octets DATA + 4 octets CRC + ';',
			//     4 values as 4 octets each
			if (currentModuleType > 0) currentModuleDataLength = 22;
			else currentModuleDataLength = 10;				
							
			// reset CRC
			u16CRC_computed = 0xFFFF;
			// compute CRC of data, set top at the beginning of stored CRC
			// starts from 1 because at position 0 we have '/'   "  /;0000DEDA?  "
			for (int i = 0; i < currentModuleDataLength-5; i++) 
			{
				u16CRC_computed = crc16_update(u16CRC_computed, _receive_buffer[i]);
			}

            // reset CRC error flag
            crcErrorFlag = 0;
			// test if computed CRC = received CRC (byte by byte)
			// if we have an error set error flag
			if (TO_HEX(((u16CRC_computed & 0xF000) >> 12)) != _receive_buffer[currentModuleDataLength-5]) crcErrorFlag = 1;
			if (TO_HEX(((u16CRC_computed & 0x0F00) >> 8))  != _receive_buffer[currentModuleDataLength-4]) crcErrorFlag = 1;
			if (TO_HEX(((u16CRC_computed & 0x00F0) >> 4))  != _receive_buffer[currentModuleDataLength-3]) crcErrorFlag = 1;
			if (TO_HEX(((u16CRC_computed & 0x000F)))       != _receive_buffer[currentModuleDataLength-2]) crcErrorFlag = 1;
									
			 
			// used to debug module reading and data integrity testing
			// the serial port is the same used by modbus 
			// use this only when modbus if not started 	 
		    /*if (crcErrorFlag == 0)
            {
				// if the CRC is OK for current module, then print received module data
				// simple method to test CRC :)  
    		    for (int i = 0; i < currentModuleDataLength; i++)
	        	{
		        	uart_putc((char)_receive_buffer[i]);
		        }		
		        uart_putc((char)'\n');
			}
            */

            // if module data integrity check was successful
		    if (crcErrorFlag == 0)
		    {		
				// Toggle led to signal received data integrity OK
			    PORTB ^= (1<<5);
				
				// module data is OK, so we reset this module error counter
				errBuffer[module_counter] = 0;												
										
				// DIGITAL MODULE
				if (currentModuleType == 0)
				{									
					for (int i = 0; i < 4; i++) 
					{
						// convert hex value to uint8_t
						tmp = TO_DEC(_receive_buffer[i + 1]);   // we have data at index 1, 2, 3, 4
						
						// we check each bit from "tmp" data at positions 3,2,1,0 (one hex character)
						for (int j = 0; j < 4; j++) 
						{
							if (tmp & (1<<(3-j))) writeCoil(coilsIndex + (i * 4) + j, 1);
							else writeCoil(coilsIndex + (i * 4) + j, 0);
						}
					}				
				}
                							
                // 4-20mA MODULE
                if (currentModuleType == 1)
                {
				    k = 1;
					// convert and store all 8 values from 4-20mA module
					for (int i = 0; i < 8; i++) 
					{
						// convert hex value 16 bit unsigned int
						tmp = (TO_DEC(_receive_buffer[k]) << 4) | TO_DEC(_receive_buffer[k+1]);
						// follows next 2 chars
						k += 2;
						// store data to modbus stack
						writeHoldingRegister(holdingRegistersIndex + i, tmp);						
					}
				}		
					
				// PT100RTD MAX31865 MODULE
				if (currentModuleType == 2) 
				{
					//tmp = TO_DEC(_receive_buffer[2]);   // first hex octet is always zero, temperature goes till 1000 maximum 
					tmp = (TO_DEC(_receive_buffer[2]) << 4) | TO_DEC(_receive_buffer[3]);
					tmp = (tmp << 4) | TO_DEC(_receive_buffer[4]);
					writeHoldingRegister(holdingRegistersIndex, tmp);

					//tmp = TO_DEC(_receive_buffer[6]);   // first hex octet is always zero, temperature goes till 1000 maximum
					tmp = (TO_DEC(_receive_buffer[6]) << 4) | TO_DEC(_receive_buffer[7]);
					tmp = (tmp << 4) | TO_DEC(_receive_buffer[8]);
					writeHoldingRegister(holdingRegistersIndex + 1, tmp);

					//tmp = TO_DEC(_receive_buffer[10]);   // first hex octet is always zero, temperature goes till 1000 maximum
					tmp = (TO_DEC(_receive_buffer[10]) << 4) | TO_DEC(_receive_buffer[11]);
					tmp = (tmp << 4) | TO_DEC(_receive_buffer[12]);
					writeHoldingRegister(holdingRegistersIndex + 2, tmp);

					//tmp = TO_DEC(_receive_buffer[14]);   // first hex octet is always zero, temperature goes till 1000 maximum
					tmp = (TO_DEC(_receive_buffer[14]) << 4) | TO_DEC(_receive_buffer[15]);
					tmp = (tmp << 4) | TO_DEC(_receive_buffer[16]);
					writeHoldingRegister(holdingRegistersIndex + 3, tmp);
				}				
			}
			else  // if we have data integrity error for current
			{
				// increment error counter for current module, five times
				// after five consecutive errors we write zero to modbus 
				// position where this module data set place is
				if (errBuffer[module_counter] < 5) 
				{
					errBuffer[module_counter]++;
				}
				else
				{
					// reset error counter for current module
					// we end up here again only after 5 more consecutive errors
					errBuffer[module_counter] = 0;
					
					// digital module
				    if (currentModuleType == 0) 
					{
						writeCoil(coilsIndex, 0);
						writeCoil(coilsIndex+1, 0);
						writeCoil(coilsIndex+2, 0);
						writeCoil(coilsIndex+3, 0);
						writeCoil(coilsIndex+4, 0);
						writeCoil(coilsIndex+5, 0);
						writeCoil(coilsIndex+6, 0);
						writeCoil(coilsIndex+7, 0);
						writeCoil(coilsIndex+8, 0);
						writeCoil(coilsIndex+9, 0);
						writeCoil(coilsIndex+10, 0);
						writeCoil(coilsIndex+11, 0);
						writeCoil(coilsIndex+12, 0);
						writeCoil(coilsIndex+13, 0);
						writeCoil(coilsIndex+14, 0);
						writeCoil(coilsIndex+15, 0);
					}

					// 4-20mA module
				    if (currentModuleType == 1)
				    {					
					    writeHoldingRegister(holdingRegistersIndex, 0);
					    writeHoldingRegister(holdingRegistersIndex+1, 0);
					    writeHoldingRegister(holdingRegistersIndex+2, 0);
					    writeHoldingRegister(holdingRegistersIndex+3, 0);
					    writeHoldingRegister(holdingRegistersIndex+4, 0);
					    writeHoldingRegister(holdingRegistersIndex+5, 0);
					    writeHoldingRegister(holdingRegistersIndex+6, 0);
					    writeHoldingRegister(holdingRegistersIndex+7, 0);
					}

					// PT100RTD module
					if (currentModuleType == 2)
					{
						writeHoldingRegister(holdingRegistersIndex, 0);
						writeHoldingRegister(holdingRegistersIndex+1, 0);
						writeHoldingRegister(holdingRegistersIndex+2, 0);
						writeHoldingRegister(holdingRegistersIndex+3, 0);
					}							
				}				
			}
				
			// if this is the last module then we reset holding registers and coils indexes
			// else we increment indexes depending of the curerent module type
			if (module_counter == NUMBER_OF_MODULES-1)
			{
				// in the holding register number 2 we store ambiant temperature (1 in the RMMS)
				// so data from modules start from position 2 in RMMS
				holdingRegistersIndex = 3;
				// coils start from position 2 (1 in RMMS)
				coilsIndex = 2;
			}
			else
			{
			    // digital module
				if (currentModuleType == 0) coilsIndex += 16;
				
				// 4-20mA module
				if (currentModuleType == 1) holdingRegistersIndex += 8;
					
				// PT100RTD module
				if (currentModuleType == 2) holdingRegistersIndex += 4;								
			}			

            // clear soft serial RX buffer			
			for (int i = 0; i < _SS_MAX_RX_BUFF; i++)
			{
				_receive_buffer[i] = '0';
			}
			
			// reset serial RX buffer even if we have an error	
			_receive_buffer_head = _receive_buffer_tail = 0;	
		}
	}
	
	return 0;
}



/*
VALORI PE CARE LE OFERA MODULUL CU ATMEGA88 PENTRU TESTARE
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