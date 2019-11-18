#include <avr/io.h>

#ifndef F_CPU
//	#define F_CPU 12000000UL
   #define F_CPU 1000000UL
#endif

#include <util/delay.h>

#include "myutils.h"

#ifndef _LCD_H
#define _LCD_H
/*_________________________________________________________________________________________*/

/************************************************
	LCD CONNECTIONS
   Uses PORTD.7 - Strobe, PORT.6 - RS, PORT.5 - RW, DataNibble PORTC.4 - PORTC.7
   Code Modified to use PORTC Only
*************************************************/

#define LCD_DATA C	//Port PC0-PC3 are connected to D4-D7

#define LCD_E C 		//Enable OR strobe signal
#define LCD_E_POS	PC7	//Position of enable in above port

#define LCD_RS C	
#define LCD_RS_POS 	PC6

#define LCD_RW C
#define LCD_RW_POS 	PC5


//************************************************

#define LS_BLINK 0B00000001
#define LS_ULINE 0B00000010



/***************************************************
			F U N C T I O N S
****************************************************/



void InitLCD(uint8_t style);
void LCDWriteString(const char *msg);
void LCDWriteInt(int val,unsigned int field_length);
void LCDGotoXY(uint8_t x,uint8_t y);

//Low level
void LCDByte(uint8_t,uint8_t);
#define LCDCmd(c) (LCDByte(c,0))
#define LCDData(d) (LCDByte(d,1))

void LCDBusyLoop();





/***************************************************
			F U N C T I O N S     E N D
****************************************************/


/***************************************************
	M A C R O S
***************************************************/
#define LCDClear() LCDCmd(0b00000001)
#define LCDHome() LCDCmd(0b00000010);

#define LCDWriteStringXY(x,y,msg) {\
 LCDGotoXY(x,y);\
 LCDWriteString(msg);\
}

#define LCDWriteIntXY(x,y,val,fl) {\
 LCDGotoXY(x,y);\
 LCDWriteInt(val,fl);\
}
/***************************************************/




/*_________________________________________________________________________________________*/
#endif






