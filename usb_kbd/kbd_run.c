/**
 * Handle Keyboard event.
 **/

#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>
#include "kbd.h"


#define		L_WIN 					0x08
#define 	L_ALT 					0x04
#define		L_SHIFT					0x02
#define 	L_CTL					0x01
#define 	R_WIN 					0x80
#define 	R_ALT 					0x40
#define 	R_SHIFT					0x20
#define 	R_CTL					0x10
#define 	SPACE					0x2C
#define		ENTER					0x28


#define LED_PIN1	(0)
#define LED_PIN2	(1)
#define KEY_PIN		(5)

SBIT(LED1, 0xB0, LED_PIN1);
SBIT(LED2, 0xB0, LED_PIN2);
SBIT(KEY, 0xB0, KEY_PIN);

extern uint32_t gnMillis;
extern __xdata uint8_t			Touch_IN;

static void SendChar (char c)
{
    static uint8_t HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
	if( ('a' <= c) && (c <= 'z' ))
    {
		c = c - 'a' + 'A';  // Upper case.
	}
    else
    {
        HIDKey[0] = L_SHIFT;
    }

	if( ('A' <= c) && (c <= 'Z' ))
    {
		HIDKey[2] = c - 'A' + 4;
	}
	else if('1' <= c && c <= '9' )
    {
        HIDKey[0] = 0x00;
        HIDKey[2] = c - '1' + 0x1E;
    }
    else
    {
		switch ( c )
        {
			case '`' :
				HIDKey[0] = 0x08;
				HIDKey[2] = 0x15;
				break;
			case '\\':
				HIDKey[2] = 0x31;
				break;
			case ' ':
				HIDKey[2] = SPACE;
				break;
			case '\n':
			case '\r':
				HIDKey[2] = ENTER;
				break;
			case ':':
				HIDKey[0] = 0x02;
				HIDKey[2] = 0x33;
				break;
			case '+':
				HIDKey[0] = 0x000;
				HIDKey[2] = 0x57;
				break;
			case '?':
				HIDKey[0] = L_SHIFT;
				HIDKey[2] = 0x38;
				break;
			case '_':
				HIDKey[0] = 0x02;
				HIDKey[2] = 0x2D;
				break;
			case '/':
				HIDKey[0] = L_CTL + L_ALT;
				HIDKey[2] = 0x16;
				break;
			case '0':
				HIDKey[2] = 0x27;
				break;
			case '.':
				HIDKey[2] = 0x37;
				break;
			case '~':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0x05;
				break;
			case '!':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0x08;
				break;
			default:
				break;
		}
	}
	mDelaymS( 10 );
	usb_SendKbd(HIDKey, sizeof(HIDKey));

	mDelaymS( 10 );
	HIDKey[0] = 0x00;
	HIDKey[2] = 0x00;
	usb_SendKbd(HIDKey, sizeof(HIDKey));
}

void SendString(char* pStr)
{
    while(*pStr)
    {
        SendChar(*pStr);
        pStr++;
    }
}

extern uint8_t gnReport;	// Key LED signal.

uint32_t nLastMillis = 0;
uint8_t nReportBak;
uint8_t nKeyIn = 0;
void kbd_run()
{
    __xdata char aBuff[32];
    if (gnMillis - nLastMillis > 40)
    {
        nLastMillis = gnMillis;

        if(nReportBak != gnReport)
        {
            LED1 = !LED1;
            nReportBak = gnReport;
//            sprintf(aBuff, "RE:%X\n", gnReport);
//            SendString(aBuff);
        }
		if(nKeyIn != KEY)
		{
			memset(aBuff, 0x0, sizeof(aBuff));
			if(KEY != 0) // Released.
			{
				aBuff[2] = 0;
				usb_SendKbd(aBuff, 8);
			}
			else // Pushed.
			{
				aBuff[2] = ENTER;
				usb_SendKbd(aBuff, 8);
			}
			nKeyIn = KEY;
		}
#if EN_TOUCH
        if (Touch_IN != 0)
        {
            LED1 = !LED1;
            sprintf(aBuff, "IN:%X\n", Touch_IN);
            SendString(aBuff);
            Touch_IN = 0;
        }
#endif
    }
}



void kbd_init()
{
    P3_MOD_OC = P3_MOD_OC | (1<<LED_PIN1);
    P3_DIR_PU = P3_DIR_PU |	(1<<LED_PIN1);
    P3_MOD_OC = P3_MOD_OC | (1<<LED_PIN2);
    P3_DIR_PU = P3_DIR_PU |	(1<<LED_PIN2);
}
