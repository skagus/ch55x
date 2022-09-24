/**
 * Handle Keyboard event.
 **/

#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>
#include "kbd.h"
#include "hid_key.h"

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
		HIDKey[2] = c - 'a' + KEY_A;
	}
	else if( ('A' <= c) && (c <= 'Z' ))
    {
        HIDKey[0] = KEY_MOD_LSHIFT;
		HIDKey[2] = c - 'A' + KEY_A;
	}
	else if('1' <= c && c <= '9' )
    {
        HIDKey[2] = c - '1' + KEY_1;
    }
    else if(' ' <= c && c <= '0')
    {
		const uint8_t anTable1['0' - ' ' + 1][2] =
		{
			{KEY_NONE, KEY_SPACE}, // space.
			{KEY_MOD_LSHIFT, KEY_1}, // !
			{KEY_MOD_LSHIFT, KEY_APOSTROPHE}, // "
			{KEY_MOD_LSHIFT, KEY_3}, // #
			{KEY_MOD_LSHIFT, KEY_4}, // $
			{KEY_MOD_LSHIFT, KEY_5}, // %
			{KEY_MOD_LSHIFT, KEY_7}, // &
			{KEY_NONE, KEY_APOSTROPHE}, // '
			{KEY_MOD_LSHIFT, KEY_9}, // (
			{KEY_MOD_LSHIFT, KEY_0}, // )
			{KEY_MOD_LSHIFT, KEY_8}, // *
			{KEY_NONE, KEY_KPPLUS}, // +
			{KEY_NONE, KEY_COMMA}, // ,
			{KEY_NONE, KEY_MINUS}, // -
			{KEY_NONE, KEY_DOT}, // .
			{KEY_NONE, KEY_SLASH}, // /
			{KEY_NONE, KEY_0}, // 0
		};
		uint8_t nIdx = c - ' ';
		HIDKey[0] = anTable1[nIdx][0];
		HIDKey[2] = anTable1[nIdx][1];
	}
	else if(':' <= c && c <= '@')
	{
		const uint8_t anTable['@' - ':' + 1][2] = 
		{
			{KEY_MOD_LSHIFT, KEY_SEMICOLON}, // :
			{KEY_NONE, KEY_SEMICOLON}, // ;
			{KEY_MOD_LSHIFT, KEY_COMMA},// <
			{KEY_NONE, KEY_EQUAL},// =
			{KEY_MOD_LSHIFT, KEY_DOT},// >
			{KEY_MOD_LSHIFT, KEY_SLASH},// ?
			{KEY_MOD_LSHIFT, KEY_2},// @
		};
		uint8_t nIdx = c - ':';
		HIDKey[0] = anTable[nIdx][0];
		HIDKey[2] = anTable[nIdx][1];
	}
	else if('[' <= c && c <= '`')
	{
		const uint8_t anTable2['`' - '[' + 1][2] =
		{
			{KEY_NONE, KEY_LEFTBRACE}, // [
			{KEY_NONE, KEY_BACKSLASH}, // '\'
			{KEY_NONE, KEY_RIGHTBRACE}, // ]
			{KEY_MOD_LSHIFT, KEY_6}, // ^
			{KEY_MOD_LSHIFT, KEY_MINUS}, // _
			{KEY_NONE, KEY_GRAVE},// `
		};
		uint8_t nIdx = c - '[';
		HIDKey[0] = anTable2[nIdx][0];
		HIDKey[2] = anTable2[nIdx][1];
	}
	else if('{' <= c && c <= '~')
	{
		const uint8_t anTable3['~' - '{' + 1][2] =
		{
			{KEY_MOD_LSHIFT, KEY_LEFTBRACE}, // {
			{KEY_MOD_LSHIFT, KEY_BACKSLASH}, // |
			{KEY_MOD_LSHIFT, KEY_RIGHTBRACE}, // }
			{KEY_MOD_LSHIFT, KEY_GRAVE}, // ~
		};
		uint8_t nIdx = c - '{';
		HIDKey[0] = anTable3[nIdx][0];
		HIDKey[2] = anTable3[nIdx][1];
	}
	else if(('\r' == c) || ('\n' == c))
	{
		HIDKey[2] = KEY_ENTER;
	}

	usb_SendKbd(HIDKey, sizeof(HIDKey));
	mDelaymS(10);

	HIDKey[0] = 0;
	HIDKey[2] = 0;
	usb_SendKbd(HIDKey, sizeof(HIDKey));
	mDelaymS(10);
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
#if 0
			for(char a = ' '; a < 0x7F; a++)
			{
				SendChar(a);
			}
#else
			memset(aBuff, 0x0, sizeof(aBuff));
			if(KEY != 0) // Released.
			{
//				aBuff[2] = 0;
				usb_SendKbd(aBuff, 8);
			}
			else // Pushed.
			{
				aBuff[2] = KEY_SPACE;
				usb_SendKbd(aBuff, 8);
			}
#endif
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
