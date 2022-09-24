/**
 * Handle Keyboard event.
 **/

#include <stdio.h>
#include <string.h>
#include <ch554.h>
#include <debug.h>
#include <ch554_usb.h>
#include "kbd.h"
#include "hid_key.h"

/**
 * 3.6, 3.7은 USB +/-
 * 3.0, 3.1은 UART0 <-- Debug.

 * 1.2, 1.3은 UART_
 * 1.6, 1.7은 UART1
 * Drive PIN: 4EA.
 * 3.2, 3.3, 3.4, 3.5
 * 
 * Sense PIN: 5EA.
 * 1.0, 1.1, 1.2(x), 1.3(x), 1.4, 1.5, 1.6
 *
 **/
#define NUM_DRV_BIT		(4)
#define DRV_OFFSET		(2)
#define MASK_DRIVE		(0x0F << DRV_OFFSET)
#define MASK_INPUT		(0x73)

#define NUM_INPUT_BIT	(7)
#define PORT_DRIVE		(P3)
#define PORT_INPUT		(P1)

#define BIT(x)			(1 << (x))
extern uint32_t gnMillis;

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
		__code const uint8_t anTable1['0' - ' ' + 1][2] =
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
		__code const uint8_t anTable['@' - ':' + 1][2] = 
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
		__code const uint8_t anTable2['`' - '[' + 1][2] =
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
		__code const uint8_t anTable3['~' - '{' + 1][2] =
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

uint8_t _ScanKey(uint8_t nDrvIdx)
{
	// Drive.
	uint8_t bmDrv = (1 << nDrvIdx + DRV_OFFSET);
	PORT_DRIVE &= ~bmDrv;	// Pull down Drive.
	// Sense
	uint8_t bmIn = PORT_INPUT & MASK_INPUT;
	PORT_DRIVE |= MASK_DRIVE;
	return bmIn;
}

/**
 * Key 한개만 대응한다.
 **/
void _SendKeyScan(uint8_t nDrvIdx, uint8_t nPrv, uint8_t nCur)
{
	__code const uint8_t aKeyCode[NUM_DRV_BIT * NUM_INPUT_BIT] =
	{
		KEY_NUMLOCK,	KEY_KP7,	KEY_NONE,	KEY_NONE,	KEY_KP4,	KEY_KP1,	KEY_KP0,
		KEY_KPSLASH,	KEY_KP8,	KEY_NONE,	KEY_NONE,	KEY_KP5,	KEY_KP2,	KEY_NONE,
		KEY_KPASTERISK,	KEY_KP9,	KEY_NONE,	KEY_NONE,	KEY_KP6,	KEY_KP3,	KEY_KPDOT,
		KEY_KPMINUS,	KEY_KPPLUS,	KEY_NONE,	KEY_NONE,	KEY_NONE,	KEY_KPENTER,KEY_NONE,
	};
    static uint8_t aBuf[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
	uint8_t nXor = nPrv ^ nCur;

	for(uint8_t nBit = 0; nBit < NUM_INPUT_BIT; nBit++)
	{
		uint8_t nIdx = nDrvIdx * NUM_INPUT_BIT + nBit;
		if(nXor & BIT(nBit))
		{
			if(nCur & BIT(nBit)) // Released.
			{
				aBuf[2] = 0;
				usb_SendKbd(aBuf, sizeof(aBuf));
			}
			else	// Pressed.
			{
				aBuf[2] = aKeyCode[nIdx];
				usb_SendKbd(aBuf, sizeof(aBuf));
			}
			mDelaymS(10);
		}
	}
}

__xdata uint32_t gnLastMillis = 0;
uint32_t gnLastPrintMillis = 0;
__xdata uint8_t gaPrvKey[NUM_DRV_BIT] = {MASK_INPUT, MASK_INPUT, MASK_INPUT, MASK_INPUT};
//__xdata uint8_t gaPrvKey[NUM_DRV_BIT] = {0, 0, 0, 0};
uint8_t gnDrvIdx = 0;

void kbd_run()
{
    if (gnMillis - gnLastMillis > 10)
    {
        gnLastMillis = gnMillis;
		uint8_t nInput = _ScanKey(gnDrvIdx);
		if(nInput != gaPrvKey[gnDrvIdx])
		{
			_SendKeyScan(gnDrvIdx, gaPrvKey[gnDrvIdx], nInput);
			gaPrvKey[gnDrvIdx] = nInput;
		}
		gnDrvIdx = (gnDrvIdx + 1) % NUM_DRV_BIT;
    }
}

void kbd_init()
{
	// Output Control, Pull up control.
	PORT_DRIVE |= MASK_DRIVE;
    P3_MOD_OC = P3_MOD_OC | MASK_DRIVE;
    P3_DIR_PU = P3_DIR_PU |	MASK_DRIVE;

	// input control.
    P1_MOD_OC = P1_MOD_OC & ~(MASK_INPUT);
    P1_DIR_PU = P1_DIR_PU |	MASK_INPUT;
}
