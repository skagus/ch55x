#include <ch554.h>
#include <string.h>
#include <stdio.h>
//#include <debug.h>
#include "usb_io.h"
#include "ws2812_bitbang.h"

#define BUF_SIZE	(128)


void logic_init()
{
//    P1_DIR_PU &= 0x0C;
}

__xdata uint8_t gaLine[BUF_SIZE+1];
__xdata uint8_t gnRcv;

void handleCmd(uint8_t* aLine)
{
	switch(aLine[0])
	{
		case 'r':
		{
			usb_send_cdc0("color R", 7);
			break;
		}
		case 'g':
		{
			usb_send_cdc0("color G", 7);
			break;
		}
		case 'b':
		{
			usb_send_cdc0("color B", 7);
			break;
		}
		default:
		{
			usb_send_cdc0("color X", 7);
			break;
		}
	}
}

void cli_run()
{
	uint8_t cBuf;
	if(usb_get_cdc0(&cBuf, 1) > 0)
	{
		if((cBuf == '\r') || (cBuf == '\n'))
		{
			gaLine[gnRcv] = 0;
			if(gnRcv > 0)
			{
				handleCmd(gaLine);
				gnRcv = 0;
			}
		}
		else
		{
			gaLine[gnRcv] = cBuf;
			gnRcv++;
			if(gnRcv >= BUF_SIZE)
			{
				gaLine[gnRcv] = 0;
				handleCmd(gaLine);
				gnRcv = 0;
			}
		}
	}
}

#if 0
__xdata uint8_t aBuff[2][BUF_SIZE];
__xdata uint8_t anSize[2] = {0,0};

void echo()
{
    if(0 == anSize[0])
    {
        anSize[0] = usb_get_cdc0(aBuff[0], BUF_SIZE);
    }
    if(anSize[0] > 0)
    {
        if(0 != usb_send_cdc0(aBuff[0], anSize[0]))
        {
            anSize[0] = 0;
        }
    }

    if(0 == anSize[1])
    {
        anSize[1] = usb_get_cdc1(aBuff[1], BUF_SIZE);
    }
    if(anSize[1] > 0)
    {
        if(0 != usb_send_cdc1(aBuff[1], anSize[1]))
        {
            anSize[1] = 0;
        }
    }
}
#endif

void logic_run()
{
	cli_run();
}