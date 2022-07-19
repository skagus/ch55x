#include <ch554.h>
#include <string.h>
#include <stdio.h>
#include <debug.h>
#include <touchkey.h>
#include "usb_io.h"

#define BUF_SIZE	(128)

__xdata uint8_t aBuff[2][BUF_SIZE];
__xdata uint8_t anSize[2] = {0,0};
__xdata uint8_t gbmBut = 0;
#define BUT_MASK        (1 << 6)

void logic_init()
{
//    P1_DIR_PU &= 0x0C;
    TouchKeyQueryCyl2ms();
    GetTouchKeyFree();
}

void _cross_resp()
{
    if(0 == anSize[0])
    {
        anSize[0] = usb_get_cdc0(aBuff[0], BUF_SIZE);
    }
    if(0 == anSize[1])
    {
        anSize[1] = usb_get_cdc1(aBuff[1], BUF_SIZE);
    }


    if(anSize[0] > 0)
    {
        if(0 != usb_send_cdc1(aBuff[0], anSize[0]))
        {
            anSize[0] = 0;
        }
    }
    if(anSize[1] > 0)
    {
        if(0 != usb_send_cdc0(aBuff[1], anSize[1]))
        {
            anSize[1] = 0;
        }
    }
}

void _touch_test()
{
    TouchKeyChannelQuery();                                                  //Query the status of touch keys
    if(KeyBuf != gbmBut)
    {
        anSize[0] = sprintf(aBuff[0], " Pushed %X \r\n", KeyBuf);
        gbmBut = KeyBuf;
    }
    else
    {
        anSize[0] = sprintf(aBuff[0], "X");
    }
    KeyBuf = 0;

    if(anSize[0] > 0) 
    {
        if(0 != usb_send_cdc0(aBuff[0], anSize[0]))
        {
            anSize[0] = 0;
        }
    }    
}


__xdata uint8_t nSeq = 0;

void _dump0()
{
    if(0 == anSize[0])
    {
        anSize[0] = sprintf(aBuff[0], " Pushed %X \r\n", nSeq);
        nSeq++;
    }

    if(anSize[0] > 0) 
    {
        if(0 != usb_send_cdc0(aBuff[0], anSize[0]))
        {
            anSize[0] = 0;
        }
    }
}

void logic_run()
{
    _touch_test();
//    _dump0();
}