#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>
//#include <bootloader.h>
#include "kbd.h"



//__xdata char sPath[] = "This is the CH552 sample text. Press Num to toggle the LED.";
//__xdata char *pStr = sPath;


__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];
__xdata __at (0x0050) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];
__xdata __at (0x000a) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];

uint8_t gbReady;  // USB setup done! work device!.
uint8_t SetupReq;
uint8_t SetupLen;   // 두가지 의미로 사용되는 듯???
uint8_t gbKbdTxDone;
uint8_t* gpNextTX;    // Pointer for next transfer.
uint8_t gnReport;

__code uint8_t DevDesc[0x12] = 
{
    0x12,0x01,0x10,0x01,0x00,0x00,0x00,0x08,
    0x3d,0x41,0x07,0x21,0x00,0x00,0x00,0x00,
    0x00,0x01
};

__code uint8_t CfgDesc[59] =
{
    0x09,0x02,0x3b,0x00,0x02,0x01,0x00,0xA0,0x32,             //configuration descriptor
    0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00,             //interface descriptor, keyboard
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00,             //HID class descriptor
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a,                       //endpoint descriptor
    0x09,0x04,0x01,0x00,0x01,0x03,0x01,0x02,0x00,             //interface descriptor, mouse
    0x09,0x21,0x10,0x01,0x00,0x01,0x22,0x34,0x00,             //HID class descriptor
    0x07,0x05,0x82,0x03,0x04,0x00,0x0a                        //endpoint descriptor
};

__code uint8_t KeyRepDesc[62] =
{
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};

__code uint8_t MouseRepDesc[52] =
{
    0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,
    0xA1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x03,
    0x81,0x02,0x75,0x05,0x95,0x01,0x81,0x01,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
    0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x03,
    0x81,0x06,0xC0,0xC0
};

uint8_t HIDMouse[4] = {0x0,0x0,0x0,0x0};


void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;                                                           // USB device mode.
    UEP2_DMA = (uint16_t)Ep2Buffer;                                            //Endpoint 2 data transfer address
    UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //Endpoint 2 transmit enable 64-byte buffer
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //Endpoint 2 automatically flips the synchronization flag, and the IN transaction returns NAK
    UEP0_DMA = (uint16_t)Ep0Buffer;                                            //Endpoint 0 data transfer address
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //Endpoint 0 single 64-byte transceiver buffer
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT transactions return ACK, IN transactions return NAK
    UEP1_DMA = (uint16_t)Ep1Buffer;                                            //Endpoint 1 data transfer address
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //Endpoint 1 transmit enable 64-byte buffer
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //Endpoint 1 automatically flips the synchronization flag, and the IN transaction returns NAK

    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_PD_DIS;                                                    // Disable DP/DM pull-down resistors
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // Start the USB device and DMA, and automatically return to NAK before the interrupt flag is cleared during the interrupt
    UDEV_CTRL |= bUD_PORT_EN;                                                  // allow usb ports
    USB_INT_FG = 0xFF;                                                         // clear interrupt flag
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

void usb_SendKbd(uint8_t* pSrc, uint8_t nBytes)
{
   	while(gbKbdTxDone == 0);
    memcpy(Ep1Buffer, pSrc, nBytes);
    UEP1_T_LEN = nBytes;
    UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
    gbKbdTxDone = 1;
	while(gbKbdTxDone == 0);
}

void usb_SendMouse(uint8_t* pSrc, uint8_t nBytes)
{
    memcpy(Ep2Buffer, pSrc, nBytes);
    UEP2_T_LEN = nBytes;
    UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void handle_Setup()
{
    static uint8_t nUsbConfig;  // Keep configuration value and response if requested.
    PUSB_SETUP_REQ pstSetupReq = (PUSB_SETUP_REQ)Ep0Buffer;
    uint8_t nRxLen = USB_RX_LEN;

    if(nRxLen == (sizeof(USB_SETUP_REQ)))
    {
        SetupLen = pstSetupReq->wLengthL;
        if(pstSetupReq->wLengthH || SetupLen > 0x7F)
        {
            SetupLen = 0x7F;    // Limit total length.
        }
        nRxLen = 0;                                                        // Default is success and upload 0 length
        SetupReq = pstSetupReq->bRequest;
        if ((pstSetupReq->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)// HID class command.
        {
            switch(SetupReq)
            {
                case 0x01://GetReport
                case 0x02://GetIdle
                case 0x03://GetProtocol
                case 0x09://SetReport
                case 0x0A://SetIdle
                case 0x0B://SetProtocol
                    break;
                default:
                    nRxLen = 0xFF; // command not supported
                    break;
            }
        }
        else // Standard request
        {
            switch(SetupReq)                                        // request code.
            {
                case USB_GET_DESCRIPTOR:
                {
                    switch(pstSetupReq->wValueH)
                    {
                        case 1:                                             // device descriptor.
                            gpNextTX = DevDesc;
                            nRxLen = sizeof(DevDesc);
                            break;
                        case 2:                                             // configuration descriptor
                            gpNextTX = CfgDesc;
                            nRxLen = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          // report descriptor
                            if(pstSetupReq->wIndexL == 0)                   // interface 0
                            {
                                gpNextTX = KeyRepDesc;
                                nRxLen = sizeof(KeyRepDesc);
                            }
                            else if(pstSetupReq->wIndexL == 1)              //  interface 1
                            {
                                gpNextTX = MouseRepDesc;
                                nRxLen = sizeof(MouseRepDesc);
                                gbReady = 1;   // If there are more interfaces, this standard bit should be valid after the last interface is configured
                            }
                            else
                            {
                                nRxLen = 0xff;                                 // wrong case.
                            }
                            break;
                        default:
                            nRxLen = 0xff;                                     // unsupported cmd.
                            break;
                    }
                    if (SetupLen > nRxLen)
                    {
                        SetupLen = nRxLen;    // Limit total length
                    }
                    nRxLen = (SetupLen >= DEFAULT_ENDP0_SIZE) ? DEFAULT_ENDP0_SIZE : SetupLen;                  // length of this transmission.
                    memcpy(Ep0Buffer, gpNextTX, nRxLen);
                    SetupLen -= nRxLen;
                    gpNextTX += nRxLen;
                    break;
                }
                case USB_SET_ADDRESS:
                    SetupLen = pstSetupReq->wValueL;                     // Temporarily store the address of the USB device
                    break;
                case USB_GET_CONFIGURATION:
                    Ep0Buffer[0] = nUsbConfig;
                    if (SetupLen >= 1)
                    {
                        nRxLen = 1;
                    }
                    break;
                case USB_SET_CONFIGURATION:
                    nUsbConfig = pstSetupReq->wValueL;
                    break;
                case USB_GET_INTERFACE:
                    break;
                case USB_CLEAR_FEATURE:                                            //Clear Feature
                    if ((pstSetupReq->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)// end point.
                    {
                        switch(pstSetupReq->wIndexL)
                        {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                nRxLen = 0xFF;
                                break;
                            }
                    }
                    if ((pstSetupReq->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) // equipment.
                    {
                        break;
                    }
                    else
                    {
                        nRxLen = 0xFF;                                                // not supported.
                    }
                    break;
                case USB_SET_FEATURE:
                    if((pstSetupReq->bRequestType & 0x1F) == 0x00) // Set up the device
                    {
                        if((((uint16_t)pstSetupReq->wValueH << 8) | pstSetupReq->wValueL) == 0x01)
                        {
                            if(CfgDesc[7] & 0x20)
                            {
                                /* Set wakeup enable flag */
                            }
                            else
                            {
                                nRxLen = 0xFF;                                        // failed
                            }
                        }
                        else
                        {
                            nRxLen = 0xFF;                                        // failed
                        }
                    }
                    else if ((pstSetupReq->bRequestType & 0x1F) == 0x02)        // set endpoint
                    {
                        if ((((uint16_t)pstSetupReq->wValueH << 8) | pstSetupReq->wValueL) == 0x00)
                        {
                            switch(((uint16_t)pstSetupReq->wIndexH << 8) | pstSetupReq->wIndexL)
                            {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;// set endpoint 2 IN STALL
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;// set endpoint 2 OUT Stall
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;// set endpoint 1 IN STALL
                                    break;
                                default:
                                    nRxLen = 0xFF;                               //failure
                                    break;
                            }
                        }
                        else
                        {
                            nRxLen = 0xFF;                                   //failure
                        }
                    }
                    else
                    {
                        nRxLen = 0xFF;                                      //failure
                    }
                    break;
                case USB_GET_STATUS:
                    Ep0Buffer[0] = 0x00;
                    Ep0Buffer[1] = 0x00;
                    if (SetupLen >= 2)
                    {
                        nRxLen = 2;
                    }
                    else
                    {
                        nRxLen = SetupLen;
                    }
                    break;
                default:
                    nRxLen = 0xff;                                           //failure
                    break;
            }
        }
    }
    else
    {
        nRxLen = 0xff;                                                   // wrong packet length.
    }

    if(nRxLen == 0xff)
    {
        SetupReq = 0xFF;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
    }
    else // if(nRxLen >= 0)
    {
        // if nRxLen == 0, although it has not yet reached the state stage, 
        // upload 0-length data packets in advance to prevent the host from entering the state stage in advance

        UEP0_T_LEN = nRxLen;
        //The default data packet is DATA1, and the response ACK is returned
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
    }
}

// Handle Upload event.
void handle_in(uint8_t nEP)
{
    switch(nEP)
    {
        case 2:
            UEP2_T_LEN = 0;   //The pre-used send length must be cleared
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Default answer NAK
            break;
        case 1:
            UEP1_T_LEN = 0;   //The pre-used send length must be cleared
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //Default answer NAK
            gbKbdTxDone = 1;  //transfer complete flag
            break;
        case 0:
        {
            uint8_t nLen;
            switch(SetupReq)
            {
                case USB_GET_DESCRIPTOR:
                    nLen = (SetupLen >= DEFAULT_ENDP0_SIZE) ? DEFAULT_ENDP0_SIZE : SetupLen;
                    memcpy(Ep0Buffer, gpNextTX, nLen);
                    SetupLen -= nLen;
                    gpNextTX += nLen;
                    UEP0_T_LEN = nLen;
                    UEP0_CTRL ^= bUEP_T_TOG;   //Sync flag flip.
                    break;
                case USB_SET_ADDRESS:
                    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
                default:
                    UEP0_T_LEN = 0;     // Status phase completion interrupt or forced upload of 0-length data packets to end control transmission
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                    break;
            }
            break;
        }
    }
}

void handle_out(uint8_t nEP)
{
    if(0 == nEP)
    {
        if(SetupReq == 0x09)
        {
            /**
             * BIT(0) : NUM LOCK
             * BIT(1) : CAPS LOCK
             * BIT(2) : SCROLL LOCK
             * BIT(3) : COMPOSE
             * BIT(4) : KANA
             * BIT(5-7) : CONSTANT
             * */
            gnReport = Ep0Buffer[0];
        }
        UEP0_CTRL ^= bUEP_R_TOG;                                     //Sync flag flip
    }
}

void ISR_USB() __interrupt (INT_NO_USB)
{
    if (UIF_TRANSFER)                                                            //USB transfer complete flag
    {
        uint8_t nEP = USB_INT_ST & MASK_UIS_ENDP;
        switch (USB_INT_ST & (MASK_UIS_TOKEN))
        {
            case UIS_TOKEN_SETUP:
                handle_Setup();
                break;
            case UIS_TOKEN_IN:                                                  //endpoint 2# upload
                handle_in(nEP);
                break;
            case UIS_TOKEN_OUT:  // endpoint0 OUT
                handle_out(nEP);
                break;
            default:
                break;
        }
        UIF_TRANSFER = 0;  // Write 0 to clear the interrupt
    }

    if(UIF_BUS_RST)  // Device Mode USB Bus Reset Interrupt
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;
    }

    if (UIF_SUSPEND)                                                     // USB bus suspend/wake up complete
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND)
        {
        }
    }
    else
    {                                                               // unexpected interrupt.
        USB_INT_FG = 0xFF;
    }
}


/*********************************
 * 
 *  Keyboard Handle.
 * 
 * *********************************/




/* Macro define */
#define		CH2				(0x04)
#define		CH3				(0x08)
#define		CH_FREE			(0x07)

#define		TH_VALUE		(100)
#define		TOUCH_NUM		(0x04)
#define		SAMPLE_TIMES	(0x05)

uint32_t gnMillis;

void ISR_Timer0() __interrupt (INT_NO_TMR0)
{
	TH0 = (65536 - 2000) / 256;  // Reload
	TL0 = (65536 - 2000) % 256;  // Reload
	gnMillis++;
}


__xdata uint8_t 	TK_Code[TOUCH_NUM] = {0x03, 0x04,};

__xdata uint16_t 		Key_FreeBuf[TOUCH_NUM];
__xdata uint8_t			Touch_IN;


uint8_t TK_SelectChannel(uint8_t ch)
{
	if (ch <= TOUCH_NUM)
	{
		TKEY_CTRL = (TKEY_CTRL & 0xF8) | TK_Code[ch];
		return 1;
	}

	return	0;
}

void ISR_Touch() __interrupt (INT_NO_TKEY)
{
    __xdata	static uint8_t nCh = 0;
    __xdata	uint16_t KeyData;
	KeyData = TKEY_DAT;

	if(KeyData < (Key_FreeBuf[nCh] - TH_VALUE))
	{
		Touch_IN |=  1 << (TK_Code[nCh] - 1);
	}
	if(++nCh >= TOUCH_NUM)
	{
		nCh = 0;
	}
	TK_SelectChannel(nCh);
}



uint8_t TK_Init(uint8_t channel)
{
    __xdata	uint8_t 	i,j;
    __xdata	uint16_t 	sum;
    __xdata	uint16_t 	OverTime;

	P1_DIR_PU &= ~channel;
	P1_MOD_OC &= ~channel;
	TKEY_CTRL |= bTKC_2MS ;

	for (i = 0; i < TOUCH_NUM; i++)
	{
		sum = 0;
		j = SAMPLE_TIMES;
		TK_SelectChannel(i);
		while(j--)
		{
			OverTime = 0;
			while((TKEY_CTRL & bTKC_IF) == 0) // Timing interrupt flag.
			{
				if(++OverTime == 0)
				{
					return 0;
				}
			}
			sum += TKEY_DAT;
		}
		Key_FreeBuf[i] = sum / SAMPLE_TIMES;
	}
	IE_TKEY = 1;
	return 1;
}

void main()
{
    CfgFsys();
    mDelaymS(5);
    mInitSTDIO();
    USBDeviceInit();

	TK_Init(0x30);
	TK_SelectChannel(0);
    kbd_init();

	TMOD = 0x11;
	TH0 = (65536 - 2000) / 256;  // for seed.
	TL0 = (65536 - 2000) % 256;  // for seed.
	TR0 = 1;    // Timer 0 start  - TR1 for time 1.
	ET0 = 1;    // Enable interrupt for timer 0
	EA  = 1;    // Activate global interrupt
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
    gbKbdTxDone = 0;
    gbReady = 0;

    while(1)
    {
        if(gbReady)
        {
            kbd_run();
        }
    }
}