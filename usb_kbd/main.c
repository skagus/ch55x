typedef unsigned char                 *PUINT8;
typedef unsigned char volatile          UINT8V;

#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>
#include <bootloader.h>

__xdata uint8_t readFlag = 0;
__xdata char sPath[] = "This is the CH552 sample text. Press Num to toggle the LED.";
__xdata char *pStr = sPath;
uint32_t millis, last,last1;

#define LED_PIN1 0
SBIT(LED1, 0xB0, LED_PIN1);
#define LED_PIN2 1
SBIT(LED2, 0xB0, LED_PIN2);

SBIT(Ep2InKey, 0xB0, 2);

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];
__xdata __at (0x0050) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];
__xdata __at (0x000a) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];

uint8_t Ready;  // USB setup done! work device!.
uint8_t   SetupReq,SetupLen,FLAG;
uint8_t UsbConfig;  // Keep configuration value and response if requested.
PUINT8  pDescr;
uint8_t b,numlock;

void jump_to_bootloader()
{
	USB_INT_EN = 0;
	USB_CTRL = 0x06;
	EA = 0;
	mDelaymS(100);
	bootloader();
	while(1);
}

void mTimer0Interrupt( void ) __interrupt (INT_NO_TMR0)
{
	TH0 = (65536 - 2000)/256;  // Reload
	TL0 = (65536 - 2000)%256;  // Reload
	millis++;
}

/* Macro define */
#define		CH2				(0x04)
#define		CH3				(0x08)
#define		CH_FREE			(0x07)

#define		TH_VALUE		(100)
#define		TOUCH_NUM		(0x04)
#define		SAMPLE_TIMES	(0x05)

__xdata uint8_t 	TK_Code[TOUCH_NUM] = {
	0x03, 0x04,
};

__xdata uint16_t 		Key_FreeBuf[TOUCH_NUM];
__xdata UINT8V			Touch_IN;

uint8_t TK_SelectChannel( uint8_t ch )
{
	if ( ch <= TOUCH_NUM )
	{
		TKEY_CTRL = ( TKEY_CTRL & 0xF8) | TK_Code[ch];
		return 1;
	}

	return	0;
}

uint8_t TK_Init( uint8_t channel)
{
    __xdata	uint8_t 	i,j;
    __xdata	uint16_t 	sum;
    __xdata	uint16_t 	OverTime;

	P1_DIR_PU &= ~channel;
	P1_MOD_OC &= ~channel;
	TKEY_CTRL |= bTKC_2MS ;

	for ( i = 0; i < TOUCH_NUM; i++ )
	{
		sum = 0;
		j = SAMPLE_TIMES;
		TK_SelectChannel( i );
		while( j-- )
		{
			OverTime = 0;
			while( ( TKEY_CTRL & bTKC_IF ) == 0 )
			{
				if( ++OverTime == 0 )
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

void	TK_int_ISR( void ) __interrupt (INT_NO_TKEY)
{
    __xdata	static uint8_t ch = 0;
    __xdata	uint16_t KeyData;
	KeyData = TKEY_DAT;

	if( KeyData < ( Key_FreeBuf[ch] - TH_VALUE ) )
	{
		Touch_IN |=  1 << ( TK_Code[ch] - 1 );
	}
	if( ++ch >= TOUCH_NUM )
	{
		ch = 0;
	}
	TK_SelectChannel( ch );
}



#define     UsbSetupBuf             ((PUSB_SETUP_REQ)Ep0Buffer)

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

__code uint8_t DevDesc[18] = {0x12,0x01,0x10,0x01,0x00,0x00,0x00,0x08,
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
uint8_t HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};


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

void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey));
    UEP1_T_LEN = sizeof(HIDKey);
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDMouse, sizeof(HIDMouse));
    UEP2_T_LEN = sizeof(HIDMouse);
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void DeviceInterrupt(void) __interrupt (INT_NO_USB)
{
    uint8_t len = 0;
    if(UIF_TRANSFER)                                                            //USB transfer complete flag
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# upload
            UEP2_T_LEN = 0;                                                     //The pre-used send length must be cleared
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //If you do not set automatic flip, you need to flip manually
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Default answer NAK
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# upload.
            UEP1_T_LEN = 0;                                                     //The pre-used send length must be cleared
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //If you do not set automatic flip, you need to flip manually
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Default answer NAK
            FLAG = 1;                                                           //transfer complete flag
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // Limit total length.
                }
                len = 0;                                                        // Default is success and upload 0 length
                SetupReq = UsbSetupBuf->bRequest;
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )// HID class command.
                {
                    switch( SetupReq )
                    {
                        case 0x01://GetReport
                        case 0x02://GetIdle
                        case 0x03://GetProtocol
                        case 0x09://SetReport
                        case 0x0A://SetIdle
                        case 0x0B://SetProtocol
                            break;
                        default:
                            len = 0xFF; // command not supported
                            break;
                    }
                }
                else // Standard request
                {
                    switch(SetupReq)                                        // request code.
                    {
                        case USB_GET_DESCRIPTOR:
                        {
                            switch(UsbSetupBuf->wValueH)
                            {
                                case 1:                                             // device descriptor.
                                    pDescr = DevDesc;
                                    len = sizeof(DevDesc);
                                    break;
                                case 2:                                             // configuration descriptor
                                    pDescr = CfgDesc;
                                    len = sizeof(CfgDesc);
                                    break;
                                case 0x22:                                          // report descriptor
                                    if(UsbSetupBuf->wIndexL == 0)                   // interface 0
                                    {
                                        pDescr = KeyRepDesc;
                                        len = sizeof(KeyRepDesc);
                                    }
                                    else if(UsbSetupBuf->wIndexL == 1)              //  interface 1
                                    {
                                        pDescr = MouseRepDesc;
                                        len = sizeof(MouseRepDesc);
                                        Ready = 1;                                  // If there are more interfaces, this standard bit should be valid after the last interface is configured
                                    }
                                    else
                                    {
                                        len = 0xff;                                 // wrong case.
                                    }
                                    break;
                                default:
                                    len = 0xff;                                     // unsupported cmd.
                                    break;
                            }
                            if ( SetupLen > len )
                            {
                                SetupLen = len;    // Limit total length
                            }
                            len = SetupLen >= 8 ? 8 : SetupLen;                  // length of this transmission.
                            memcpy(Ep0Buffer,pDescr,len);
                            SetupLen -= len;
                            pDescr += len;
                            break;
                        }
                        case USB_SET_ADDRESS:
                            SetupLen = UsbSetupBuf->wValueL;                     // Temporarily store the address of the USB device
                            break;
                        case USB_GET_CONFIGURATION:
                            Ep0Buffer[0] = UsbConfig;
                            if ( SetupLen >= 1 )
                            {
                                len = 1;
                            }
                            break;
                        case USB_SET_CONFIGURATION:
                            UsbConfig = UsbSetupBuf->wValueL;
                            break;
                        case 0x0A:
                            break;
                        case USB_CLEAR_FEATURE:                                            //Clear Feature
                            if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// end point.
                            {
                                switch( UsbSetupBuf->wIndexL )
                                {
                                    case 0x82:
                                        UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                        break;
                                    case 0x81:
                                        UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                        break;
                                    case 0x01:
                                        UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                        break;
                                    default:
                                        len = 0xFF;
                                        break;
                                    }
                            }
                            if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE ) // equipment.
                            {
                                break;
                            }
                            else
                            {
                                len = 0xFF;                                                // not supported.
                            }
                            break;
                        case USB_SET_FEATURE:
                            if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 ) // Set up the device
                            {
                                if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                                {
                                    if( CfgDesc[ 7 ] & 0x20 )
                                    {
                                        /* Set wakeup enable flag */
                                    }
                                    else
                                    {
                                        len = 0xFF;                                        // failed
                                    }
                                }
                                else
                                {
                                    len = 0xFF;                                        // failed
                                }
                            }
                            else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        // set endpoint
                            {
                                if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                                {
                                    switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
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
                                            len = 0xFF;                               //failure
                                            break;
                                    }
                                }
                                else
                                {
                                    len = 0xFF;                                   //failure
                                }
                            }
                            else
                            {
                                len = 0xFF;                                      //failure
                            }
                            break;
                        case USB_GET_STATUS:
                            Ep0Buffer[0] = 0x00;
                            Ep0Buffer[1] = 0x00;
                            if ( SetupLen >= 2 )
                            {
                                len = 2;
                            }
                            else
                            {
                                len = SetupLen;
                            }
                            break;
                        default:
                            len = 0xff;                                           //failure
                            break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   // wrong packet length.
            }

            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len > 0)                                                // The upload data or status phase returns a 0-length packet
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
            }
            else
            {
                UEP0_T_LEN = 0;  // Although it has not yet reached the state stage, upload 0-length data packets in advance to prevent the host from entering the state stage in advance
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default data packet is DATA1, and the response ACK is returned
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
                case USB_GET_DESCRIPTOR:
                    len = SetupLen >= 8 ? 8 : SetupLen;
                    memcpy( Ep0Buffer, pDescr, len );
                    SetupLen -= len;
                    pDescr += len;
                    UEP0_T_LEN = len;
                    UEP0_CTRL ^= bUEP_T_TOG;                                     //Sync flag flip.
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
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if(SetupReq == 0x09)
            {
                if(Ep0Buffer[0])
                {
                    numlock = 1;
                }
                else if(Ep0Buffer[0] == 0)
                {
                    numlock = 0;
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG;                                     //Sync flag flip
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
        if ( USB_MIS_ST & bUMS_SUSPEND )
        {
        }
    }
    else
    {                                                               // unexpected interrupt.
        USB_INT_FG = 0xFF;
    }
}


static void SendKey ( char *p )
{
	char c = *p;
	char d = 0;

	if( (c >= 'a') && (c <= 'z' ))
    {
		c = c - 'a' + 'A';
		d=1;
	}
	if(d == 0)HIDKey[0] = L_SHIFT;
	if( (c >= 'A') && (c <= 'Z' ))
    {
		HIDKey[2] = c - 'A' + 4;
	}
	else if( c >= '1' && c <= '9' )
    {
        HIDKey[0] = 0x000;
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
	while(FLAG == 0);
	Enp1IntIn();
	while(FLAG == 0);
	mDelaymS( 10 );
	HIDKey[0] = 0x00;
	HIDKey[2] = 0x00;
	while(FLAG == 0);
	Enp1IntIn();
	while(FLAG == 0);
}


void HIDValueHandle()
{
    if( readFlag == 1 )
    {
        SendKey(pStr);
        pStr++;
        if(*pStr == '\0')
        {
            readFlag = 0;
            b=0;
        }
    }
    else
    {
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
        if(b == 1)
        {
            b=0;
            pStr = sPath;
            readFlag=1;
        }
    }
}


main()
{
    CfgFsys( );
    mDelaymS(5);
    mInitSTDIO( );
    USBDeviceInit();
	TK_Init( 0x10 + 0x20);
	TK_SelectChannel(0);

    P3_MOD_OC = P3_MOD_OC |(1<<LED_PIN1);
    P3_DIR_PU = P3_DIR_PU |	(1<<LED_PIN1);
    P3_MOD_OC = P3_MOD_OC |(1<<LED_PIN2);
    P3_DIR_PU = P3_DIR_PU |	(1<<LED_PIN2);


	TMOD = 0x11;
	TH0 = (65536 - 2000)/256;  // for seed.
	TL0 = (65536 - 2000)%256;  // for seed.
	TR0 = 1;    // Timer 0 start  - TR1 for time 1.
	ET0 = 1;    // Enable interrupt for timer 0
	EA  = 1;    // Activate global interrupt
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
    FLAG = 0;
    Ready = 0;
	b=0;
    while(1)
    {
        if (millis-last > 40)
        {
            LED1 = !LED1;
            last = millis;
            LED2 = numlock;
            if (Touch_IN != 0)
            {
                if (Touch_IN & CH2) jump_to_bootloader();
                if (Touch_IN & CH3) b=1;
                Touch_IN = 0;
            }

            if (Ready)
            {
                HIDValueHandle();
            }
        }
    }
}