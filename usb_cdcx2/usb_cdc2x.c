﻿/********************************** (C) COPYRIGHT *******************************
* File Name          : CDCx2.C
* Author             : qianfan Zhao
* Version            : V1.0
* Date               : 2021/05/27
* Description        : CH552 virtual two channel CDC serial port
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>

#define CDC_LOOPBACK

#ifndef CDC_LOOPBACK
uint32_t get_baud_rate(__xdata uint8_t *acm_line_code)
{
	uint32_t baud = 0;

	baud |= acm_line_code[3];
	baud <<= 8;
	baud |= acm_line_code[2];
	baud <<= 8;
	baud |= acm_line_code[1];
	baud <<= 8;
	baud |= acm_line_code[0];
	return baud;
}
#endif

/*
 * Endpoint configuration:
 * CDC0: bulk_in(0x82), bulk_out(0x02), int_in(0x81)
 * CDC1: bulk_in(0x83), bulk_out(0x03), int_in(0x04)
 */
__xdata __at (0x0000) uint8_t  gaBuf4EP0[DEFAULT_ENDP0_SIZE];
__xdata __at (0x0040) uint8_t  gaBuf4EP1[8];
__xdata __at (0x0080) uint8_t  gaBuf4EP2[2 * MAX_PACKET_SIZE];	// IN buffer + OUT buffer
__xdata __at (0x0100) uint8_t  gaBuf4EP3[2 * MAX_PACKET_SIZE];	// IN buffer + OUT buffer
__xdata __at (0x0180) uint8_t  gaBuf4EP4[8];

#define  SET_LINE_CODING                0X20	// Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING                0X21	// This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE         0X22	// This request generates RS-232/V.24 style control signals.

/*
 * usb device descriptors are generated by: https://xtoolbox.gitee.io/teenydt/
 * return Device {
 *    strManufacturer = "dog2nd_Zhao",
 *    strProduct = "USB CDCx2",
 *    strSerial =  "USB123456",
 *    idVendor = 0x1a86,
 *    idProduct = 0x5723,
 *    prefix = "CDC_ACM2",
 *    Config {
 *        CDC_ACM{
 *            EndPoint(IN(1), Interrupt, 8),
 *            EndPoint(IN(2), Bulk, 64),
 *            EndPoint(OUT(2), Bulk, 64),
 *        },
 *        CDC_ACM{
 *            EndPoint(IN(4),  Interrupt, 8),
 *            EndPoint(IN(3), Bulk, 64),
 *            EndPoint(OUT(3),  Bulk, 64),
 *        },
 *   }
 *}
*/

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_IAD_DESCRIPTOR_TYPE                 0x0B
#define USB_HUB_DESCRIPTOR_TYPE                 0x29
#define USB_HID_DESCRIPTOR_TYPE                 0x21
#define USB_REPORT_DESCRIPTOR_TYPE              0x22
#define USB_DESC_TYPE_REPORT                    0x22
#define USB_FUCNTION_DESCRIPTOR_TYPE            0x24

__code const uint8_t CDC_ACM2_DeviceDescriptor [18] = {
  ///////////////////////////////////////
  /// device descriptor
  ///////////////////////////////////////
  0x12,                                             /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  0x00, 0x02,                                       /* bcdUSB */
  0xef,                                             /* bDeviceClass */
  0x02,                                             /* bDeviceSubClass */
  0x01,                                             /* bDeviceProtocol */
  DEFAULT_ENDP0_SIZE,                               /* bMaxPacketSize */
  0x86, 0x1a,                                       /* idVendor */
  0x23, 0x57,                                       /* idProduct */
  0x00, 0x01,                                       /* bcdDevice */
  0x01,                                             /* iManufacturer */
  0x02,                                             /* iProduct */
  0x03,                                             /* iSerial */
  0x01,                                             /* bNumConfigurations */
};

__code const uint8_t CDC_ACM2_ConfigDescriptor1 [141] = {
  ///////////////////////////////////////
  /// config descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,                /* bDescriptorType */
  0x8d, 0x00,                                       /* wTotalLength */
  0x04,                                             /* bNumInterfaces */
  0x01,                                             /* bConfigurationValue */
  0x00,                                             /* iConfiguration */
  0x80,                                             /* bmAttributes */
  0x64,                                             /* bMaxPower */

  ///////////////////////////////////////
  /// interface association descriptor
  ///////////////////////////////////////
  0x08,                                             /* bLength */
  USB_IAD_DESCRIPTOR_TYPE,                          /* bDescriptorType */
  0x00,                                             /* bFirstInterface */
  0x02,                                             /* bInterfaceCount */
  0x02,                                             /* bFunctionClass */
  0x02,                                             /* bFunctionSubClass */
  0x01,                                             /* bFunctionProtocol */
  0x00,                                             /* iFunction */

  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x00,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x01,                                             /* bNumEndpoints */
  0x02,                                             /* bInterfaceClass */
  0x02,                                             /* bInterfaceSubClass */
  0x01,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */

  ///////////////////////////////////////
  /// cdc acm header descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                                             /* bDescriptorType */
  0x00,                                             /* bDescriptorSubtype */
  0x10, 0x01,                                       /* bcdCDC */

  ///////////////////////////////////////
  /// cdc acm call management descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x01,                                             /* bDescriptorSubtype */
  0x00,                                             /* bmCapabilities */
  0x01,                                             /* bDataInterface */

  ///////////////////////////////////////
  /// cdc acm descriptor
  ///////////////////////////////////////
  0x04,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x02,                                             /* bDescriptorSubtype */
  0x02,                                             /* bmCapabilities */

  ///////////////////////////////////////
  /// cdc acm union descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x06,                                             /* bDescriptorSubtype */
  0x00,                                             /* bMasterInterface */
  0x01,                                             /* bSlaveInterface0 */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x81,                                             /* bEndpointAddress */
  0x03,                                             /* bmAttributes */
  0x08, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */

  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x01,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x02,                                             /* bNumEndpoints */
  0x0a,                                             /* bInterfaceClass */
  0x00,                                             /* bInterfaceSubClass */
  0x00,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x82,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x40, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x02,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x40, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */

  ///////////////////////////////////////
  /// interface association descriptor
  ///////////////////////////////////////
  0x08,                                             /* bLength */
  USB_IAD_DESCRIPTOR_TYPE,                          /* bDescriptorType */
  0x02,                                             /* bFirstInterface */
  0x02,                                             /* bInterfaceCount */
  0x02,                                             /* bFunctionClass */
  0x02,                                             /* bFunctionSubClass */
  0x01,                                             /* bFunctionProtocol */
  0x00,                                             /* iFunction */

  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x02,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x01,                                             /* bNumEndpoints */
  0x02,                                             /* bInterfaceClass */
  0x02,                                             /* bInterfaceSubClass */
  0x01,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */

  ///////////////////////////////////////
  /// cdc acm header descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x00,                                             /* bDescriptorSubtype */
  0x10, 0x01,                                       /* bcdCDC */

  ///////////////////////////////////////
  /// cdc acm call management descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x01,                                             /* bDescriptorSubtype */
  0x00,                                             /* bmCapabilities */
  0x01,                                             /* bDataInterface */

  ///////////////////////////////////////
  /// cdc acm descriptor
  ///////////////////////////////////////
  0x04,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x02,                                             /* bDescriptorSubtype */
  0x02,                                             /* bmCapabilities */

  ///////////////////////////////////////
  /// cdc acm union descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  USB_FUCNTION_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x06,                                             /* bDescriptorSubtype */
  0x02,                                             /* bMasterInterface */
  0x03,                                             /* bSlaveInterface0 */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x84,                                             /* bEndpointAddress */
  0x03,                                             /* bmAttributes */
  0x08, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */

  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x03,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x02,                                             /* bNumEndpoints */
  0x0a,                                             /* bInterfaceClass */
  0x00,                                             /* bInterfaceSubClass */
  0x00,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x83,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x40, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */

  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x03,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x40, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */
};

__code const uint8_t CDC_ACM2_StringDescriptor0 [0x04] = {
  0x04,                                         /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
  0x09, 0x04,                                   /* wLangID0 */
};

__code const uint8_t CDC_ACM2_StringDescriptor1 [0x18] = {	// "dog2nd_Zhao"
  0x18,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'd', 0x00,'o', 0x00,'g', 0x00,'2', 0x00,'n', 0x00,'d', 0x00,'_', 0x00,'Z', 0x00,'h', 0x00,'a', 0x00,'o', 0x00
  };

__code const uint8_t CDC_ACM2_StringDescriptor2 [0x14] = {	// "USB CBCx2"
  0x14,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'U', 0x00,'S', 0x00,'B', 0x00,' ', 0x00,'C', 0x00,'D', 0x00,'C', 0x00,'x', 0x00,'2', 0x00
  };

__code const uint8_t CDC_ACM2_StringDescriptor3 [0x14] = {	// "USB123456"
  0x14,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'U', 0x00,'S', 0x00,'B', 0x00,'1', 0x00,'2', 0x00,'3', 0x00,'4', 0x00,'5', 0x00,'6', 0x00,
};

#ifndef CDC_LOOPBACK

#define UART_FIFO_SIZE			64
#define UART_FIFO_SIZE_MASK		(UART_FIFO_SIZE - 1)

struct uart_fifo {
	uint8_t			uart_fifo[UART_FIFO_SIZE];
	uint8_t			uart_fifo_w_idx, uart_fifo_r_idx;
};

static volatile __idata struct uart_fifo uart0 = {
	.uart_fifo_w_idx = 0,
	.uart_fifo_r_idx = 0,
};

static volatile __idata struct uart_fifo uart1 = {
	.uart_fifo_w_idx = 0,
	.uart_fifo_r_idx = 0,
};

#define uart_fifo_is_empty(uart)								\
	((uart)->uart_fifo_w_idx == (uart)->uart_fifo_r_idx)
#define uart_fifo_is_full(uart)									\
	((((uart)->uart_fifo_w_idx - (uart)->uart_fifo_r_idx) & UART_FIFO_SIZE_MASK) == UART_FIFO_SIZE_MASK)
#define uart_fifo_length(uart)									\
	(((uart)->uart_fifo_w_idx - (uart)->uart_fifo_r_idx) & UART_FIFO_SIZE_MASK)

#define uart_fifo_put(uart, b) do {								\
	if (!uart_fifo_is_full(uart)) {								\
		(uart)->uart_fifo[(uart)->uart_fifo_w_idx] = b;					\
		(uart)->uart_fifo_w_idx = ((uart)->uart_fifo_w_idx + 1) & UART_FIFO_SIZE_MASK;	\
	}											\
}while (0)

#define uart_fifo_get_without_check(uart, b) do {						\
	*(b) = (uart)->uart_fifo[(uart)->uart_fifo_r_idx];					\
	(uart)->uart_fifo_r_idx = ((uart)->uart_fifo_r_idx + 1) & UART_FIFO_SIZE_MASK;		\
} while (0)
#endif

typedef struct _cdc_device
{
	uint8_t				anLineCoding[7];	// CDC configuration information.
	uint8_t				bUploadBusy;
	uint8_t				nRxSize;
	uint8_t				nNxtByte;	///< Next byte to handle, data is EP buffer.
#ifndef CDC_LOOPBACK
	uint8_t				usb_fifo_idx;
	__idata struct uart_fifo	*uart;
#endif
} CdcDevice;

/* the default baud rate is 57600-8-N-1 */
static volatile __xdata CdcDevice gstCdc0 =
{
	.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
	.bUploadBusy = 0,
	.nRxSize = 0,
	.nNxtByte = 0,
#ifndef CDC_LOOPBACK
	.usb_fifo_idx = 0,
	.uart = &uart0,
#endif
};

static volatile __xdata CdcDevice gstCdc1 =
{
	.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
	.bUploadBusy = 0,
	.nRxSize = 0,
	.nNxtByte = 0,
#ifndef CDC_LOOPBACK
	.usb_fifo_idx = 0,
	.uart = &uart1,
#endif
};

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description    : USB device mode configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceCfg()
{
	// default: Full speed 12M mode.
	USB_CTRL =  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;		//USB device and internal pull-up enable, automatically return to NAK before interrupt flag is cleared
	USB_DEV_AD = 0x00;						// Device address initialization
	UDEV_CTRL = bUD_PD_DIS | bUD_PORT_EN;	// Disable DP/DM pull-down resistor, Enable physical port
}
/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description    : USB device mode interrupt initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceIntCfg()
{
	// Enable suspend, Tx completion, bus reset interrupt.
	USB_INT_EN |= bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	USB_INT_FG |= 0x1F;					//Clear interrupt flag
	IE_USB = 1;							//Enable USB interrupt
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description    : USB device mode endpoint configuration, simulation compatible HID device, in addition to endpoint 0 control transmission, also includes endpoint 2 batch upload
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceEndPointCfg()
{
	UEP0_DMA = (uint16_t) gaBuf4EP0;	// Endpoint 0 data transfer address
	UEP1_DMA = (uint16_t) gaBuf4EP1;	// Endpoint 1 sends data transfer address
	UEP2_DMA = (uint16_t) gaBuf4EP2;	// Endpoint 2 IN data transfer address
	UEP3_DMA = (uint16_t) gaBuf4EP3;
	UEP2_3_MOD = 0xCC;					// Endpoint 2/3 single buffer TX/RX enable.
	UEP4_1_MOD = 0x44;					// Endpoint 1 TX buffer;
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;	// Manual flip, OUT transaction returns ACK, IN transaction returns NAK
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;	// Endpoint 1 automatically flips the synchronization flag, IN transaction returns NAK
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;	//Endpoint 2 automatically flips the synchronization flag, IN transaction returns NAK, OUT returns ACK
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
}

static volatile uint8_t gnUsbNewAddr = 0;
static volatile uint8_t gnUsbCfg = 0;
static const uint8_t *gpSetupXferPoint = NULL;
static volatile uint16_t gnSetupXferSize = 0;
static volatile USB_SETUP_REQ gstUsbLastSetupReq;

void usb_irq_reset_handler(void)
{
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
	USB_DEV_AD = 0x00;
	UIF_SUSPEND = 0;
	UIF_TRANSFER = 0;
	UIF_BUS_RST = 0;

	gstCdc0.bUploadBusy = 0;
	gstCdc0.nRxSize = 0;
	gstCdc1.bUploadBusy = 0;
	gstCdc1.nRxSize = 0;

	gnUsbCfg = 0;
}

void usb_irq_suspend_handler(void)
{
	UIF_SUSPEND = 0;
	if ( USB_MIS_ST & bUMS_SUSPEND )
	{
		while ( XBUS_AUX & bUART0_TX ) {;} // Wait Xfer done.

		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; // Wakeup on USB event, RXD0/1 signal low.
		PCON |= PD; // Sleep.
		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = 0x00;
	}
}

/***
 * Handle data sending.
 */
void handle_in(uint8_t nEP)
{
	switch (nEP)
	{
		case 1: /* CDC0 interrupt in, NAK */
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			break;

		case 2: /* CDC0 bulk in, NAK and clear busy flag */
			UEP2_T_LEN = 0;
			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			gstCdc0.bUploadBusy = 0;
			break;

		case 3: /* CDC1 bulk in, NAK and clear busy flag */
			UEP3_T_LEN = 0;
			UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			gstCdc1.bUploadBusy = 0;
			break;

		case 4: /* CDC1 interrupt in, NAK */
			UEP4_T_LEN = 0;
			UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
			break;

		case 0: /* EP0 packet transfer */
			switch(gstUsbLastSetupReq.bRequest)
			{
				case USB_GET_DESCRIPTOR:
				{
					int nThisLen = gnSetupXferSize;

					if (nThisLen == 0 && !gpSetupXferPoint)
					{
						/* nothing need sending, force ending setup transfer */
						UEP0_T_LEN = 0;
						UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
						break;
					}
					else if (nThisLen > DEFAULT_ENDP0_SIZE)
					{
						nThisLen = DEFAULT_ENDP0_SIZE;
					}

					memcpy(gaBuf4EP0, gpSetupXferPoint, nThisLen);
					gpSetupXferPoint += nThisLen;
					gnSetupXferSize -= nThisLen;
					if (nThisLen < DEFAULT_ENDP0_SIZE && gnSetupXferSize == 0)
					{
						gpSetupXferPoint = NULL;
					}

					UEP0_T_LEN = nThisLen;
					UEP0_CTRL ^= bUEP_T_TOG;
					break;
				}
				case USB_SET_ADDRESS:
				{
					USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | gnUsbNewAddr;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
				default:
				{
					UEP0_T_LEN = 0;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
			}
			break;
	}
}

/**
 * Handle data receiving.
 **/
void handle_out(uint8_t nEP)
{
	__xdata CdcDevice *pstCdc = NULL;

	if (U_TOG_OK)  /* Out of sync packets will be dropped. */
	{
		switch (nEP)
		{
			case 2: /* CDC0 bulk out, save recved length and NAK */
			{
				gstCdc0.nRxSize = USB_RX_LEN;
				// NAK: Not ACK, Host don't resume send.
				// ACK will issued on main function.
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
				break;
			}
			case 3: /* CDC1 bulk out */
			{
				gstCdc1.nRxSize = USB_RX_LEN;
				// NAK: Not ACK, Host don't resume send.
				UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
				break;
			}
			case 0:
			{
				switch (gstUsbLastSetupReq.bRequest)
				{
					case SET_LINE_CODING:
					{
						if (gstUsbLastSetupReq.wIndexL == 0) /* interface 0 is CDC0 */
						{
							pstCdc = &gstCdc0;
#ifndef CDC_LOOPBACK
							uint32_t baud = get_baud_rate(gaBuf4EP0);
							TH1 = 256 - FREQ_SYS / baud / 16;
#endif
						}
						else if (gstUsbLastSetupReq.wIndexL == 2) /* interface 2 is CDC1 */
						{
							pstCdc = &gstCdc1;
#ifndef CDC_LOOPBACK
							uint32_t baud = get_baud_rate(gaBuf4EP0);
							SBAUD1 = 256 - FREQ_SYS / 16 / baud;
#endif
						}

						if (NULL != pstCdc)
						{
							memcpy(pstCdc->anLineCoding, gaBuf4EP0, USB_RX_LEN);
							UEP0_T_LEN = 0;
							UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;
						}
						else
						{
							UEP0_T_LEN = 0;
							UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
						}
						break;
					}
					default:
					{
						UEP0_T_LEN = 0;
						UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
						break;
					}
				}
				break;
			}
		}
	}
}

/**
 * Get Descriptor information into gpSetupXferPoint/gnSetupXferSize
 **/
int handle_request_get_desc(uint8_t nDescType, uint8_t nStrIdx)
{
	const uint8_t *pstDest = NULL;
	int nSize = 0;

	switch (nDescType)
	{
		case 1: /* device descriptor */
			pstDest = CDC_ACM2_DeviceDescriptor;
			nSize = sizeof(CDC_ACM2_DeviceDescriptor);
			break;

		case 2: /* config descriptor */
			pstDest = CDC_ACM2_ConfigDescriptor1;
			nSize = sizeof(CDC_ACM2_ConfigDescriptor1);
			break;

		case 3: /* string descriptor */
			switch (nStrIdx)
			{
				case 0:
					pstDest = CDC_ACM2_StringDescriptor0;
					nSize = sizeof(CDC_ACM2_StringDescriptor0);
					break;
				case 1: /* iManufacturer */
					pstDest = CDC_ACM2_StringDescriptor1;
					nSize = sizeof(CDC_ACM2_StringDescriptor1);
					break;
				case 2:  /* iProduct */
					pstDest = CDC_ACM2_StringDescriptor2;
					nSize = sizeof(CDC_ACM2_StringDescriptor2);
					break;
				case 3: /* iSerial */
					pstDest = CDC_ACM2_StringDescriptor3;
					nSize = sizeof(CDC_ACM2_StringDescriptor3);
					break;
				default:
					return -1;
			}
			break;
		default:
			return -1;
	}

	if (nSize > gnSetupXferSize)
		nSize = gnSetupXferSize;
	gnSetupXferSize = nSize;
	gpSetupXferPoint = pstDest;

	return 0;
}

int handle_setup_standard(PXUSB_SETUP_REQ req)
{
	switch (req->bRequest)
	{
		case USB_GET_DESCRIPTOR:
			return handle_request_get_desc(req->wValueH, req->wValueL);
		case USB_SET_ADDRESS:
			/* new address is addressed after device ACK */
			gnUsbNewAddr = req->wValueL;
			gpSetupXferPoint = NULL;
			gnSetupXferSize = 0;
			break;
		case USB_GET_CONFIGURATION:
			gnSetupXferSize = sizeof(gnUsbCfg);
			gpSetupXferPoint = &gnUsbCfg;
			break;
		case USB_SET_CONFIGURATION:
			gnUsbCfg = req->wValueL;
			gpSetupXferPoint = NULL;
			gnSetupXferSize = 0;
			break;
		case USB_GET_INTERFACE:
			break;
		default:
			return -1;
	}

	return 0;
}

int handle_setup_vendor(PXUSB_SETUP_REQ pstReq)
{
	switch (pstReq->bRequest)
	{
		case GET_LINE_CODING:
		{
			__xdata CdcDevice *pstCurCdc = NULL;
			if (pstReq->wIndexL == 0)  /* interface 0 is gstCdc0 */
				pstCurCdc = &gstCdc0;
			else if (pstReq->wIndexL == 2) /* interface 2 is gstCdc1 */
				pstCurCdc = &gstCdc1;
			else
				return -1;

			gpSetupXferPoint = pstCurCdc->anLineCoding;
			break;
		}
		case SET_CONTROL_LINE_STATE:
		case SET_LINE_CODING:
			gpSetupXferPoint = NULL;
			gnSetupXferSize = 0;
			/* setting data packet defined in EP0 packet out */
			break;
		default:
			return -1;
	}

	return 0;
}

/**
 * Setup handler called only for EP0
 **/
void handle_setup(uint8_t nEP)
{
	if (nEP == 0)
	{
		PXUSB_SETUP_REQ pstReq = (PXUSB_SETUP_REQ)gaBuf4EP0;
		int bFailed = -1;
	
		if(USB_RX_LEN == (sizeof(USB_SETUP_REQ)))
		{
			gnSetupXferSize = ((uint16_t)pstReq->wLengthH << 8) | pstReq->wLengthL;
			memcpy(&gstUsbLastSetupReq, pstReq, sizeof(gstUsbLastSetupReq));

			if ((pstReq->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_STANDARD)
				bFailed = handle_setup_standard(pstReq);
			else
				bFailed = handle_setup_vendor(pstReq);
		}

		if (bFailed || (gnSetupXferSize > 0 && !gpSetupXferPoint))
		{
			/* STALL request */
			UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
			return;
		}

		int n1stSize = (gnSetupXferSize > DEFAULT_ENDP0_SIZE) ? DEFAULT_ENDP0_SIZE : gnSetupXferSize;

		memcpy(gaBuf4EP0, gpSetupXferPoint, n1stSize);
		/* The last packet of data is less than the maximum length of EP0, no need to add empty packets */
		if (gnSetupXferSize == 0 && n1stSize < DEFAULT_ENDP0_SIZE)
			gpSetupXferPoint = NULL;

		gpSetupXferPoint += n1stSize;
		gnSetupXferSize -= n1stSize;

		UEP0_T_LEN = n1stSize;
		UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
	}
}

void DeviceInterrupt(void) __interrupt (INT_NO_USB)                       //USB interrupt service routine, using register set 1
{
	if (UIF_TRANSFER) // Transfer done (partial or full)
	{
		uint8_t nEP = USB_INT_ST & MASK_UIS_ENDP;
		switch (USB_INT_ST & MASK_UIS_TOKEN)
		{
			case UIS_TOKEN_IN:
				handle_in(nEP);
				break;
			case UIS_TOKEN_OUT:
				handle_out(nEP);
				break;
			case UIS_TOKEN_SETUP:
				handle_setup(nEP);
				break;
		}
		UIF_TRANSFER = 0; /* clear interrupt */
	}
	else if (UIF_BUS_RST)
	{
		usb_irq_reset_handler();
	}
	else if (UIF_SUSPEND)
	{
		usb_irq_suspend_handler();
	}
	else
	{
		USB_INT_FG = 0xFF;
	}
}

#ifndef CDC_LOOPBACK
void uart0_isr(void) __interrupt (INT_NO_UART0)
{
	if (RI)
	{
		uart_fifo_put(&uart0, SBUF);
		RI = 0;
	}
}

void uart1_isr(void) __interrupt (INT_NO_UART1)
{
	if (U1RI)
	{
		uart_fifo_put(&uart1, SBUF1);
		U1RI = 0;
	}
}
#endif

uint8_t usb_send_cdc0(uint8_t* pSrc, uint8_t nSize)
{
	uint8_t nSizeTx = 0;
	if(!gstCdc0.bUploadBusy)
	{
		nSizeTx = (nSize > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : nSize;
		__xdata uint8_t *pTX = gaBuf4EP2 + MAX_PACKET_SIZE;

		memcpy(pTX, pSrc, nSizeTx);

		UEP2_T_LEN = nSizeTx;
		UEP2_CTRL &= ~MASK_UEP_T_RES | UEP_T_RES_ACK;
		gstCdc0.bUploadBusy = 1;
	}
	return nSizeTx;
}

uint8_t usb_send_cdc1(uint8_t* pSrc, uint8_t nSize)
{
	uint8_t nSizeTx = 0;
	if(!gstCdc1.bUploadBusy)
	{
		nSizeTx = (nSize > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : nSize;
		__xdata uint8_t *pTX = gaBuf4EP3 + MAX_PACKET_SIZE;

		memcpy(pTX, pSrc, nSizeTx);

		UEP3_T_LEN = nSizeTx;
		UEP3_CTRL &= ~MASK_UEP_T_RES | UEP_T_RES_ACK;
		gstCdc1.bUploadBusy = 1;
	}
	return nSizeTx;
}

uint8_t usb_get_cdc0(uint8_t* pDst, uint8_t nMaxSize)
{
	uint8_t nByte = gstCdc0.nRxSize > nMaxSize ? nMaxSize : gstCdc0.nRxSize;
	
	if(nByte > 0)
	{
		memcpy(pDst, gaBuf4EP2 + gstCdc0.nNxtByte, nByte);
		gstCdc0.nRxSize -= nByte;
		if(0 == gstCdc0.nRxSize)
		{
			gstCdc0.nNxtByte = 0; // resume.
			UEP2_CTRL &= ~MASK_UEP_R_RES | UEP_R_RES_ACK;
		}
		else
		{
			gstCdc0.nNxtByte += nByte;
		}
	}
	return nByte;
}

uint8_t usb_get_cdc1(uint8_t* pDst, uint8_t nMaxSize)
{
	uint8_t nByte = gstCdc1.nRxSize > nMaxSize ? nMaxSize : gstCdc1.nRxSize;
	
	if(nByte > 0)
	{
		memcpy(pDst, gaBuf4EP3 + gstCdc1.nNxtByte, nByte);
		gstCdc1.nRxSize -= nByte;
		if(0 == gstCdc1.nRxSize)
		{
			gstCdc1.nNxtByte = 0; // resume.
			UEP3_CTRL &= ~MASK_UEP_R_RES | UEP_R_RES_ACK;
		}
		else
		{
			gstCdc1.nNxtByte += nByte;
		}
	}
	return nByte;	
}


void main()
{
	uint8_t t0 = 0;	// timeout check.
	uint8_t t1 = 0;	// timeout check.

	CfgFsys( );
	mDelaymS(5);
	mInitSTDIO();
	UART1Setup();
	/* TI(Transmit Interrupt Flag) is seted in mInitSTDIO */
	TI = 0;

	USBDeviceCfg();
	USBDeviceEndPointCfg();
	USBDeviceIntCfg();
	EA = 1;		//Allow microcontroller interrupt

	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;
	UEP2_T_LEN = 0;
	UEP3_T_LEN = 0;
	UEP4_T_LEN = 0;

	ES = 1;
	PS = 1;

	logic_init();

	IE_UART1 = 1;
	IP_EX |= bIP_UART1;


	while(1)
	{
		if(gnUsbCfg)
		{
#ifdef CDC_LOOPBACK  // Cross loopback.
			logic_run();
#else
			int len0, len1;

			len0 = uart_fifo_length(gstCdc0.uart);
			len1 = uart_fifo_length(gstCdc1.uart);

			if (len0 > 0)
				t0++;
			if (len1 > 0)
				t1++;

			if (!gstCdc0.bUploadBusy && (t0 > 100 || len0 > UART_FIFO_SIZE >> 1))
			{
				__xdata uint8_t *ep2_out = gaBuf4EP2 + MAX_PACKET_SIZE;
				int i;

				t0 = 0;
				for (i = 0; i < len0; i++) {
					uint8_t b;

					uart_fifo_get_without_check(gstCdc0.uart, &b);
					ep2_out[i] = b;
				}

				UEP2_T_LEN = len0;
				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
				gstCdc0.bUploadBusy = 1;
			}

			if (gstCdc0.nRxSize > 0)
			{
				__xdata uint8_t *ep2_in = gaBuf4EP2;

				CH554UART0SendByte(ep2_in[gstCdc0.usb_fifo_idx++]);
				if (--gstCdc0.nRxSize == 0) {
					gstCdc0.usb_fifo_idx = 0;
					/* gstCdc0: ready, continue recving */
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
				}
			}

			if (!gstCdc1.bUploadBusy && (t1 > 100 || len1 > UART_FIFO_SIZE >> 1))
			{
				__xdata uint8_t *ep3_out = gaBuf4EP3 + MAX_PACKET_SIZE;
				int i;

				t1 = 0;
				for (i = 0; i < len1; i++)
				{
					uint8_t b;

					uart_fifo_get_without_check(gstCdc1.uart, &b);
					ep3_out[i] = b;
				}

				UEP3_T_LEN = len1;
				UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
				gstCdc1.bUploadBusy = 1;
			}

			if (gstCdc1.nRxSize > 0)
			{
				__xdata uint8_t *ep3_in = gaBuf4EP3;

				CH554UART1SendByte(ep3_in[gstCdc1.usb_fifo_idx++]);
				if (--gstCdc1.nRxSize == 0)
				{
					gstCdc1.usb_fifo_idx = 0;
					/* gstCdc1: ready, continue recving */
					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
				}
			}
#endif
		}
	}
}
