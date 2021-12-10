#include <stdint.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>

#include "ch55x_mouse.h"

#define UsbSetupBuf ((USB_SETUP_REQ *)Ep0Buffer)

__xdata __at (FIXED_ADDRESS_EP0_BUFFER) uint8_t Ep0Buffer[DEFAULT_ENDP0_SIZE];	//Endpoint0 OUT&IN
__xdata __at (FIXED_ADDRESS_EP1_BUFFER) uint8_t Ep1Buffer[MAX_PACKET_SIZE];		//Endpoint1 IN

uint8_t SetupReq, SetupLen, UsbConfig;
USB_SETUP_REQ SetupReqBuf;
__code uint8_t *pDescr;

volatile __idata uint8_t RepDescSent;
volatile __idata uint8_t HIDDataSent;

uint8_t buttonsState = 0;

int8_t HIDMouse[4] = 
{
	0x0,	// Buttons
	0x0,	// X
	0x0,	// Y
	0x0		// Wheel
};

// Device Descriptor
__code uint8_t DevDesc[18] = 
{
	0x12,				// bLength
	0x01,				// bDescriptorType: DEVICE
	0x10, 0x01,			// bcdUSB: USB1.1
	0x00,				// bDeviceClass
	0x00,				// bDeviceSubClass
	0x00,				// bDeviceProtocol
	DEFAULT_ENDP0_SIZE,	// bMaxPacketSize0
	VENDOR_ID,			// idVendor
	PRODUCT_ID,			// idProduct
	0x00, 0x01,			// bcdDevice
	0x01,				// iManufacturer
	0x02,				// iProduct
	0x00,				// iSerialNumber
	0x01				// bNumConfigurations
};

// Configuration Descriptor
__code uint8_t CfgDesc[59] =
{
	// Device
	0x09,		// bLength
	0x02,		// bDescriptorType: CONFIGURATION
	0x22, 0x00,	// wTotalLength
	0x01,		// bNumInterface
	0x01,		// bConfigurationValue
	0x00,		// iConfiguration
	0x80,		// bmAttributes: Bus Power/No Remote Wakeup
	0x32,		// bMaxPower

	// Interface
	0x09,		// bLength
	0x04,		// bDescriptorType: INTERFACE
	0x00, 		// bInterfaceNumber
	0x00,		// bAlternateSetting
	0x01,		// bNumEndpoints
	0x03,		// bInterfaceClass: Human Interface Device (HID)
	0x01,		// bInterfaceSubClass
	0x02,		// bInterfaceProtocol: Mouse
	0x00,		// iInterface

	// HID
	0x09,		// bLength
	0x21,		// bDescriptorType: HID
	0x11, 0x01,	// bcdHID: 1.10
	0x00,		// bCountryCode
	0x01,		// bNumDescriptors
	0x22,		// bDescriptorType: Report
	0x34, 0x00,	// wDescriptorLength: 52

	// Endpoint
	0x07,		// bLength
	0x05,		// bDescriptorType: ENDPOINT
	0x81,		// bEndpointAddress: IN/Endpoint1
	0x03,		// bmAttributes: Interrupt
	MAX_PACKET_SIZE & 0xff, MAX_PACKET_SIZE >> 8, // wMaxPacketSize
	0x0a,		// bInterval
};

// HID Report Descriptor
__code uint8_t MouseRepDesc[52] =
{
	0x05, 0x01,		// USAGE_PAGE (Generic Desktop)
	0x09, 0x02,		// USAGE (Mouse)
	0xa1, 0x01,		// COLLECTION (Application)
	0x09, 0x01,		//   USAGE (Pointer)
	0xa1, 0x00,		//   COLLECTION (Physical)
	0x05, 0x09,		//	  USAGE_PAGE (Button)
	0x19, 0x01,		//	  USAGE_MINIMUM
	0x29, 0x03,		//	  USAGE_MAXIMUM
	0x15, 0x00,		//	  LOGICAL_MINIMUM (0)
	0x25, 0x01,		//	  LOGICAL_MAXIMUM (1)
	0x95, 0x03,		//	  REPORT_COUNT (3)
	0x75, 0x01,		//	  REPORT_SIZE (1)
	0x81, 0x02,		//	  INPUT (Data,Var,Abs)
	0x95, 0x01,		//	  REPORT_COUNT (1)
	0x75, 0x05,		//	  REPORT_SIZE (5)
	0x81, 0x03,		//	  INPUT (Const,Var,Abs)
	0x05, 0x01,		//	  USAGE_PAGE (Generic Desktop)
	0x09, 0x30,		//	  USAGE (X)
	0x09, 0x31,		//	  USAGE (Y)
	0x09, 0x38,		//	  USAGE (Wheel)
	0x15, 0x81,		//	  LOGICAL_MINIMUM (-127)
	0x25, 0x7f,		//	  LOGICAL_MAXIMUM (127)
	0x75, 0x08,		//	  REPORT_SIZE (8)
	0x95, 0x03,		//	  REPORT_COUNT (3)
	0x81, 0x06,		//	  INPUT (Data,Var,Rel)
	0xc0,			//   END_COLLECTION
	0xc0,			// END COLLECTION
};

// String Descriptor (Language)
__code unsigned char LangDesc[] = {0x04, 0x03, 0x09, 0x04};

// String Descriptor (Manufacturer)
__code unsigned char ManufDesc[] =
{
	sizeof(ManufDesc), 0x03, MANUFACTURER_DESCRIPTION
};

// String Descritor (Product)
__code unsigned char ProdDesc[] =
{
	sizeof(ProdDesc), 0x03, PRODUCT_DESCRIPTION
};

void USBInterruptHandler( void ) __interrupt (INT_NO_USB)
{
	uint8_t len;

	if(UIF_TRANSFER)
	{
		switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
		{
		case UIS_TOKEN_IN | 1:
			UEP1_T_LEN = 0;
			UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
			HIDDataSent = 1;
			break;
		case UIS_TOKEN_SETUP | 0:
			len = USB_RX_LEN;
			if(len == (sizeof(USB_SETUP_REQ)))
			{
				SetupLen = UsbSetupBuf->wLengthL;
				if(UsbSetupBuf->wLengthH || SetupLen > 0x7f )
				{
					SetupLen = 0x7f;
				}
				len = 0;
				SetupReq = UsbSetupBuf->bRequest;								
				if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
				{
					switch( SetupReq ) 
					{
						case 0x01:	//GetReport
							 break;
						case 0x02:	//GetIdle
							 break;	
						case 0x03:	//GetProtocol
							 break;				
						case 0x09:	//SetReport										
							 break;
						case 0x0a:	//SetIdle
							 break;	
						case 0x0b:	//SetProtocol
							 break;
						default:
							 len = 0xff;
							 break;
					  }	
				}
				else
				{
					switch(SetupReq)
					{
					case USB_GET_DESCRIPTOR:
						switch(UsbSetupBuf->wValueH)
						{
						case 1:
							pDescr = DevDesc;
							len = sizeof(DevDesc);
							break;
						case 2:
							pDescr = CfgDesc;
							len = sizeof(CfgDesc);
							break;
						case 3:
							if(UsbSetupBuf->wValueL == 0)
							{
								pDescr = LangDesc;
								len = sizeof(LangDesc);
							}
							else if(UsbSetupBuf->wValueL == 1)
							{
								pDescr = ManufDesc;
								len = sizeof(ManufDesc);
							}
							else if(UsbSetupBuf->wValueL == 2)
							{
								pDescr = ProdDesc;
								len = sizeof(ProdDesc);
							}
							else
							{
								len = 0xff;
							}
							break;

						case 0x22:
							if(UsbSetupBuf->wIndexL == 0)
							{
								pDescr = MouseRepDesc;
								len = sizeof(MouseRepDesc);
								RepDescSent = 1;
							}
							else
							{
								len = 0xff;
							}
							break;
						default:
							len = 0xff;
							break;
						}
						if ( SetupLen > len )
						{
							SetupLen = len;
						}
						len = SetupLen >= 8 ? 8 : SetupLen;
						memcpy(Ep0Buffer, pDescr, len);
						SetupLen -= len;
						pDescr += len;
						break;
					case USB_SET_ADDRESS:
						SetupLen = UsbSetupBuf->wValueL;
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
					case 0x0a:
						break;
					case USB_CLEAR_FEATURE:
						if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
						{
							switch( UsbSetupBuf->wIndexL )
							{
							case 0x81:
								UEP1_CTRL = UEP1_CTRL & ~( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
								break;
							case 0x01:
								UEP1_CTRL = UEP1_CTRL & ~( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
								break;
							default:
								len = 0xff;
								break;
							}
						}
						else
						{
							len = 0xff;
						}
						break;
					case USB_SET_FEATURE:
						if( ( UsbSetupBuf->bRequestType & 0x1f ) == 0x00 )
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
							{
								if( CfgDesc[ 7 ] & 0x20 )
								{
								}
								else
								{
									len = 0xff;
								}
							}
							else
							{
								len = 0xff;
							}
						}
						else if( ( UsbSetupBuf->bRequestType & 0x1f ) == 0x02 )
						{
							if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
							{
								switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
								{
								case 0x81:
									UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;
									break;
								default:
									len = 0xff;
									break;
								}
							}
							else
							{
								len = 0xff;
							}
						}
						else
						{
							len = 0xff;
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
						len = 0xff;
						break;
					}
				}
			}
			else
			{
				len = 0xff;
			}
			if(len == 0xff)
			{
				SetupReq = 0xff;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
			}
			else if(len <= 8)
			{
				UEP0_T_LEN = len;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
			}
			else
			{
				UEP0_T_LEN = 0;
				UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
			}
			break;
		case UIS_TOKEN_IN | 0:	//endpoint0 IN
			switch(SetupReq)
			{
			case USB_GET_DESCRIPTOR:
				len = SetupLen >= 8 ? 8 : SetupLen;
				memcpy( Ep0Buffer, pDescr, len );
				SetupLen -= len;
				pDescr += len;
				UEP0_T_LEN = len;
				UEP0_CTRL ^= bUEP_T_TOG;
				break;
			case USB_SET_ADDRESS:
				USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			default:
				UEP0_T_LEN = 0;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			break;
		case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
			UEP0_T_LEN = 0;
			UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;
			break;
		default:
			break;
		}
		UIF_TRANSFER = 0;
	}
	if(UIF_BUS_RST)
	{
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;
	}
	if (UIF_SUSPEND)
	{
		UIF_SUSPEND = 0;
	}
	else {
		USB_INT_FG = 0xff;
	}
}

void HIDValueHandle(uint8_t button, int8_t x, int8_t y, int8_t wheel)
{
	HIDDataSent = 0;		
	HIDMouse[0] = button;
	HIDMouse[1] = x;
	HIDMouse[2] = y;
	HIDMouse[3] = wheel;
	memcpy(Ep1Buffer, HIDMouse, sizeof(HIDMouse));
	UEP1_T_LEN = sizeof(HIDMouse);
	UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
	while(HIDDataSent == 0) {}
}

void ch55x_mouse_init()
{
	IE_USB = 0;
	USB_CTRL = 0x00;
	UEP0_DMA = (uint16_t)Ep0Buffer;
	UEP1_DMA = (uint16_t)Ep1Buffer;
	UEP4_1_MOD = ~(bUEP4_RX_EN | bUEP4_TX_EN | bUEP1_RX_EN | bUEP1_BUF_MOD) | bUEP4_TX_EN;
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	UEP1_CTRL = bUEP_T_TOG | UEP_T_RES_NAK;
	UEP1_T_LEN = 0;

	USB_DEV_AD = 0x00;
	UDEV_CTRL = bUD_PD_DIS;
	USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;

	UDEV_CTRL |= bUD_PORT_EN;
	USB_INT_FG = 0xff;
	USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	IE_USB = 1;

	EA = 1;

	RepDescSent = 0;
	while(RepDescSent == 0) {};
}

void ch55x_mouse_move(int8_t delta_x, int8_t delta_y)
{
	HIDValueHandle(0, delta_x, delta_y, 0);
}

void ch55x_mouse_scroll(int8_t delta)
{
	HIDValueHandle(0, 0, 0, delta);
}

void ch55x_mouse_press(uint8_t buttons)
{
	buttonsState |= buttons & MOUSE_BUTTONS_MASK;
	HIDValueHandle(buttonsState, 0, 0, 0);
}

void ch55x_mouse_release(uint8_t buttons)
{
	buttonsState &= ~(buttons & MOUSE_BUTTONS_MASK);
	HIDValueHandle(buttonsState, 0, 0, 0);
}

void ch55x_mouse_release_all()
{
	HIDValueHandle(0, 0, 0, 0);
}
