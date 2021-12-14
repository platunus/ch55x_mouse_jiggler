#define VENDOR_ID	0xc0, 0x16		// 16c0
#define PRODUCT_ID	0xda, 0x27		// 27da

#define MANUFACTURER_DESCRIPTION	\
	'N', 0x00, 'S', 0x00, ' ', 0x00, 'T', 0x00, 'e', 0x00, 'c', 0x00, 'h', 0x00, ' ', 0x00, \
	'R', 0x00, 'e', 0x00, 's', 0x00, 'e', 0x00, 'a', 0x00, 'r', 0x00, 'c', 0x00, 'h', 0x00
#define PRODUCT_DESCRIPTION	\
	'M', 0x00, 'o', 0x00, 'u', 0x00, 's', 0x00, 'e', 0x00, ' ', 0x00, \
	'J', 0x00, 'i', 0x00, 'g', 0x00, 'g', 0x00, 'l', 0x00, 'e', 0x00, 'r', 0x00

#define MOUSE_LEFT_BUTTON	1
#define MOUSE_RIGHT_BUTTON	2
#define MOUSE_MIDDLE_BUTTON	4
/*
#define MOUSE_EXTEND1_BUTTON	8
#define MOUSE_EXTEND2_BUTTON	16
#define MOUSE_BUTTONS_MASK	0b00011111
*/
#define MOUSE_BUTTONS_MASK	0b00000111

/*
Memory map:
EP0 Buf     00 - 07 
EP1 Buf     10 - 4f 
*/
#define FIXED_ADDRESS_EP0_BUFFER    0x0000  
#define FIXED_ADDRESS_EP1_BUFFER    0x0010 

void ch55x_mouse_init();
void ch55x_mouse_move(int8_t delta_x, int8_t delta_y);
void ch55x_mouse_scroll(int8_t delta);
void ch55x_mouse_press(uint8_t buttons);
void ch55x_mouse_release(uint8_t buttons);
void ch55x_mouse_release_all();

void USBInterruptHandler( void ) __interrupt (INT_NO_USB);

