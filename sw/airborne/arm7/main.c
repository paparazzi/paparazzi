/******************************************************************************
 *
 * WinARM Demo-Application 
 * by Martin THOMAS <eversmith@heizung-thomas.de>
 * based on code from Bill Knight, R O SoftWare <BillK@rosw.com>
 *
 * - UART0 and UART1 send and receive (also in Interrupt-Mode)
 * - Sends message if button "INT1" (P0.14) on demo-board have been hit.
 * - LED blink (2 of the 8 LEDs on the MCB2130)
 * - Thumb/Interwork (makefile)
 * - Adapted to the Keil MCB2130 demo-board (Philips LPC2138) (config.h)
 *
 *
 * $RCSfile$
 * $Revision$
 *
 *****************************************************************************/
 
#include "types.h"
#include "LPC21xx.h"
#include "config.h"
#include "armVIC.h"
#include "sysTime.h"
#include "uart.h"
#include "adc.h"

/******************************************************************************
 *
 * Function Name: lowInit()
 *
 * Description:
 *    This function starts up the PLL then sets up the GPIO pins before
 *    waiting for the PLL to lock.  It finally engages the PLL and
 *    returns
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
static void lowInit(void)
{
	// set PLL multiplier & divisor.
	// values computed from config.h
	PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;
	
	// enable PLL
	PLLCON = PLLCON_PLLE;
	PLLFEED = 0xAA;                       // Make it happen.  These two updates
	PLLFEED = 0x55;                       // MUST occur in sequence.
	
	// setup port pins
	IO0CLR = PIO0_ZERO_BITS;                // clear the ZEROs output
	IO0SET = PIO0_ONE_BITS;                 // set the ONEs output
	IO0DIR = PIO0_OUTPUT_BITS;              // set the output bit direction
	
	IO1CLR = PIO1_ZERO_BITS;                // clear the ZEROs output
	IO1SET = PIO1_ONE_BITS;                 // set the ONEs output
	IO1DIR = PIO1_OUTPUT_BITS;              // set the output bit direction
	
	// wait for PLL lock
	while (!(PLLSTAT & PLLSTAT_LOCK))
		continue;
	
	// enable & connect PLL
	PLLCON = PLLCON_PLLE | PLLCON_PLLC;
	PLLFEED = 0xAA;                       // Make it happen.  These two updates
	PLLFEED = 0x55;                       // MUST occur in sequence.
	
	// setup & enable the MAM
	MAMTIM = MAMTIM_CYCLES;
	MAMCR = MAMCR_FULL;
	
	// set the peripheral bus speed
	// value computed from config.h
	VPBDIV = VPBDIV_VALUE;                // set the peripheral bus clock speed
}

/******************************************************************************
 *
 * Function Name: sysInit()
 *
 * Description:
 *    This function is responsible for initializing the program
 *    specific hardware
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
static void sysInit(void)
{
	lowInit();                            // setup clocks and processor port pins
	
	// set the interrupt controller defaults
#if defined(RAM_RUN)
	MEMMAP = MEMMAP_SRAM;                 // map interrupt vectors space into SRAM
#elif defined(ROM_RUN)
	MEMMAP = MEMMAP_FLASH;                // map interrupt vectors space into FLASH
#else
#error RUN_MODE not defined!
#endif

	VICIntEnClear = 0xFFFFFFFF;           // clear all interrupts
	VICIntSelect = 0x00000000;            // clear all FIQ selections
	VICDefVectAddr = (uint32_t)reset;     // point unvectored IRQs to reset()
	
	//  wdtInit();                        // initialize the watchdog timer
	initSysTime();                        // initialize the system timer
	
	uart0Init(UART_BAUD(HOST_BAUD), UART_8N1, UART_FIFO_8); // setup the UART
	//	uart1Init(UART_BAUD(HOST_BAUD), UART_8N1, UART_FIFO_8); // setup the UART
	adcInit();
}

/******************************************************************************
 *
 * Function Name: button_state()
 *
 * Description:
 *    This function checks if a key has been pressed. Assumes
 *    keys to be "active low" (PIN-Bit==0 -> pressed). Does
 *    debouncing for given debounce time-difference
 *
 * Calling Sequence: 
 *    GPIO-Initialisation for Inputs
 *
 * Returns:
 *    -1 : keys changed or bouncing
 *     0 : keys released
 *     1 : key1 pressed
 *
 *****************************************************************************/
#define KEY_DEBOUNCE FIFTY_MS
static int16_t button_state(void)
{
	static uint32_t lastchangetime;
	static uint32_t laststate;
	int16_t retval = 0;
	uint32_t actstate;
	
	actstate = (IO0PIN & (SW1_BIT));
	
	if (laststate != actstate) {
		lastchangetime = getSysTICs();
		laststate = actstate;
	}
	else {
		if (getElapsedSysTICs(lastchangetime) > KEY_DEBOUNCE) {
			retval = 0;
			if ( !(IO0PIN & SW1_BIT) ) retval |= 0x01;
			return retval;
		}
	}
	return -1; // changed or bouncing
}

void uart0_print_hex(const uint8_t c) {
  const unsigned char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  unsigned char high = (c & 0xF0)>>4;
  unsigned char low  = c & 0x0F;
  uart0Putch(hex[high]);
  uart0Putch(hex[low]);
}

void uart0_print_hex_16(const uint16_t c) {
  unsigned char high = (c & 0xFF00)>>8;
  unsigned char low  = c & 0x00FF;
  uart0_print_hex(high);
  uart0_print_hex(low);
}

void uart0_print_hex_32(const uint32_t c) {
  uint16_t high = (c & 0xFFFF0000)>>16;
  uint16_t low  = c & 0x0000FFFF;
  uart0_print_hex_16(high);
  uart0_print_hex_16(low);
}

/******************************************************************************
 *
 * Function Name: main()
 *
 * Description:
 *    This function is the program entry point.  After initializing the
 *    system, it sends a greeting out UART0 then enters an endless loop
 *    echoing chracters on the UART and blinking an LED every half
 *    second.
 *
 * Calling Sequence: 
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
int main(void)
{
	uint32_t startTime;
	boolean  lock=FALSE;
	int16_t  bt;
	
	sysInit();
#if defined(UART0_TX_INT_MODE) || defined(UART0_RX_INT_MODE) || \
    defined(UART1_TX_INT_MODE) || defined(UART1_RX_INT_MODE)
	enableIRQ();
#endif
	startTime = getSysTICs();
	
	uart0Puts("\r\nHello World! (UART0)\r\n");
	uart0Puts("A WinARM Demo-Application by Martin Thomas\r\n");
	uart0Puts("based on code from Bill Knight\r\n\r\n");
//	uart1Puts("\r\nHello World! (UART1)\r\n");
	
	for (;;) {
		do {
			int ch;
	
			if ((ch = uart0Getch()) >= 0) {
				uart0Puts("the <");
				uart0Putch(ch);
				uart0Puts("> key has been pressed on UART0\r\n");
			}

//			if ((ch = uart1Getch()) >= 0) {
//				uart1Puts("the <");
//				uart1Putch(ch);
//				uart1Puts("> key has been pressed on UART1\r\n");
//			}

			// send button-pressed string only once if hit
			if (button_state()==0) lock=FALSE; // release lock if button is released
			
			if ( ((bt=button_state()) > 0 ) && !lock ) {
				if ( bt & 0x01 ) {
					uart0Puts("\r\nButton 1 Pressed!\r\n");
//					uart1Puts("\r\nButton 1 Pressed!\r\n");
					lock=TRUE;
				}
			}
		} while (getElapsedSysTICs(startTime) < HALF_SEC);
		
		if (IO0PIN & LED1_BIT) {
			IO0CLR = LED1_BIT;
			IO0SET = LED2_BIT;
		}
		else {
			IO0SET = LED1_BIT; 
			IO0CLR = LED2_BIT; 
		}
		uart0Puts("\r\ntick!\r\n");
		uart0_print_hex_16(adc0_val[0]);
		uart0Puts(", ");
		uart0_print_hex_16(adc0_val[1]);
		uart0Puts(", ");
		uart0_print_hex_16(adc0_val[2]);
		uart0Puts(", ");
		uart0_print_hex_16(adc0_val[3]);
		uart0Puts(", ");
		uart0_print_hex_16(adc0_val[4]);
		uart0Puts(", ");
		uart0_print_hex_16(adc0_val[5]);
		uart0Puts("\r\n");
		uart0_print_hex_16(adc1_val[0]);
		uart0Puts(", ");
		uart0_print_hex_16(adc1_val[1]);
		uart0Puts(", ");
		uart0_print_hex_16(adc1_val[2]);
		uart0Puts(", ");
		uart0_print_hex_16(adc1_val[3]);
		uart0Puts(", ");
		uart0_print_hex_16(adc1_val[4]);
		uart0Puts(", ");
		uart0_print_hex_16(adc1_val[5]);
		uart0Puts("\r\n");
		startTime += HALF_SEC;
	} // for
	
	return 0;
}
