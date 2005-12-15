#include "types.h"
#include "LPC21xx.h"
#include "config.h"
#include "armVIC.h"
#include "sysTime.h"
#include "uart.h"
#include "adc.h"
#include "traces.h"
#include "servos.h"
#include "ppm.h"
#include "radio_control.h"
#include "gps_crado.h"


static void periodic_task ( void );

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
static void lowInit(void) {
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

static void sysInit(void) {
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
  uart1Init(B38400, UART_8N1, UART_FIFO_8);               // setup the UART
  adc_init();
  servos_init();
  ppm_init();
  radio_control_init();
}

static void periodic_task ( void ) {
  static uint32_t foo = 0;
  foo++;
  if (!(foo%10)) {
    if (IO0PIN & LED1_BIT)
      IO0CLR = LED1_BIT;
    else
      IO0SET = LED1_BIT; 
    //    PRINT_ADC();
  }
  radio_control_periodic_task();
  if (rc_status == RC_OK) 
    IO0CLR = LED2_BIT;
  else
    IO0SET = LED2_BIT;
}

#define PERIODIC_TASK_PERIOD FIFTY_MS

int main (void) {
  uint32_t startTime;
  
  sysInit();
  enableIRQ();
  startTime = getSysTICs();
  uart0Puts("\r\nTiny booting (UART0)\r\n");
  while (1) {
    if (getElapsedSysTICs(startTime) >= PERIODIC_TASK_PERIOD) {
      periodic_task();
      startTime += PERIODIC_TASK_PERIOD; 
    }
    int ch;
    if ((ch = uart1Getch()) >= 0) {
      parse_ubx(ch);
      if (gps_msg_received) {
	parse_gps_msg();
	gps_msg_received = FALSE;
      }
      if (gps_pos_available) {
	PRINT_GPS();
	gps_pos_available = FALSE;
      }
    }
    if (ppm_valid) {
      radio_control_process_ppm();
      ppm_valid = FALSE;
      //      PRINT_PPM();
    }
  }
  return 0;
}
