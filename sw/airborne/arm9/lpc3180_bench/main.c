
/*********************************************************FileHeaderBegin******
 *
 * FILE:
 *     main.c
 *
 * REVISION:
 *
 * AUTHOR:
 *     (C) 2008, Dirk Behme, dirk.behme@gmail.com
 *
 * CREATED:
 *     29.06.2008
 *
 * DESCRIPTION:
 *     Basic test routines for Phytec LPC3180 board.
 *
 * NOTES:
 *
 * MODIFIED:
 *
 * LICENSE:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 **********************************************************FileHeaderEnd******/

/* We can run without any library or with standard C library (newlib?) */
#define USE_C_LIB

#ifdef USE_C_LIB
#include <stdio.h>
#else
static int print_string(char *);
void print_int(unsigned int);
#endif

static void init_irq(void);
static void init_hstimer(void);
static void init_led(void);
static void get_system_info(void);
extern void uart5_putc(char);
extern void test_vfp(void);

#define WELCOME_STRING " demo on LPC3180! Version: "__DATE__" "__TIME__"\n"
#define COPYRIGHT "(C) 2008, Dirk Behme, dirk.behme@gmail.com\n"

#define REG32(x) (*(volatile unsigned int *)(x))

/******************************************************************************
 * Main function
 *****************************************************************************/
int main(void) {
  
  /* Output welcome message... */
#ifdef USE_C_LIB
  printf(WELCOME_STRING);
  printf(COPYRIGHT);
#else
  print_string(WELCOME_STRING);
  print_string(COPYRIGHT);
#endif

  get_system_info();
  init_irq();
  init_led();
  init_hstimer();
  test_vfp();

  return 0;
}

/******************************************************************************
 * Get and print system information
 *****************************************************************************/

#define CLOCK_BASE   (0x40004000)
#define PWR_CTRL     REG32(CLOCK_BASE + 0x44)
#define SYCLK_CTRL   REG32(CLOCK_BASE + 0x50)
#define HCLKPLL_CTRL REG32(CLOCK_BASE + 0x58)
#define HCLKDIV_CTRL REG32(CLOCK_BASE + 0x40)


static void get_system_info(void) {

  unsigned int val;
  char *info1, *info2, *info3;

  printf("System information:\n");

  asm ("mrc p15, 0, %0, c0, c0, 0":"=r" (val));

  if(((val & 0xFFF0) >> 4)== 0x926)
    info1 = "ARM926EJ-S";
  else
    info1 = "unknown";

  if(((val & 0xF0000) >> 16)== 0x6)
    info2 = "ARMv5TEJ";
  else
    info2 = "unknown";

  printf(" CPU:           %s\n", info1);
  printf(" Architecture:  %s\n", info2);

  asm ("mrc p15, 0, %0, c0, c0, 1":"=r" (val));

  if((val & 0x3) == 0x2)
    info1 = "8 words per line";
  else
    info1 = "unknown";

  if(((val & 0x38) >> 3) == 0x2)
    info2 = "4 way";
  else
    info2 = "unknown";

  if(((val & 0x3C0) >> 6) == 0x6)
    info3 = "32KB";
  else
    info3 = "unknown";

  printf(" Instr. cache:  %s, %s, %s\n", info3, info2, info1);

  if(((val & 0x3000) >> 12) == 0x2)
    info1 = "8 words per line";
  else
    info1 = "unknown";

  if(((val & 0x38000) >> 15) == 0x2)
    info2 = "4 way";
  else
    info2 = "unknown";

  if(((val & 0x3C0000) >> 18) == 0x6)
    info3 = "32KB";
  else
    info3 = "unknown";

  printf(" Data cache:    %s, %s, %s\n", info3, info2, info1);

  asm ("mrc p15, 0, %0, c1, c0, 0":"=r" (val));

  if(val & (1 << 12))
    info1 = "on";
  else
    info1 = "off";

  if(val & (1 << 2))
    info2 = "on";
  else
    info2 = "off";

  if(val & (1 << 0))
    info3 = "on";
  else
    info3 = "off";

  printf(" System config: ICache: %s, DCache: %s, MMU: %s\n",
	 info1, info2, info3);
  printf(" SDRAM:         32MB, 32bit\n");

  val = SYCLK_CTRL;
  if((val & 0x1) == 0) {
    /* 13 MHz osciallator used */
    val = PWR_CTRL;
    if(((val & (1 << 10)) == 0) && ((val & (1 << 2)) == (1 << 2))) {
      /* Normal mode & normal RUN mode */
      if((HCLKPLL_CTRL == 0x0001601f) && (HCLKDIV_CTRL == 0x0000003d)) {
	info1 = "208MHz";
	info2 = "104MHz";
      } else {
	info1 = "unknown";
	info2 = "unknown";
    }
    } else {
      info1 = "unknown";
      info2 = "unknown";
    }
  } else {
    info1 = "unknown";
    info2 = "unknown";
  }
  printf(" Clocks:        Processor: %s, SDRAM: %s\n\n", info1, info2);
}


/******************************************************************************
 * IRQ handling
 *****************************************************************************/

#define NUM_IRQ (3*32) /* Max 96 interrupts */

#define MIC_BASE   (0x40008000)
#define MIC_ER     REG32(MIC_BASE + 0x00)
#define MIC_RSR    REG32(MIC_BASE + 0x04)
#define MIC_SR     REG32(MIC_BASE + 0x08)
#define MIC_APR    REG32(MIC_BASE + 0x0C)
#define MIC_ATR    REG32(MIC_BASE + 0x10)
#define MIC_ITR    REG32(MIC_BASE + 0x14)

#define SIC1_BASE  (0x4000C000)
#define SIC1_ER    REG32(SIC1_BASE + 0x00)
#define SIC1_RSR   REG32(SIC1_BASE + 0x04)
#define SIC1_SR    REG32(SIC1_BASE + 0x08)
#define SIC1_APR   REG32(SIC1_BASE + 0x0C)
#define SIC1_ATR   REG32(SIC1_BASE + 0x10)
#define SIC1_ITR   REG32(SIC1_BASE + 0x14)

#define SIC2_BASE  (0x40010000)
#define SIC2_ER    REG32(SIC2_BASE + 0x00)
#define SIC2_RSR   REG32(SIC2_BASE + 0x04)
#define SIC2_SR    REG32(SIC2_BASE + 0x08)
#define SIC2_APR   REG32(SIC2_BASE + 0x0C)
#define SIC2_ATR   REG32(SIC2_BASE + 0x10)
#define SIC2_ITR   REG32(SIC2_BASE + 0x14)

typedef void(*irq_handler_t)(unsigned int);

irq_handler_t irq_handler[NUM_IRQ] = {0};

/*
 * Register an IRQ handler for an IRQ number
 * Returns NULL if no handler is registered yet, else the old handler.
 *
 * IRQ numbers:
 *             Main interrupt controller:
 *              5   HSTIMER_INT
 *             ..   ...
 *             28   DMAINT
 *
 *             Sub interrupt controller 1:
 *             32   Reserved
 *             33   JTAG_COMM_TX
 *             ..   ...    
 *             63   USB_I2C_INT
 *
 *             Sub interrupt controller 2:
 *             64   GPIO_00
 *             65   GPIO_01
 *             ..   ...    
 *             95   SYSCLK_MUX
 */
irq_handler_t register_irq(irq_handler_t new_handler,
			   unsigned int irq_number) {

  irq_handler_t old_handler;

  old_handler = irq_handler[irq_number];
  irq_handler[irq_number] = new_handler;

  return old_handler;
}

void dispatch_irq(void) {

  unsigned int mic_sr, sic1_sr, sic2_sr, irq_reg, irq_offs, i;
  unsigned int irq_number;

  /* Scan interrupt status registers */
  mic_sr  = MIC_SR;
  sic1_sr = SIC1_SR;
  sic2_sr = SIC2_SR;

  if((mic_sr & 0x3) == 0) {
    /* MIC interrupt */
    irq_reg  = mic_sr;
    irq_offs = 0;
  } else if ((mic_sr & 0x1) == 1) {
    /* SIC1 interrupt */
    irq_reg = sic1_sr;
    irq_offs = 32;
  } else if ((mic_sr & 0x2) == 2) {
    /* SIC2 interrupt */
    irq_reg = sic2_sr;
    irq_offs = 64;
  } else {
    printf("Error: Spurious interrupt1\n");
    return;
  }

  /* Calculate IRQ number */
  for(i = 0; i < 32; i++) {
    if(irq_reg & (1 << i))
      break;
  }

  if(i == 32) {
    printf("Error: Spurious interrupt2\n");
    return;
  }

  irq_number = i + irq_offs;

  /* Call the handler */
  if(irq_handler[irq_number] != 0) {
    (*irq_handler[irq_number])(irq_number);
  } else {
    printf("Error: Unhandled interrupt %i\n", irq_number);
  }
}

static void init_irq(void) {

  /* Disable all interrupts */
  MIC_ER  = 0;
  SIC1_ER = 0;
  SIC2_ER = 0;

  /* We only do IRQ, no FIQ */
  MIC_ITR = 0;
  SIC1_ITR = 0;
  SIC2_ITR = 0;

  /* SIC1 and SIC2 IRQs are low active in MIC */
  MIC_APR &= ~(0x3);  /* low or falling edge */
  MIC_ATR &= ~(0x3);  /* level sensitive */

  /* Enable SIC1 and SIC2 IRQ in MIC */
  MIC_ER |= 0x3;
}

/******************************************************************************
 * LED handling
 *****************************************************************************/

#define PIO_BASE     (0x40028000)
#define PIO_OUTP_SET REG32(PIO_BASE + 0x04)
#define PIO_OUTP_CLR REG32(PIO_BASE + 0x08)

#define LED_D400    (1 << 2) /* #1 red */
#define LED_D401    (1 << 3) /* #2 red */
#define LED_D402    (1 << 7) /* #3 green */
#define LED_D403    (1 << 6) /* #4 green */

#define ENABLE_LED(x)  (PIO_OUTP_SET = (x))
#define DISABLE_LED(x) (PIO_OUTP_CLR = (x))

static void init_led(void) {

  /* Disable all four LEDs */
  DISABLE_LED(LED_D400 | LED_D401 | LED_D402 | LED_D403);
}

/******************************************************************************
 * HS timer handling
 *****************************************************************************/

#define TIMCLK_CTRL    REG32(0x400040BC)

#define HSTIM_BASE     (0x40038000)
#define HSTIM_INT      REG32(HSTIM_BASE + 0x00)
#define HSTIM_CTRL     REG32(HSTIM_BASE + 0x04)
#define HSTIM_COUNTER  REG32(HSTIM_BASE + 0x08)
#define HSTIM_PMATCH   REG32(HSTIM_BASE + 0x0C)
#define HSTIM_PCOUNT   REG32(HSTIM_BASE + 0x10)
#define HSTIM_MCTRL    REG32(HSTIM_BASE + 0x14)
#define HSTIM_MATCH0   REG32(HSTIM_BASE + 0x18)
#define HSTIM_MATCH1   REG32(HSTIM_BASE + 0x1C)
#define HSTIM_MATCH2   REG32(HSTIM_BASE + 0x20)
#define HSTIM_CCR      REG32(HSTIM_BASE + 0x28)
#define HSTIM_CR0      REG32(HSTIM_BASE + 0x2C)
#define HSTIM_CR1      REG32(HSTIM_BASE + 0x30)

#define PER_CLK     (13000000UL)
#define TICK_500MS  (500  * (PER_CLK/(13*1000)))

unsigned int tick_count = 0;

static void irq_hstimer(unsigned int irq_number) {

  if(irq_number != 5)
    printf("Error, something seems to be wrong with interrupt handling %i\n",
	   irq_number);

  /* Clear IRQ */
  HSTIM_INT = (0x1 << 0);

  tick_count++;

  /* IRQ should come every 1/2s, so LED3 should blink with 1s */
  if(tick_count & 0x1)
    ENABLE_LED(LED_D402);
  else
    DISABLE_LED(LED_D402);
}

struct _measure {
  unsigned int start_count;
  unsigned int end_count;
  unsigned int start_tick;
  unsigned int end_tick;
  unsigned int count;
} measure;

void start_time_measure(void) {

  if((measure.start_count) == 0 && (measure.start_tick == 0)) {
    measure.start_count = HSTIM_COUNTER;
    measure.start_tick = tick_count;
    ENABLE_LED(LED_D400);
  } else {
    printf("Error: start_time_measure(): Already in use!\n");
  }
}

void stop_time_measure(void) {

  unsigned int time;

  DISABLE_LED(LED_D400);
  measure.end_count = HSTIM_COUNTER;
  measure.end_tick = tick_count;
  measure.count++;

  if(measure.start_tick == measure.end_tick) {
    time = measure.end_count - measure.start_count;
  } else {
    time = (measure.end_tick - measure.start_tick) * TICK_500MS +
           (TICK_500MS - measure.start_count) + measure.end_count;
  }

  printf("Measure #%i: %ius\n", measure.count, time);

  measure.start_count = 0;
  measure.start_tick = 0;
}

static void init_hstimer(void) {

  /* Init High speed timer */
  TIMCLK_CTRL =   (1 << 1);   /* HSTimer clock enable */
  HSTIM_CTRL  &= ~(1 << 0);   /* Stop counting */
  HSTIM_MCTRL =   0x3;        /* Unmask MATCH0 intr flag, */
                              /* Enable reset of Timer Counter on Match 0, */
                              /* Disable the stop functionality on Match 0 */
  HSTIM_PMATCH = 13-1;        /* Set prescaler generate 1MHz == 1us */
  HSTIM_CTRL |= (1 << 1);     /* Reset the counter */
  while(HSTIM_COUNTER);
  HSTIM_CTRL &= ~(1 << 1) ;   /* release reset of the counter */
  HSTIM_MATCH0 = TICK_500MS;  /* IRQ every 500ms == 1/2s */
  HSTIM_INT =   (1 << 0);     /* Clear MATCH0 interrupt flag */

  /* Init timer interrupts */
  MIC_APR |=    (1 << 5);     /* Interrupt is generated on high level signal */
  MIC_ATR &=   ~(1 << 5);     /* Interrupt is level sensitive */
  MIC_ER |=     (1 << 5);     /* Enable millisecond timer interrupt */
  HSTIM_CTRL |= (1 << 0);     /* Enable counting */

  if(register_irq(irq_hstimer, 5) != 0) {
    printf("Warning: HSTIMER interrupt already in use...\n");
  }

  measure.start_count = 0;
  measure.start_tick = 0;
  measure.end_count = 0;
  measure.end_tick = 0;
  measure.count = 0;
}

/******************************************************************************
 * Helper functions
 *****************************************************************************/

#define ASCII_CR           0x0D         /* Carriage Return */
#define ASCII_LF           0x0A         /* Line Feed */

#ifdef USE_C_LIB

int _lseek(int file, int ptr, int dir){
    return 0;
}

int _read(int file, char *ptr, int len){
    return 0;
}

int _write(int file, char *ptr, int len){
    int todo;
  
    for (todo = 0; todo < len; todo++) {
      if( *ptr != '\n') {
          uart5_putc(*ptr++);
      } else {
	  uart5_putc( ASCII_CR );
	  uart5_putc( ASCII_LF );
	  ptr++;
      }
    }
    return len;
}

int _close(int file){
    return -1;
}

int _isatty(int file){
   return 1;
}

#include <sys/stat.h>
int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

extern char *get_sp(void);
extern void finished(void);

caddr_t _sbrk(int incr){
  extern char _end;		/* Defined by the linker */
  static char *heap_end;
  char *prev_heap_end;
 
  if (heap_end == 0) {
    heap_end = &_end;
  }
  prev_heap_end = heap_end;
  if (heap_end + incr > get_sp())
    {
      _write (1, "Heap and stack collision\n", 25);
      finished();
    }

  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

#else

/* print_string -- print a string */ 
static int print_string(char *OutString)
{
  int count;
   
  count = 0;

  for( ;(*OutString != 0); OutString++)
  {
      if( *OutString != '\n')
      {
          uart5_putc( *OutString );
	  count++;
      }
      else
      {
	  uart5_putc( ASCII_CR );
	  uart5_putc( ASCII_LF );
      }
  }
	   
  return( count );
}

#endif /* USE_C_LIB */

/* print_int -- print the value of an 32 bit integer as hex */ 
void print_int(unsigned int value)
{
  int i;
  char outchar;

  uart5_putc('0');uart5_putc('x');
   
  for(i = 7; i >= 0; i--) {
    outchar = (value >> (i << 2)) & 0xF;
    if(outchar > 9)
      uart5_putc(outchar - 10 + 'A' );
    else
      uart5_putc(outchar + '0');
  }
   
  return;
}

/* End of main.c */
