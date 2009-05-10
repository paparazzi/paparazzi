/*
 * $Id: main_logger.c 3080 2009-03-11 17:02:19Z gautier $
 *  
 * Copyright (C) 2009  Martin Mueller
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file main_logger.c
 *  \brief Logger application
 *
 *   This collects telemetry received through a serial port and writes that
 * to a (micro) SD card through the efsl library
 */

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart_hw.h"
#include "uart.h"

#include "efs.h"
#include "ls.h"
#include "mkfs.h"

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

/* BUTTON that stops logging (BUTTON = P0.7, INT1 = P0.14) */
#define STOP_KEY 14

static inline void main_init( void );
static inline void main_periodic_task( void );
int main_log(void);
void set_filename(unsigned int local, char* name);

extern int main_mass_storage(void);

DirList list;
EmbeddedFileSystem efs;
EmbeddedFile filer;	
EmbeddedFile filew;

void set_filename(unsigned int local, char* name)
{
    /* do not use sprintf or similar */
    int i;
    
    for (i=7; i>=0; i--) {
        name[i] = (local % 10) + '0';
        local /= 10;
    }
    name[8]='.';name[9]='t';name[10]='x';name[11]='t';name[12]=0;
}

int main_log(void)
{
    unsigned int count;
    unsigned char name[13];
    unsigned char inc;               

	if(efs_init(&efs, 0) != 0) {
		return(-1);
	}

    /* find an unused file number the dumb way */
    for (count = 1; count < 0xFFFFFFF; count++)
    {
        set_filename(count, name);
        if(file_fopen(&filer, &efs.myFs, name,'r')!=0) break;
        file_fclose(&filer);
    }

    if (file_fopen(&filew, &efs.myFs, name, 'w' ) != 0) 
    {
		return(-1);
    }

    /* write to SD until key is pressed */
    LED_ON(3);
    while ((IO0PIN & _BV(STOP_KEY))>>STOP_KEY)
    {
        if (Uart1ChAvailable())
        {   
			LED_TOGGLE(2);
			inc = Uart1Getch();
			file_write(&filew, 1, &inc);
        }
    }
    LED_OFF(2);
    LED_OFF(3);

    file_fclose( &filew );
    fs_umount( &efs.myFs ) ;
 
main_mass_storage();
   
	return(0);
}

int main( void ) {
  main_init();

  main_log();

  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
#ifdef USE_UART1
    Uart1Init();
#endif
  int_enable();
}

static inline void main_periodic_task( void ) {
  LED_TOGGLE(1);
}
