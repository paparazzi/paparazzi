/*********************************************************FileHeaderBegin******
 *
 * FILE:
 *     vfp.c
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
 *     ARM vector floating point (VFP) test code for Phytec LPC3180 board.
 *
 * NOTES:
 *
 *     Some code doing
 *
 *     volatile unsigned int count = 0;
 *   
 *     while(count < 1000000)
 *        count++;
 *
 *     compiled with no optimization takes ~53ms
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

// 33687us -m
//  6483us -mopt
//   229us X86

extern void start_time_measure(void);
extern void stop_time_measure(void);

#include <math.h>

/* Called from main() after system initialization */
void test_vfp(void) {

  start_time_measure();

  /* Put your VFP test code here */
  {
    volatile float i;
    int j;

    for(j = 0; j < 100; j++) {
    i = sin(0.1);
    i = cos(0.1);
    i = tan(0.1);
    i = sin(0.2);
    i = cos(0.2);
    i = tan(0.2);
    i = sin(0.3);
    i = cos(0.3);
    i = tan(0.3);
    i = sin(0.4);
    i = cos(0.4);
    i = tan(0.4);
    i = sin(0.5);
    i = cos(0.5);
    i = tan(0.5);
    i = sin(0.6);
    i = cos(0.6);
    i = tan(0.6);
    i = sin(0.7);
    i = cos(0.7);
    i = tan(0.7);
    i = sin(0.8);
    i = cos(0.8);
    i = tan(0.8);
    i = sin(0.9);
    i = cos(0.9);
    i = tan(0.9);
    i = sin(1.0);
    i = cos(1.0);
    i = tan(1.0);
    }
  }

  stop_time_measure();
}

/* End of vfp.c */
