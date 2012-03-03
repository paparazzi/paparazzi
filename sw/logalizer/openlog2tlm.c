/*
 * $Id$
 *
 * Converter for OpenLog logfiles to TLM
 *  
 * Copyright (C) 2011 Christoph Niemann
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

/** Converts a Paparazzi Message dump, containing a timestamp,
    to Paparazzi TLM, wich can be converted to .data and .log by
    sw/logalizer/sd2log .
    The openlog Module has to be loaded for use
*/

#include <stdio.h>
#include <stdlib.h>

/* define the message id for the TIMESTAMP message (default is 129) */
#define MSG_NUMBER 129

int main(int argc, char *argv[]) {
  FILE *in,*out;

  int current_timestamp = 0;
  int timestamp_bytes[4] = {0,0,0,0};
  int temp = 0;

  if(argc != 3){
    puts("wrong number of parameters!\n"
      "usage is openlog2tlm <inuptfile> <outputfile>");
    return EXIT_FAILURE;
  }
  if((in=fopen(argv[1],"rb"))==NULL){
    puts("openlog2tlm wasn't able to open the inputfile\n");
    return EXIT_FAILURE;
  }
  if((out=fopen(argv[2],"wb"))==NULL){
    puts("openlog2tlm wasn't able to open the outputfile\n");
    return EXIT_FAILURE;
  }

  printf("converting %s to %s\n",argv[1],argv[2]);

  temp = fgetc(in);

  while(!feof(in)){
    if(temp==0x99){/// if a message starts
      int length=fgetc(in); /// determining the length of the message
      int message[length-2]; /// allocate an array that fits for the message
      int i;
      for(i = 0; i<(length-2); i++){/// read the complete message first
        message[i]=fgetc(in);
      }
      temp = fgetc(in);
      if(message[1]==MSG_NUMBER){
        current_timestamp = message[2]+(message[3]<<8)+(message[4]<<16)+(message[5]<<24);
        current_timestamp = current_timestamp*10; /// now according to 100 microsecond grid for tlm
        /// splitting the timestamp into bytes again, to use it for the messages
        timestamp_bytes[0] = current_timestamp & 0xff;
        timestamp_bytes[1] = ( current_timestamp >> 8 ) & 0xff;
        timestamp_bytes[2] = ( current_timestamp >> 16 ) & 0xff;
        timestamp_bytes[3] = ( current_timestamp >> 24 ) & 0xff;
      }
      /// start to write the message to the tlm-file
      fputc(0x99,out);/// write PPRZ_STX
      fputc(length-4,out);/// write LENGTH, recalculated for TLM
      fputc(0,out); /// write SOURCE, defaults to uart0
      fputc(timestamp_bytes[0],out); /// write TIMESTAMP_LSB
      fputc(timestamp_bytes[1],out); /// write TIMESTAMP
      fputc(timestamp_bytes[2],out); /// write TIMESTAMP
      fputc(timestamp_bytes[3],out); /// write TIMESTAMP_MSB
      int checksum = length-4+timestamp_bytes[0]+timestamp_bytes[1]+timestamp_bytes[2]+timestamp_bytes[3];
      for(i = 0; i<(length-4); i++){/// write payload
        fputc(message[i],out);
        checksum+=message[i];
      }
      fputc(checksum,out);/// write checksum, recalculated for tlm
     }
  }
  return EXIT_SUCCESS;
}