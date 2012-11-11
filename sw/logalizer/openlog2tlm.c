/*
 * Converter for OpenLog logfiles to TLM
 *
 * Copyright (C) 2011-2012 Christoph Niemann
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

  int current_timestamp = 0;/// complete integer for the timestamp
  int timestamp_bytes[4] = {0,0,0,0};/// bytes for the timestamp to write
  int temp = 0;/// buffer variable for the char read
  int read = 0;/// another buffer for the next char for an error check
  int length = 0;/// length of the message currently read
  int nbmsg = 0;/// counter for the messages
  int nberr = 0;/// counter for the number of errors
  char err = 0;/// error flag
  char nomsg = 0; /// if there was nom message
  if(argc != 3){
    puts("wrong number of parameters!\n"
      "usage is openlog2tlm <inuptfile> <outputfile>");
    return EXIT_FAILURE;
  }
  if((in=fopen(argv[1],"rb"))==NULL){
    puts("openlog2tlm wasn't able to open the inputfile\n");
    return EXIT_FAILURE;
  }
  fseek(in,0L,SEEK_END);/// place the descriptor at the end of the file
  int inlength = ftell(in);/// get the length of the file
  fseek(in,0L,SEEK_SET);/// place the descriptor to the head of the file
  if((out=fopen(argv[2],"w+"))==NULL){
    puts("openlog2tlm wasn't able to open the outputfile\n");
    return EXIT_FAILURE;
  }
  printf("now converting %s to %s\n",argv[1],argv[2]);

  temp = fgetc(in);///read the first char
  if (temp!=0x99) {/// if the first char isn't a STX, the message was broken
	  while(temp!=0x99){///try to find the first STX
		  temp = fgetc(in);
		  if(temp==EOF) {
			  nomsg=1;///there is no message at all, set the flag to get out of here
			  break;
		  }
		 }
  }
  fseek(in,-1L,SEEK_CUR);///put the filedescriptor back to read 0x99 again


  while(!feof(in)){/// do this from the first STX (descriptor for STX is here) until the end of the file
    if(temp==0x99){/// if a message starts
      nbmsg ++;/// increment the message counter
      length=fgetc(in); /// determining the length of the message
      int message[length-2]; /// allocate an array that fits for the message
      int i;
      for(i = 0; i<(length-2); i++){/// read the complete message first
        message[i]=fgetc(in);
      }
      read = fgetc(in);///read the next char
      if (read!=0x99) {/// if the next char isn't a STX, the message was broken
    	  nberr ++;/// increment the error counter
    	  while(fgetc(in)!=0x99){///try to find the next STX
    		  if (inlength <= ftell(in)) break;///don't shift the filedescriptor and break in case the end is reached
    	  }
    	  err = 1;///set the error flag, so this package won't be written
    	  if (inlength <= ftell(in)) break;///break if the end is reached
      }
      fseek(in,-1L,SEEK_CUR);///put the filedescriptor back to read 0x99 again
      temp = fgetc(in);//read the next char
      if (err!=1){///don't write damaged packages and don't do something else with it
          if(message[1]==MSG_NUMBER){/// if the message was a timestamp, update the timestamp to write
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
      } else {
    	  err = 0; //reset the error variable and do nothing else
      }

     }
     if (inlength < ftell(in)+2) break;///end if the file is at the end
  }
  ///Drop the last message, since it is supspected to be corrupt, due to the power-down.
  ///In case it won't be overwritten, sd2log will crash
  ///put the file-descriptor to the end of the file

  fseek(out, 0L, SEEK_END);
  int end = ftell(out);//remember the offset
  ///rewind to the last STX, to find the beginning of the last package and place the filedescriptor there
  int temp2 = fgetc(out);
  while(temp2!=0x99 && nomsg==0) {/// skip the rewind in case there was no message, otherwise search for the last STX
	  fseek(out, -2L, SEEK_CUR);
	  temp2 = fgetc(out);
  }
  fseek(out,-1L,SEEK_CUR);///overwrite the STX as well

  while(ftell(out)!=end) {///overwrite the corrupt data with zeroes
	  fputc(0x0,out);
  }
  ///Check, wether there was an error during the conversion and make suggestions :-)
  if (length == 0){
	  printf("\nThere have been no messages at all. Perhaps you misconfigured your Openlog, so it uses the wrong baudrate, or you aren't using the transparent telemetry, or this even wasn't a Paparazzi-log. For debugging you can open the logfile using a hexeditor and check there is a more or less periodic occurance of the Paparazzi STX (0x99)\n Also check your wiring and the configfile of the openlog. It should be named CONFIG.TXT and contain 57600,26,3,0 if your baudrate is 57600.\n\n ");
	  return EXIT_FAILURE;
  } else if (current_timestamp == 0) {
	  printf("There have been messages but openlog2tlm didn't find the TIMESTAMP-Message. Check the messages-tool if there is a message called \"TIMESTAMP\". Make sure the Openlog-Module was loaded and check the messages.xml, the Timestamp was really %i\n", MSG_NUMBER);
	  return EXIT_FAILURE;
  } else {
	  printf("The conversion from the logfile from the SD to TLM seems to be successful, %i have been converted, %i packages have been broken\n",nbmsg,nberr);
  }
  return EXIT_SUCCESS;
}
