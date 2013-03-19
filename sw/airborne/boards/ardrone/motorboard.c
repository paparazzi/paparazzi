/*
    motorboard.c - AR.Drone motor driver

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

/*
power consumption @ 11V all 4 motors running
PWM   A
  0  0.2
 80  1.3
100  1.5
150  2.0
190  2.5
130  3.0
*/


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "gpio.h"
#include "motorboard.h"

int mot_fd; /* File descriptor for the port */


int motorboard_cmd(uint8_t cmd, uint8_t *reply, int replylen) {
	write(mot_fd,&cmd,1);
	return read(mot_fd,reply,replylen);
}

int motorboard_Init() {
	//open mot port
	mot_fd = open("/dev/ttyO0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (mot_fd == -1)
	{
		perror("open_port: Unable to open /dev/ttyO0 - ");
		return 201;
	} 
	fcntl(mot_fd, F_SETFL, 0); //read calls are non blocking
	fcntl(mot_fd, F_GETFL, 0);

	//set port options
	struct termios options;
	//Get the current options for the port
	tcgetattr(mot_fd, &options);
	//Set the baud rates to 115200
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
/*Control Options  (options.c_cflag)
B0 0 baud (drop DTR) 
B50 50 baud 
B75 75 baud 
B110 110 baud 
B134 134.5 baud 
B150 150 baud 
B200 200 baud 
B300 300 baud 
B600 600 baud 
B1200 1200 baud 
B1800 1800 baud 
B2400 2400 baud 
B4800 4800 baud 
B9600 9600 baud 
B19200 19200 baud 
B38400 38400 baud 
B57600 57,600 baud 
B76800 76,800 baud 
B115200 115,200 baud 
EXTA External rate clock 
EXTB External rate clock 
CSIZE Bit mask for data bits 
CS5 5 data bits 
CS6 6 data bits 
CS7 7 data bits 
CS8 8 data bits 
CSTOPB 2 stop bits (1 otherwise) 
CREAD Enable receiver 
PARENB Enable parity bit 
PARODD Use odd parity instead of even 
HUPCL Hangup (drop DTR) on last close 
CLOCAL Local line - do not change "owner" of port 
LOBLK Block job control output 
CNEW_RTSCTS 
CRTSCTS Enable hardware flow control (not supported on all platforms) 
*/
	options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
/*Input Options (options.c_iflag)
INPCK Enable parity check 
IGNPAR Ignore parity errors 
PARMRK Mark parity errors 
ISTRIP Strip parity bits 
IXON Enable software flow control (outgoing) 
IXOFF Enable software flow control (incoming) 
IXANY Allow any character to start flow again 
IGNBRK Ignore break condition 
BRKINT Send a SIGINT when a break condition is detected 
INLCR Map NL to CR 
IGNCR Ignore CR 
ICRNL Map CR to NL 
IUCLC Map uppercase to lowercase 
IMAXBEL Echo BEL on input line too long 
*/
	options.c_iflag = 0; //clear input options
/*Local Options (options.c_lflag)
ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals 
ICANON Enable canonical input (else raw) 
XCASE Map uppercase \lowercase (obsolete) 
ECHO Enable echoing of input characters 
ECHOE Echo erase character as BS-SP-BS 
ECHOK Echo NL after kill character 
ECHONL Echo NL 
NOFLSH Disable flushing of input buffers after interrupt or quit characters 
IEXTEN Enable extended functions 
ECHOCTL Echo control characters as ^char and delete as ~? 
ECHOPRT Echo erased character as character erased 
ECHOKE BS-SP-BS entire line on line kill 
FLUSHO Output being flushed 
PENDIN Retype pending input at next read or input char 
TOSTOP Send SIGTTOU for background output 
*/
	options.c_lflag=0; //clear local options
/*Output Options (options.c_oflag)
OPOST Postprocess output (not set = raw output) 
OLCUC Map lowercase to uppercase 
ONLCR Map NL to CR-NL 
OCRNL Map CR to NL 
NOCR No CR output at column 0 
ONLRET NL performs CR function 
OFILL Use fill characters for delay 
OFDEL Fill character is DEL 
NLDLY Mask for delay time needed between lines 
NL0 No delay for NLs 
NL1 Delay further output after newline for 100 milliseconds 
CRDLY Mask for delay time needed to return carriage to left column 
CR0 No delay for CRs 
CR1 Delay after CRs depending on current column position 
CR2 Delay 100 milliseconds after sending CRs 
CR3 Delay 150 milliseconds after sending CRs 
TABDLY Mask for delay time needed after TABs 
TAB0 No delay for TABs 
TAB1 Delay after TABs depending on current column position 
TAB2 Delay 100 milliseconds after sending TABs 
TAB3 Expand TAB characters to spaces 
BSDLY Mask for delay time needed after BSs 
BS0 No delay for BSs 
BS1 Delay 50 milliseconds after sending BSs 
VTDLY Mask for delay time needed after VTs 
VT0 No delay for VTs 
VT1 Delay 2 seconds after sending VTs 
FFDLY Mask for delay time needed after FFs 
FF0 No delay for FFs 
FF1 Delay 2 seconds after sending FFs 
*/
	options.c_oflag &= ~OPOST; //clear output options (raw output)
	//Set the new options for the port
	tcsetattr(mot_fd, TCSANOW, &options);
	
	//reset IRQ flipflop - on error 106 read 1, this code resets 106 to 0
	gpio_set(106,-1);
	gpio_set(107,0);
	gpio_set(107,1);

	//all select lines inactive
	gpio_set(68,1);
	gpio_set(69,1);
	gpio_set(70,1);
	gpio_set(71,1);

	//configure motors
	int retval=0;
	uint8_t reply[256];
	for(int m=0;m<4;m++) {
		gpio_set(68+m,-1);
		motorboard_cmd(0xe0,reply,2);
		if(reply[0]!=0xe0 || reply[1]!=0x00)
		{
			printf("motor%d cmd=0x%02x reply=0x%02x\n",m+1,(int)reply[0],(int)reply[1]);
			retval=1;
		}
		motorboard_cmd(m+1,reply,1);
		gpio_set(68+m,1);
	}

	//all select lines active
	gpio_set(68,-1);
	gpio_set(69,-1);
	gpio_set(70,-1);
	gpio_set(71,-1);

	//start multicast
	motorboard_cmd(0xa0,reply,1);
	motorboard_cmd(0xa0,reply,1);
	motorboard_cmd(0xa0,reply,1);
	motorboard_cmd(0xa0,reply,1);
	motorboard_cmd(0xa0,reply,1);

	//reset IRQ flipflop - on error 106 read 1, this code resets 106 to 0
	gpio_set(106,-1);
	gpio_set(107,0);
	gpio_set(107,1);

	//all leds green
//	motorboard_SetLeds(MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN);
	
	return retval;
}

//write motor speed command
//cmd = 001aaaaa aaaabbbb bbbbbccc ccccccdd ddddddd0 
void motorboard_SetPWM(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
	uint8_t cmd[5];
	cmd[0] = 0x20 | ((pwm0&0x1ff)>>4);
	cmd[1] = ((pwm0&0x1ff)<<4) | ((pwm1&0x1ff)>>5);
	cmd[2] = ((pwm1&0x1ff)<<3) | ((pwm2&0x1ff)>>6);
	cmd[3] = ((pwm2&0x1ff)<<2) | ((pwm3&0x1ff)>>7);
	cmd[4] = ((pwm3&0x1ff)<<1);
	write(mot_fd, cmd, 5);
}

//write led command
//cmd = 011grgrg rgrxxxxx
void motorboard_SetLeds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3)
{
	uint8_t cmd[2];
	cmd[0]=0x60 | ((led0&3)<<3)  | ((led1&3)<<1)   | ((led2&3)>>1);
	cmd[1]=((led2&3)<<7)  | ((led3&3)<<5);
	write(mot_fd, cmd, 2);
}  

void motorboard_Close()
{
	close(mot_fd);
}
