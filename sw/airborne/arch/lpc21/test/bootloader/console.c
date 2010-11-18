/*
	Simple console input/output, over serial port #0

	Partially copied from Jim Lynch's tutorial
*/

#include "console.h"

#define PINSEL0		*(volatile unsigned int *)0xE002C000

#define U0THR		*(volatile unsigned int *)0xE000C000
#define U0RBR		*(volatile unsigned int *)0xE000C000
#define U0DLL		*(volatile unsigned int *)0xE000C000
#define U0DLM		*(volatile unsigned int *)0xE000C004
#define U0FCR		*(volatile unsigned int *)0xE000C008
#define U0LCR		*(volatile unsigned int *)0xE000C00C
#define U0LSR		*(volatile unsigned int *)0xE000C014


/* Initialize Serial Interface       */
void ConsoleInit(int iDivider)
{
	PINSEL0 = (PINSEL0 & ~0x0000000F) | 0x00000005;	/* Enable RxD0 and TxD0              */
	U0LCR = 0x83;                          			/* 8 bits, no Parity, 1 Stop bit     */
	U0DLL = iDivider & 0xFF;						/* set divider / baud rate */
	U0DLM = iDivider >> 8;
	U0LCR = 0x03;                          			/* DLAB = 0                          */

	// enable FIFO
	U0FCR = 1;
}


/* Write character to Serial Port    */
int putchar(int ch)
{
	if (ch == '\n') {
		while (!(U0LSR & 0x20));
		U0THR = '\r';
	}
	while (!(U0LSR & 0x20));
	U0THR = ch;

	return ch;
}


int getchar (void)  {                    /* Read character from Serial Port   */

  while (!(U0LSR & 0x01));

  return (U0RBR);
}


int puts(char *s)
{
	while (*s) {
		putchar(*s++);
	}
	putchar('\n');
	return 1;
}


