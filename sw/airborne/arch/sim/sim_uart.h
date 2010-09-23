/** \file sim_uart.h
 * \brief Simulation of uart transmission on an Unix pipe
 */

#include <stdio.h>

extern FILE* pipe_stream;

#define SimUartCheckFreeSpace(_) TRUE

#define SimUartTransmit(_x) fputc(_x, pipe_stream)
#define SimUartPrintString(_s) fputs(_s, pipe_stream)
#define SimUartSendMessage() fflush(pipe_stream);
#define SimUartPrintHex16(c) _PrintHex16(SimUartTransmit, c)
