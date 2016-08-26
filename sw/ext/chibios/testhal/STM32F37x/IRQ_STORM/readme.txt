*****************************************************************************
** ChibiOS/RT HAL - IRQ_STORM stress test demo for STM32F37x.              **
*****************************************************************************

** TARGET **

The demo runs on an STMicroelectronics STM32373C-EVAL board.

** The Demo **

The application demonstrates the use of the STM32F37x GPT, PAL and Serial
drivers in order to implement a system stress demo.

** Board Setup **

- Connect a terminal emulator to serial port connector (38400-N-8-1).

** Build Procedure **

The demo has been tested using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
