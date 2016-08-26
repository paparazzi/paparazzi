*****************************************************************************
** ChibiOS/RT HAL - UART driver demo for STM32L1xx.                        **
*****************************************************************************

** TARGET **

The demo runs on an STMicroelectronics STM32L-Discovery board.

** The Demo **

The application demonstrates the use of the STM32L1xx UART driver.

** Board Setup **

- Remove the LCD module.
- Connect an RS232 transceiver to pins PA9(TX) and PA10(RX).
- Connect a terminal emulator to the transceiver (38400-N-8-1).

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
