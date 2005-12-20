#ifndef TRACES_H
#define TRACES_H

#include "types.h"
#include "uart.h"

void uart0_print_hex(const uint8_t c);
void uart0_print_hex_16(const uint16_t c);
void uart0_print_hex_32(const uint32_t c);
void uart0_print_dec_u32(const uint32_t c);
void uart0_print_dec_32(const int32_t c);

#define PRINT_ADC() {				\
  uart0Puts("ADC0 ");				\
  uart0_print_hex_16(adc0_val[0]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc0_val[1]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc0_val[2]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc0_val[3]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc0_val[4]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc0_val[5]);		\
  uart0Puts("\r\n");				\
  uart0Puts("ADC1 ");				\
  uart0_print_hex_16(adc1_val[0]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc1_val[1]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc1_val[2]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc1_val[3]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc1_val[4]);		\
  uart0Puts(", ");				\
  uart0_print_hex_16(adc1_val[5]);		\
  uart0Puts("\r\n");				\
  uart0Puts("\r\n");				\
}


#define PRINT_PPM() {				\
  uart0Puts("PPM ");				\
  uart0_print_hex_32(ppm_pulses[0]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[1]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[2]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[3]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[4]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[5]);		\
  uart0Puts(", ");				\
  uart0_print_hex_32(ppm_pulses[6]);		\
  uart0Puts("\r\n");				\
}

#define PRINT_RADIO_CONTROL() {			\
  uart0Puts("RC ");				\
  uart0_print_dec_32(rc_values[0]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[1]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[2]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[3]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[4]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[5]);		\
  uart0Puts(", ");				\
  uart0_print_dec_32(rc_values[6]);		\
  uart0Puts("\r\n");				\
}

#define PRINT_GPS() {				\
 uart0Puts("GPS ");				\
 uart0_print_dec_32(gps_itow);			\
 uart0Puts("\r\n");				\
}

#define MODEM_PRINT_GPS() {			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
    modem_put_one_byte('G');			\
    modem_put_one_byte('P');			\
    modem_put_one_byte('S');			\
}


#endif /* TRACES_H */
