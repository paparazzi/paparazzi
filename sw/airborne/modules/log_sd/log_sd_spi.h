#ifndef BARO_SCP_H
#define BARO_SCP_H

#include "std.h"

#ifdef STM32
#error LPC21_only
#endif

#define LOG_SD_SPI_UNINIT       0

extern uint8_t log_sd_spi_status;
extern uint8_t log_sd_spi_run;

void log_sd_spi_init(void);
void log_sd_spi_event(void);
void log_sd_spi_periodic(void);
void log_sd_spi_dummy(void);
void log_sd_spi_start(void);
void log_sd_spi_stop(void);

#endif
