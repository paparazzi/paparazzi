#ifndef CONFIG_NAZE32_REV5_H
#define CONFIG_NAZE32_REV5_H

#include "boards/naze32_common.h"

/* Naze32 rev5 has a 12MHz external clock and 72MHz internal */
#define EXT_CLK 12000000
#define AHB_CLK 72000000

/* 16Mbit flash on spi2 (Naze32 full) */
#define SPI_SELECT_SLAVE1_PORT GPIOB
#define SPI_SELECT_SLAVE1_PIN GPIO12

#endif /* CONFIG_NAZE32_REV5_H */
