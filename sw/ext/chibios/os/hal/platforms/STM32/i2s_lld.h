/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32/i2s_lld.h
 * @brief   I2S Driver subsystem low level driver header template.
 *
 * @addtogroup I2S
 * @{
 */

#ifndef _I2S_LLD_H_
#define _I2S_LLD_H_

#if HAL_USE_I2S || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   I2S2 driver enable switch.
 * @details If set to @p TRUE the support for I2S2 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_I2S_USE_I2S2) || defined(__DOXYGEN__)
#define STM32_I2S_USE_I2S2                  FALSE
#endif

/**
 * @brief   I2S3 driver enable switch.
 * @details If set to @p TRUE the support for I2S3 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(STM32_I2S_USE_I2S3) || defined(__DOXYGEN__)
#define STM32_I2S_USE_I2S3                  FALSE
#endif

/**
 * @brief   I2S2 interrupt priority level setting.
 */
#if !defined(STM32_I2S_I2S2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2S_I2S2_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2S3 interrupt priority level setting.
 */
#if !defined(STM32_I2S_I2S3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2S_I2S3_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2S2 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_I2S_I2S2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2S_I2S2_DMA_PRIORITY         1
#endif

/**
 * @brief   I2S3 DMA priority (0..3|lowest..highest).
 */
#if !defined(STM32_I2S_I2S2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2S_I2S2_DMA_PRIORITY         1
#endif

/**
 * @brief   I2S DMA error hook.
 */
#if !defined(STM32_I2S_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_I2S_DMA_ERROR_HOOK(i2sp)      chSysHalt()
#endif

#if STM32_ADVANCED_DMA || defined(__DOXYGEN__)

/**
 * @brief   DMA stream used for I2S2 RX operations.
 * @note    This option is only available on platforms with enhanced DMA.
 */
#if !defined(STM32_I2S_I2S2_RX_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_I2S_I2S2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 0)
#endif

/**
 * @brief   DMA stream used for I2S2 TX operations.
 * @note    This option is only available on platforms with enhanced DMA.
 */
#if !defined(STM32_I2S_I2S2_TX_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_I2S_I2S2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)
#endif

/**
 * @brief   DMA stream used for I2S3 RX operations.
 * @note    This option is only available on platforms with enhanced DMA.
 */
#if !defined(STM32_I2S_I2S3_RX_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_I2S_I2S3_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 0)
#endif

/**
 * @brief   DMA stream used for I2S3 TX operations.
 * @note    This option is only available on platforms with enhanced DMA.
 */
#if !defined(STM32_I2S_I2S3_TX_DMA_STREAM) || defined(__DOXYGEN__)
#define STM32_I2S_I2S3_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 7)
#endif

#else /* !STM32_ADVANCED_DMA */

/* Fixed streams for platforms using the old DMA peripheral, the values are
   valid for both STM32F1xx and STM32L1xx.*/
#define STM32_I2S_I2S2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)
#define STM32_I2S_I2S2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 5)
#define STM32_I2S_I2S3_RX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 1)
#define STM32_I2S_I2S3_TX_DMA_STREAM        STM32_DMA_STREAM_ID(2, 2)

#endif /* !STM32_ADVANCED_DMA */
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_I2S_USE_I2S2 && !STM32_HAS_SPI2
#error "SPI2 not present in the selected device"
#endif

#if STM32_I2S_USE_I2S3 && !STM32_HAS_SPI3
#error "SPI3 not present in the selected device"
#endif

#if !STM32_I2S_USE_I2S2 && !STM32_I2S_USE_I2S3
#error "I2S driver activated but no I2S peripheral assigned"
#endif

#if STM32_I2S_USE_I2S2 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_I2S_I2S2_RX_DMA_STREAM, STM32_SPI2_RX_DMA_MSK)
#error "invalid DMA stream associated to I2S2 RX"
#endif

#if STM32_I2S_USE_I2S2 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_I2S_I2S2_TX_DMA_STREAM, STM32_SPI2_TX_DMA_MSK)
#error "invalid DMA stream associated to I2S2 TX"
#endif

#if STM32_I2S_USE_I2S3 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_I2S_I2S3_RX_DMA_STREAM, STM32_SPI3_RX_DMA_MSK)
#error "invalid DMA stream associated to I2S3 RX"
#endif

#if STM32_I2S_USE_I2S3 &&                                                   \
    !STM32_DMA_IS_VALID_ID(STM32_I2S_I2S3_TX_DMA_STREAM, STM32_SPI3_TX_DMA_MSK)
#error "invalid DMA stream associated to I2S3 TX"
#endif

#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   I2S mode type.
 */
typedef uint32_t i2smode_t;

/**
 * @brief   Type of a structure representing an I2S driver.
 */
typedef struct I2SDriver I2SDriver;

/**
 * @brief   I2S notification callback type.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] buffer    pointer to the buffer
 * @param[in] n         number of sample positions starting from @p buffer
 */
typedef void (*i2scallback_t)(I2SDriver *i2sp, void *buffer, size_t n);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   I2S mode selection.
   */
  i2smode_t                 mode;
  /**
   * @brief   Transmission buffer pointer.
   */
  const void                *tx_buffer;
  /**
   * @brief   Transmission buffer size in number of samples.
   */
  size_t                    tx_size;
  /**
   * @brief   Callback function associated to the transmission or @p NULL.
   */
  i2scallback_t             tx_cb;
  /**
   * @brief   Receive buffer pointer.
   */
  void                      *rx_buffer;
  /**
   * @brief   Receive buffer size in number of samples.
   */
  size_t                    rx_size;
  /**
   * @brief   Callback function associated to the reception or @p NULL.
   */
  i2scallback_t             rx_cb;;
  /* End of the mandatory fields.*/
  /**
   * @brief   Configuration of the I2SCFGR register.
   * @details See the STM32 reference manual, this register is used for
   *          the I2S configuration, the following bits must not be
   *          specified because handled directly by the driver:
   *          - I2SMOD
   *          - I2SE
   *          - I2SCFG
   *          .
   */
  int16_t                   i2scfgr;
  /**
   * @brief   Configuration of the I2SPR register.
   * @details See the STM32 reference manual, this register is used for
   *          the I2S clock setup.
   */
  int16_t                   i2spr;
} I2SConfig;

/**
 * @brief   Structure representing an I2S driver.
 */
struct I2SDriver {
  /**
   * @brief   Driver state.
   */
  i2sstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2SConfig           *config;
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to the SPIx registers block.
   */
  SPI_TypeDef               *spi;
  /**
   * @brief   DMA stream.
   */
  const stm32_dma_stream_t  *dma;
  /**
   * @brief   DMA mode bit mask.
   */
  uint32_t                  dmamode;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_I2S_USE_I2S2 && !defined(__DOXYGEN__)
extern I2SDriver I2SD2;
#endif

#if STM32_I2S_USE_I2S3 && !defined(__DOXYGEN__)
extern I2SDriver I2SD3;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2s_lld_init(void);
  void i2s_lld_start(I2SDriver *i2sp);
  void i2s_lld_stop(I2SDriver *i2sp);
  void i2s_lld_start_exchange(I2SDriver *i2sp);
  void i2s_lld_start_exchange_continuous(I2SDriver *i2sp);
  void i2s_lld_stop_exchange(I2SDriver *i2sp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2S */

#endif /* _I2S_LLD_H_ */

/** @} */
