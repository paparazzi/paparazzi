/*
 * Copyright (C) 2019 Alexandre Bustico, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file    hal_stm32_dma.c
 * @brief   STM32 DMA subsystem driver header.
 *
 * @author Alexandre Bustico
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#include "hal_stm32_dma.h"
#include "string.h"

/*
TODO :

° split lld and hardware independant code : hal_stm32_dma et hal_lld_stm32_dma

° port to H7,L4+ : bdma, dmav3, mdma+dmamux

° allow fifo burst when STM32_DMA_USE_ASYNC_TIMOUT is true by forcing a flush of the fifo : could be
  done disabling stream : should flush fifo and trig full code ISR. full code ISR should re-enable stream
  after a timout.

*/


#if (! STM32_DMA_ADVANCED) && STM32_DMA_USE_DOUBLE_BUFFER
#error "STM32_DMA_USE_DOUBLE_BUFFER only available on DMAv2" 
#endif

#if (STM32_DMA_USE_ASYNC_TIMOUT) && STM32_DMA_USE_DOUBLE_BUFFER
#error "STM32_DMA_USE_DOUBLE_BUFFER only not yet compatible with STM32_DMA_USE_ASYNC_TIMOUT"
#endif


/**
 * @brief   DMA ISR service routine.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void dma_lld_serve_interrupt(DMADriver *dmap, uint32_t flags);

#if STM32_DMA_ADVANCED
static inline uint32_t getFCR_FS(const DMADriver *dmap) {
  return (dmap->dmastream->stream->FCR & DMA_SxFCR_FS_Msk);
}
#endif

void dmaObjectInit(DMADriver *dmap)
{
  osalDbgCheck(dmap != NULL);

  dmap->state    = DMA_STOP;
  dmap->config   = NULL;
  dmap->mem0p    = NULL;
#if STM32_DMA_USE_WAIT == TRUE
  dmap->thread   = NULL;
#endif
#if STM32_DMA_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&dmap->mutex);
#endif
#if defined( STM32_DMA_DRIVER_EXT_INIT_HOOK)
  STM32_DMA_DRIVER_EXT_INIT_HOOK(dmap);
#endif
#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
  dmap->nbTransferError = dmap->nbDirectModeError = dmap->nbFifoError = 0U;
  dmap->lastError = 0U;
#endif
#if STM32_DMA_USE_ASYNC_TIMOUT
  chVTObjectInit(&dmap->vt);
#endif
}


/**
 * @brief   Configures and activates the DMA peripheral.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 * @param[in] config    pointer to the @p DMAConfig object.
 * @return              The operation result.
 * @retval true         dma driver is OK
 * @retval false        incoherencies has been found in config
 * @api
 */
bool dmaStart(DMADriver *dmap, const DMAConfig *cfg)
{
  osalDbgCheck((dmap != NULL) && (cfg != NULL));
  #if STM32_DMA_USE_DOUBLE_BUFFER
  osalDbgAssert((cfg->op_mode != DMA_CONTINUOUS_DOUBLE_BUFFER) || (!STM32_DMA_USE_ASYNC_TIMOUT),
                "STM32_DMA_USE_ASYNC_TIMOUT not yet implemented in DMA_CONTINUOUS_DOUBLE_BUFFER mode");

  osalDbgAssert((cfg->op_mode != DMA_CONTINUOUS_DOUBLE_BUFFER) || (cfg->next_cb != NULL),
                "DMA_CONTINUOUS_DOUBLE_BUFFER mode implies next_cb not NULL");
#endif
  osalSysLock();
  osalDbgAssert((dmap->state == DMA_STOP) || (dmap->state == DMA_READY),
                "invalid state");
  dmap->config = cfg;
  const bool statusOk = dma_lld_start(dmap, true);
  dmap->state = DMA_READY;
  #if  STM32_DMA_USE_DOUBLE_BUFFER
  dmap->next_cb_errors = 0U;
#endif  
  osalSysUnlock();
  return statusOk;
}

/**
 * @brief   Configures and activates the DMA peripheral.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 * @param[in] config    pointer to the @p DMAConfig object.
 * @return              The operation result.
 * @retval true         dma driver is OK
 * @retval false        incoherencies has been found in config
 * @api
 */
bool dmaReloadConf(DMADriver *dmap, const DMAConfig *cfg)
{
  osalDbgCheck((dmap != NULL) && (cfg != NULL));
#if STM32_DMA_USE_DOUBLE_BUFFER
  osalDbgAssert((cfg->op_mode != DMA_CONTINUOUS_DOUBLE_BUFFER) || (!STM32_DMA_USE_ASYNC_TIMOUT),
                "STM32_DMA_USE_ASYNC_TIMOUT not yet implemented in DMA_CONTINUOUS_DOUBLE_BUFFER mode");

  osalDbgAssert((cfg->op_mode != DMA_CONTINUOUS_DOUBLE_BUFFER) || (cfg->next_cb != NULL),
                "DMA_CONTINUOUS_DOUBLE_BUFFER mode implies next_cb not NULL");
#endif
  osalSysLock();
  osalDbgAssert(dmap->state == DMA_READY, "invalid state");
  dmap->config = cfg;
  const bool statusOk = dma_lld_start(dmap, false);
#if  STM32_DMA_USE_DOUBLE_BUFFER
  dmap->next_cb_errors = 0U;
#endif  
  osalSysUnlock();
  return statusOk;
}


/**
 * @brief   Deactivates the DMA peripheral.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @api
 */
void dmaStop(DMADriver *dmap)
{
  osalDbgCheck(dmap != NULL);

  osalDbgAssert((dmap->state == DMA_STOP) || (dmap->state == DMA_READY),
                "invalid state");

  dma_lld_stop(dmap);

  osalSysLock();
  dmap->config = NULL;
  dmap->state  = DMA_STOP;
  dmap->mem0p   = NULL;
  osalSysUnlock();
}


/**
 * @brief   Starts a DMA transaction.
 * @details Starts one or many asynchronous dma transaction(s) depending on continuous field
 * @post    The callbacks associated to the DMA config will be invoked
 *          on buffer fill and error events, and timeout events in case
 *          STM32_DMA_USE_ASYNC_TIMOUT == TRUE
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      dmap      pointer to the @p DMADriver object
 * @param[in,out]  periphp   pointer to a @p peripheral register address
 * @param[in,out]  mem0p    pointer to the data buffer
 * @param[in]      size     buffer size. The buffer size
 *                          must be one or an even number.
 *
 * @api
 */
bool dmaStartTransfert(DMADriver *dmap, volatile void *periphp,  void * mem0p, const size_t size)
{
  osalSysLock();
  const bool statusOk = dmaStartTransfertI(dmap, periphp, mem0p, size);
  osalSysUnlock();
  return statusOk;
}

/**
 * @brief   Starts a DMA transaction.
 * @details Starts one or many asynchronous dma transaction(s) depending on continuous field
 * @post    The callbacks associated to the DMA config will be invoked
 *          on buffer fill and error events, and timeout events in case
 *          STM32_DMA_USE_ASYNC_TIMOUT == TRUE
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      dmap      pointer to the @p DMADriver object
 * @param[in,out]  periphp   pointer to a @p peripheral register address
 * @param[in,out]  mem0p    pointer to the data buffer
 * @param[in]      size     buffer size. The buffer size
 *                          must be one or an even number.
 *
 * @iclass
 */
bool dmaStartTransfertI(DMADriver *dmap, volatile void *periphp,  void *  mem0p, const size_t size)
{
  osalDbgCheckClassI();

  #if STM32_DMA_USE_DOUBLE_BUFFER
  if (dmap->config->op_mode == DMA_CONTINUOUS_DOUBLE_BUFFER) {
    osalDbgAssert(mem0p == NULL,
		  "in double buffer mode memory pointer is dynamically completed by next_cb callback");
    mem0p = dmap->config->next_cb(dmap, size);    
  }
#endif

#if (CH_DBG_ENABLE_ASSERTS != FALSE)
  if (size != dmap->size) {
    osalDbgCheck((dmap != NULL) && (mem0p != NULL) && (periphp != NULL) &&
		 (size > 0U) && ((size == 1U) ||
				 ((dmap->config->op_mode != DMA_CONTINUOUS_HALF_BUFFER) ||
				  (((size & 1U) == 0U)))));

    const DMAConfig	    *cfg = dmap->config;
    osalDbgAssert((dmap->state == DMA_READY) ||
		  (dmap->state == DMA_COMPLETE) ||
		  (dmap->state == DMA_ERROR),
		  "not ready");
    /* if (cfg->pburst) */
    /*   osalDbgAssert((uint32_t) periphp % (cfg->pburst * cfg->psize) == 0, "peripheral address not aligned"); */
    /* else */
      osalDbgAssert((uint32_t) periphp % cfg->psize == 0, "peripheral address not aligned");

    /* if (cfg->mburst) */
    /*   osalDbgAssert((uint32_t) mem0p % (cfg->mburst * cfg->msize) == 0, "memory address not aligned"); */
    /* else */
      osalDbgAssert((uint32_t) mem0p % cfg->msize == 0, "memory address not aligned");

    /*
      In the circular mode, it is mandatory to respect the following rule in case of a burst mode
      configured for memory:
      DMA_SxNDTR = Multiple of ((Mburst beat) × (Msize)/(Psize)), where:
      – (Mburst beat) = 4, 8 or 16 (depending on the MBURST bits in the DMA_SxCR
      register)
      – ((Msize)/(Psize)) = 1, 2, 4, 1/2 or 1/4 (Msize and Psize represent the MSIZE and
      PSIZE bits in the DMA_SxCR register. They are byte dependent)
      – DMA_SxNDTR = Number of data items to transfer on the AHB peripheral port

      NDTR must also be a multiple of the Peripheral burst size multiplied by the peripheral data
      size, otherwise this could result in a bad DMA behavior.

    */
# if  STM32_DMA_ADVANCED
    if (cfg->mburst) {
      osalDbgAssert((size % (cfg->mburst * cfg->msize / cfg->psize)) == 0,
		    "mburst alignment rule not respected");
      osalDbgAssert((size % (cfg->mburst * cfg->msize)) == 0,
		    "mburst alignment rule not respected");
      osalDbgAssert((((uint32_t) mem0p) % cfg->mburst) == 0,
		    "memory address alignment rule not respected");
    }
    if (cfg->pburst) {
      osalDbgAssert((size % (cfg->pburst * cfg->psize)) == 0,
		    "pburst alignment rule not respected");
      osalDbgAssert((((uint32_t) periphp) % cfg->pburst) == 0,
		    "peripheral address alignment rule not respected");
   }
    if (cfg->periph_inc_size_4) {
      osalDbgAssert(cfg->inc_peripheral_addr,
		    "periph_inc_size_4 implies enabling inc_peripheral_addr");
      osalDbgAssert(cfg->fifo,
		    "periph_inc_size_4 implies enabling fifo");
    }

# endif //  STM32_DMA_ADVANCED
  }
#endif // CH_DBG_ENABLE_ASSERTS != FALSE
  dmap->state    = DMA_ACTIVE;

#if STM32_DMA_USE_ASYNC_TIMOUT
  dmap->currPtr = mem0p;
  if (dmap->config->timeout != TIME_INFINITE) {
    chVTSetI(&dmap->vt, dmap->config->timeout,
             &dma_lld_serve_timeout_interrupt, (void *) dmap);
  }
#endif

  return dma_lld_start_transfert(dmap, periphp, mem0p, size);
}

/**
 * @brief   copy the dma register to memory.
 * @details mainly used to preapare mdma linked list chained 
 *          transferts
 *
 * @param[in]      dmap      pointer to the @p DMADriver object
 * @param[in,out]  periphp   pointer to a @p peripheral register address
 * @param[in,out]  mem0p     pointer to the data buffer
 * @param[in]      size      buffer size. The buffer size
 *                           must be one or an even number.
 * @param[out]     registers pointer to structure representing 
                             a DMA stream set of registers
 *
 * @iclass
 */
#ifndef DMA_request_TypeDef
void  dmaGetRegisters(DMADriver *dmap, volatile void *periphp, void *mem0p,
		      const size_t size,
		      DMA_Stream_TypeDef *registers)
{
#if STM32_DMA_USE_DOUBLE_BUFFER
  if (dmap->config->op_mode == DMA_CONTINUOUS_DOUBLE_BUFFER) {
    osalDbgAssert(mem0p == NULL,
		  "in double buffer mode memory pointer is dynamically completed by next_cb callback");
    mem0p = dmap->config->next_cb(dmap, size);    
  }
#endif
  
#if (CH_DBG_ENABLE_ASSERTS != FALSE)
  osalDbgCheck((dmap != NULL) && (mem0p != NULL) && (periphp != NULL) &&
	       (size > 0U) && ((size == 1U) ||
			       ((dmap->config->op_mode != DMA_CONTINUOUS_HALF_BUFFER) ||
				(((size & 1U) == 0U)))));
  
  const DMAConfig	    *cfg = dmap->config;
  osalDbgAssert(dmap->state == DMA_READY,  "not ready");
  osalDbgAssert((uint32_t) periphp % cfg->psize == 0,
		"peripheral address not aligned");

  osalDbgAssert((uint32_t) mem0p % cfg->msize == 0,
		"memory address not aligned");

# if  STM32_DMA_ADVANCED
  if (cfg->mburst) {
    osalDbgAssert((size % (cfg->mburst * cfg->msize / cfg->psize)) == 0,
		  "mburst alignment rule not respected");
    osalDbgAssert((size % (cfg->mburst * cfg->msize)) == 0,
		  "mburst alignment rule not respected");
    osalDbgAssert((((uint32_t) mem0p) % cfg->mburst) == 0,
		  "memory address alignment rule not respected");
  }
  if (cfg->pburst) {
    osalDbgAssert((size % (cfg->pburst * cfg->psize)) == 0,
		  "pburst alignment rule not respected");
    osalDbgAssert((((uint32_t) periphp) % cfg->pburst) == 0,
		  "peripheral address alignment rule not respected");
  }
  if (cfg->periph_inc_size_4) {
    osalDbgAssert(cfg->inc_peripheral_addr,
		  "periph_inc_size_4 implies enabling inc_peripheral_addr");
    osalDbgAssert(cfg->fifo,
		  "periph_inc_size_4 implies enabling fifo");
  }
  
# endif //  STM32_DMA_ADVANCED
#endif // CH_DBG_ENABLE_ASSERTS != FALSE

  dma_lld_get_registers(dmap, periphp, mem0p, size, registers);
}
#endif

/**
 * @brief   Stops an ongoing transaction.
 * @details This function stops the currently ongoing transaction and returns
 *          the driver in the @p DMA_READY state. If there was no transaction
 *          being processed then the function does nothing.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @api
 */
void dmaStopTransfert(DMADriver *dmap)
{

  osalDbgCheck(dmap != NULL);

  osalSysLock();
  osalDbgAssert((dmap->state == DMA_READY) || (dmap->state == DMA_ACTIVE),
                "invalid state");
  if (dmap->state != DMA_READY) {
    dma_lld_stop_transfert(dmap);
    dmap->state = DMA_READY;
    _dma_reset_s(dmap);
  }
  osalSysUnlock();
}



/**
 * @brief   Stops an ongoing transaction.
 * @details This function stops the currently ongoing transaction and returns
 *          the driver in the @p DMA_READY state. If there was no transaction
 *          being processed then the function does nothing.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @iclass
 */
void dmaStopTransfertI(DMADriver *dmap)
{
  osalDbgCheckClassI();
  osalDbgCheck(dmap != NULL);
  osalDbgAssert((dmap->state == DMA_READY) ||
                (dmap->state == DMA_ACTIVE) ||
                (dmap->state == DMA_COMPLETE),
                "invalid state");


  if (dmap->state != DMA_READY) {
    dma_lld_stop_transfert(dmap);
    dmap->state = DMA_READY;
    _dma_reset_i(dmap);
  }
}

uint8_t dmaGetStreamIndex(DMADriver *dmap)
{
  for(uint8_t i = 0; i < 16; i++) {
    if (dmap->dmastream == STM32_DMA_STREAM(i))
      return i;
  }
  return 0xff;
}

#if (STM32_DMA_USE_WAIT == TRUE) || defined(__DOXYGEN__)

/**
 * @brief   Performs  a DMA transaction.
 * @details Performs one synchronous dma transaction
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      dmap      pointer to the @p DMADriver object
 * @param[in,out]  periphp   pointer to a @p peripheral register address
 * @param[in,out]  mem0p    pointer to the data buffer
 * @param[in]      size     buffer size. The buffer size
 *                          must be one or an even number.
 * @param[in]      timeout  function will exit after timeout is transaction is not done
 *                          can be TIME_INFINITE (but not TIME_IMMEDIATE)
 * @return              The operation result.
 * @retval MSG_OK       Transaction finished.
 * @retval MSG_RESET    The transaction has been stopped using
 *                      @p dmaStopTransaction() or @p dmaStopTransactionI(),
 *                      the result buffer may contain incorrect data.
 * @retval MSG_TIMEOUT  The transaction has been stopped because of hardware
 *                      error or timeout limit reach
 *
 * @api
 */
msg_t dmaTransfertTimeout(DMADriver *dmap, volatile void *periphp, void *mem0p, const size_t size,
                          sysinterval_t timeout)
{
  msg_t msg;

  osalSysLock();
  osalDbgAssert(dmap->thread == NULL, "already waiting");
  osalDbgAssert(dmap->config->op_mode == DMA_ONESHOT, "blocking API is incompatible with circular modes");
  dmaStartTransfertI(dmap, periphp, mem0p, size);
  msg = osalThreadSuspendTimeoutS(&dmap->thread, timeout);
  if (msg != MSG_OK) {
    dmaStopTransfertI(dmap);
  }
  osalSysUnlock();
  return msg;
}
#endif

#if (STM32_DMA_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the DMA peripheral.
 * @details This function tries to gain ownership to the DMA bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option
 *          @p DMA_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @api
 */
void dmaAcquireBus(DMADriver *dmap)
{

  osalDbgCheck(dmap != NULL);

  osalMutexLock(&dmap->mutex);
}

/**
 * @brief   Releases exclusive access to the DMA peripheral.
 * @pre     In order to use this function the option
 *          @p DMA_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @api
 */
void dmaReleaseBus(DMADriver *dmap)
{

  osalDbgCheck(dmap != NULL);

  osalMutexUnlock(&dmap->mutex);
}
#endif /* DMA_USE_MUTUAL_EXCLUSION == TRUE */



/*
#                 _                                  _                              _
#                | |                                | |                            | |
#                | |        ___   __      __        | |        ___  __   __   ___  | |
#                | |       / _ \  \ \ /\ / /        | |       / _ \ \ \ / /  / _ \ | |
#                | |____  | (_) |  \ V  V /         | |____  |  __/  \ V /  |  __/ | |
#                |______|  \___/    \_/\_/          |______|  \___|   \_/    \___| |_|
#                 _____            _
#                |  __ \          (_)
#                | |  | |   _ __   _   __   __   ___   _ __
#                | |  | |  | '__| | |  \ \ / /  / _ \ | '__|
#                | |__| |  | |    | |   \ V /  |  __/ | |
#                |_____/   |_|    |_|    \_/    \___| |_|
*/

/**
 * @brief   Configures and activates the DMA peripheral.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @notapi
 */
bool dma_lld_start(DMADriver *dmap, bool allocate_stream)
{
  uint32_t psize_msk, msize_msk;

  const DMAConfig *cfg = dmap->config;
  osalDbgAssert(PORT_IRQ_IS_VALID_KERNEL_PRIORITY(cfg->irq_priority),
		"illegal IRQ priority");
  switch (cfg->psize) {
    case 1 : psize_msk = STM32_DMA_CR_PSIZE_BYTE; break;
    case 2 : psize_msk = STM32_DMA_CR_PSIZE_HWORD; break;
    case 4 : psize_msk = STM32_DMA_CR_PSIZE_WORD; break;
    default: osalSysHalt("psize should be 1 or 2 or 4");
      return false;
  }
  switch (cfg->msize) {
    case 1 : msize_msk = STM32_DMA_CR_MSIZE_BYTE; break;
    case 2 : msize_msk = STM32_DMA_CR_MSIZE_HWORD; break;
    case 4 : msize_msk = STM32_DMA_CR_MSIZE_WORD; break;
    default: osalDbgAssert(false, "msize should be 1 or 2 or 4");
      return false;
  }

  uint32_t dir_msk = 0UL;
  switch (cfg->direction) {
    case DMA_DIR_P2M: dir_msk = STM32_DMA_CR_DIR_P2M; break;
    case DMA_DIR_M2P: dir_msk = STM32_DMA_CR_DIR_M2P; break;
    case DMA_DIR_M2M: dir_msk = STM32_DMA_CR_DIR_M2M; break;
    default: osalDbgAssert(false, "direction not set or incorrect");
  }

  uint32_t isr_flags = cfg->op_mode == DMA_ONESHOT ? STM32_DMA_CR_TCIE : 0UL;
  
  // in M2M mode, half buffer transfert ISR is disabled, but
  // full buffer transfert complete ISR is enabled
  if (cfg->end_cb) {
    isr_flags |= STM32_DMA_CR_TCIE;
    if ((cfg->direction != DMA_DIR_M2M) &&
	(cfg->op_mode == DMA_CONTINUOUS_HALF_BUFFER)) {
      isr_flags |= STM32_DMA_CR_HTIE;
    }
  }

  if (cfg->error_cb) {
    isr_flags |= STM32_DMA_CR_DMEIE | STM32_DMA_CR_TCIE;
  }

#if CH_KERNEL_MAJOR < 6
  dmap->dmastream =  STM32_DMA_STREAM(cfg->stream);
#endif

  // portable way (V1, V2) to retreive controler number
#if STM32_DMA_SUPPORTS_DMAMUX == 0
#if STM32_DMA_ADVANCED
  dmap->controller = 1 + (cfg->stream / STM32_DMA_STREAM_ID(2, 0));
#else
  dmap->controller = 1 + (cfg->stream / STM32_DMA_STREAM_ID(2, 1));
#endif
#endif

  dmap->dmamode = STM32_DMA_CR_PL(cfg->dma_priority) |
    dir_msk | psize_msk | msize_msk | isr_flags |
    (cfg->op_mode == DMA_CONTINUOUS_HALF_BUFFER ? STM32_DMA_CR_CIRC : 0UL) |
#if  STM32_DMA_USE_DOUBLE_BUFFER
    (cfg->op_mode == DMA_CONTINUOUS_DOUBLE_BUFFER ? STM32_DMA_CR_DBM : 0UL) |
#endif
    (cfg->inc_peripheral_addr ? STM32_DMA_CR_PINC : 0UL) |
    (cfg->inc_memory_addr ? STM32_DMA_CR_MINC : 0UL)

#if STM32_DMA_SUPPORTS_CSELR
                  | STM32_DMA_CR_CHSEL(cfg->request)
#elif STM32_DMA_ADVANCED
#if STM32_DMA_SUPPORTS_DMAMUX == 0
                  | STM32_DMA_CR_CHSEL(cfg->channel)
#endif
                  | (cfg->periph_inc_size_4 ? STM32_DMA_CR_PINCOS : 0UL) |
                  (cfg->transfert_end_ctrl_by_periph? STM32_DMA_CR_PFCTRL : 0UL)
#   endif
                  ;


#if STM32_DMA_ADVANCED
  uint32_t  pburst_msk, mburst_msk, fifo_msk; // STM32_DMA_CR_PBURST_INCRx, STM32_DMA_CR_MBURST_INCRx
  switch (cfg->pburst) {
    case 0 : pburst_msk = 0UL; break;
    case 4 : pburst_msk  = STM32_DMA_CR_PBURST_INCR4; break;
    case 8 : pburst_msk  = STM32_DMA_CR_PBURST_INCR8; break;
    case 16 : pburst_msk = STM32_DMA_CR_PBURST_INCR16; break;
    default: osalDbgAssert(false, "pburst size should be 0 or 4 or 8 or 16");
      return false;
  }
  switch (cfg->mburst) {
    case 0 : mburst_msk = 0UL; break;
    case 4 : mburst_msk  = STM32_DMA_CR_MBURST_INCR4; break;
    case 8 : mburst_msk  = STM32_DMA_CR_MBURST_INCR8; break;
    case 16 : mburst_msk = STM32_DMA_CR_MBURST_INCR16; break;
    default: osalDbgAssert(false, "mburst size should be 0 or 4 or 8 or 16");
      return false;
  }
  switch (cfg->fifo) {
    case 0 : fifo_msk = 0UL; break;
    case 1 : fifo_msk = STM32_DMA_FCR_FTH_1Q; break;
    case 2 : fifo_msk  = STM32_DMA_FCR_FTH_HALF; break;
    case 3 : fifo_msk  = STM32_DMA_FCR_FTH_3Q;  break;
    case 4 : fifo_msk =  STM32_DMA_FCR_FTH_FULL; break;
    default: osalDbgAssert(false, "fifo threshold should be 1(/4) or 2(/4) or 3(/4) or 4(/4)");
      return false;
  }


# if (CH_DBG_ENABLE_ASSERTS != FALSE)
#   if STM32_DMA_USE_ASYNC_TIMOUT
  osalDbgAssert(dmap->config->timeout != 0,
                "timeout cannot be 0 if STM32_DMA_USE_ASYNC_TIMOUT is enabled");
  osalDbgAssert(!((dmap->config->timeout != TIME_INFINITE) && (dmap->config->fifo != 0)),
                "timeout should be dynamically disabled (dmap->config->timeout = TIME_INFINITE) "
                "if STM32_DMA_USE_ASYNC_TIMOUT is enabled and fifo is enabled (fifo != 0)");

#   endif


  // lot of combination of parameters are forbiden, and some conditions must be meet
  if (!cfg->mburst !=  !cfg->pburst) {
    osalDbgAssert(false, "pburst and mburst should be enabled or disabled together");
    return false;
  }

  // from RM0090, Table 48. FIFO threshold configurations
  if (cfg->fifo && cfg->mburst) {
    const size_t fifo_level = cfg->fifo * 4U;
    osalDbgAssert(fifo_level % (cfg->mburst * cfg->msize) == 0, "threshold combination forbidden");
  }
  // if (cfg->fifo) {
  //   switch (cfg->msize) {
  //   case 1: // msize 1
  //     switch (cfg->mburst) {
  //     case 4 : // msize 1 mburst 4
	// switch (cfg->fifo) {
	// case 1: break; // msize 1 mburst 4 fifo 1/4
	// case 2: break; // msize 1 mburst 4 fifo 2/4
	// case 3: break; // msize 1 mburst 4 fifo 3/4
	// case 4: break; // msize 1 mburst 4 fifo 4/4
	// }
	// break;
  //     case 8 : // msize 1 mburst 8
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination;  // msize 1 mburst 8 fifo 1/4
	// case 2: break;			    // msize 1 mburst 8 fifo 2/4
	// case 3: goto forbiddenCombination;  // msize 1 mburst 8 fifo 3/4
	// case 4: break;			    // msize 1 mburst 8 fifo 4/4
	// }
	// break;
  //     case 16 :  // msize 1 mburst 16
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 1 mburst 16 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 1 mburst 16 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 1 mburst 16 fifo 3/4
	// case 4: break;			   // msize 1 mburst 16 fifo 4/4
	// }
	// break;
  //     }
  //     break;
  //   case 2: // msize 2
  //     switch (cfg->mburst) {
  //     case 4 : // msize 2 mburst 4
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 2 mburst 4 fifo 1/4
	// case 2: break;			   // msize 2 mburst 4 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 2 mburst 4 fifo 3/4
	// case 4: break;			   // msize 2 mburst 4 fifo 4/4
	// }
	// break;
  //     case 8 :
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 2 mburst 8 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 2 mburst 8 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 2 mburst 8 fifo 3/4
	// case 4: break;			   // msize 2 mburst 8 fifo 4/4
	// }
	// break;
  //     case 16 :
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 2 mburst 16 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 2 mburst 16 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 2 mburst 16 fifo 3/4
	// case 4: goto forbiddenCombination; // msize 2 mburst 16 fifo 4/4
	// }
  //     }
  //     break;
  //   case 4:
  //     switch (cfg->mburst) {
  //     case 4 :
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 4 mburst 4 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 4 mburst 4 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 4 mburst 4 fifo 3/4
	// case 4: break;			   // msize 4 mburst 4 fifo 4/4
	// }
	// break;
  //     case 8 :
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 4 mburst 8 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 4 mburst 8 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 4 mburst 8 fifo 3/4
	// case 4: goto forbiddenCombination; // msize 4 mburst 8 fifo 4/4
	// }
	// break;
  //     case 16 :
	// switch (cfg->fifo) {
	// case 1: goto forbiddenCombination; // msize 4 mburst 16 fifo 1/4
	// case 2: goto forbiddenCombination; // msize 4 mburst 16 fifo 2/4
	// case 3: goto forbiddenCombination; // msize 4 mburst 16 fifo 3/4
	// case 4: goto forbiddenCombination; // msize 4 mburst 16 fifo 4/4
	// }
  //     }
  //   }
  // }
# endif

  dmap->dmamode |= (pburst_msk | mburst_msk);

#  if (CH_DBG_ENABLE_ASSERTS != FALSE)


  /*
    When burst transfers are requested on the peripheral AHB port and the FIFO is used
    (DMDIS = 1 in the DMA_SxCR register), it is mandatory to respect the following rule to
    avoid permanent underrun or overrun conditions, depending on the DMA stream direction:
    If (PBURST × PSIZE) = FIFO_SIZE (4 words), FIFO_Threshold = 3/4 is forbidden
  */
  if ( ((cfg->pburst * cfg->psize) == STM32_DMA_FIFO_SIZE) && (cfg->fifo == 3)) {
    goto forbiddenCombination;
  }

  /*
    When memory-to-memory mode is used, the Circular and direct modes are not allowed.
    Only the DMA2 controller is able to perform memory-to-memory transfers.
  */
#if STM32_DMA_SUPPORTS_DMAMUX == 0
  if (cfg->direction == DMA_DIR_M2M) {
    osalDbgAssert(dmap->controller == 2, "M2M not available on DMA1");
  }
#endif

#  endif
#endif

#if STM32_DMA_ADVANCED
  if (cfg->fifo) {
    dmap->fifomode = STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FEIE | fifo_msk;
  } else {
    osalDbgAssert(cfg->psize == cfg->msize,
		  "msize == psize is mandatory when fifo is disabled");
    dmap->fifomode = 0U;
  }
#endif

  if (allocate_stream == true) {
#if CH_KERNEL_MAJOR < 6
    const bool error = dmaStreamAllocate( dmap->dmastream,
            cfg->irq_priority,
            (stm32_dmaisr_t) &dma_lld_serve_interrupt,
            (void *) dmap );
#else
    dmap->dmastream = dmaStreamAllocI(dmap->config->stream,
              cfg->irq_priority,
              (stm32_dmaisr_t) &dma_lld_serve_interrupt,
              (void *) dmap );
    const bool error = dmap->dmastream == NULL;
#endif
    if (error) {
      osalDbgAssert(false, "stream already allocated");
      return false;
    }
  }
  return true;

#if (CH_DBG_ENABLE_ASSERTS != FALSE)
#if STM32_DMA_ADVANCED
forbiddenCombination:
  chSysHalt("forbidden combination of msize, mburst, fifo, see FIFO threshold "
            "configuration in reference manuel");
  return false;
# endif
#endif
}

static inline size_t getCrossCacheBoundaryAwareSize(const void *memp,
						    const size_t dsize)
{
  // L1 cache on F7 and H7 is organised of line of 32 bytes
  // returned size is not 32 bytes aligned by a mask operation
  // because cache management does internal mask and this operation
  // would be useless

#if defined CACHE_LINE_SIZE && CACHE_LINE_SIZE != 0
  const uint32_t endp = ((uint32_t) memp % CACHE_LINE_SIZE  +
			 dsize % CACHE_LINE_SIZE );
  return endp < CACHE_LINE_SIZE  ? dsize + CACHE_LINE_SIZE  :
    dsize + CACHE_LINE_SIZE *2U;
#else
  (void) memp;
  return dsize;
#endif
}

/**
 * @brief   Copy the register of a ready stream
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 * @note : main use is preparing link list of transactions for mdma use
 * @notapi
 */
#ifndef DMA_request_TypeDef
void  dma_lld_get_registers(DMADriver *dmap, volatile void *periphp,
			    void *mem0p, const size_t size,
			    DMA_Stream_TypeDef *registers)
{
  dmaStreamSetPeripheral(dmap->dmastream, periphp);
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(dmap->dmastream, dmap->config->dmamux);
#endif

  dmaStreamSetMemory0(dmap->dmastream, mem0p);
#if  STM32_DMA_USE_DOUBLE_BUFFER
  if (dmap->config->op_mode == DMA_CONTINUOUS_DOUBLE_BUFFER) {
    dmaStreamSetMemory1(dmap->dmastream, dmap->config->next_cb(dmap, size));
  }
#endif
  dmaStreamSetTransactionSize(dmap->dmastream, size);
  dmaStreamSetMode(dmap->dmastream, dmap->dmamode);
#if STM32_DMA_ADVANCED
  dmaStreamSetFIFO(dmap->dmastream, dmap->fifomode);
#endif

  memcpy(registers, dmap->dmastream->stream, sizeof(DMA_Stream_TypeDef));
  registers->CR |= STM32_DMA_CR_EN;
}
#endif
/**
 * @brief   Starts a DMA transaction.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @notapi
 */
bool dma_lld_start_transfert(DMADriver *dmap, volatile void *periphp, void *mem0p, const size_t size)
{
#if __DCACHE_PRESENT
  if (dmap->config->activate_dcache_sync &&
      (dmap->config->direction != DMA_DIR_P2M)) {
    const size_t cacheSize = getCrossCacheBoundaryAwareSize(mem0p,
						    size * dmap->config->msize);
    cacheBufferFlush(mem0p, cacheSize);
  }
#endif
  dmap->mem0p = mem0p;
#if __DCACHE_PRESENT
  dmap->periphp = periphp;
#endif
  dmap->size = size;
  dmaStreamSetPeripheral(dmap->dmastream, periphp);
#if STM32_DMA_SUPPORTS_DMAMUX
  dmaSetRequestSource(dmap->dmastream, dmap->config->dmamux);
#endif

  dmaStreamSetMemory0(dmap->dmastream, mem0p);
#if  STM32_DMA_USE_DOUBLE_BUFFER
  if (dmap->config->op_mode == DMA_CONTINUOUS_DOUBLE_BUFFER) {
    dmaStreamSetMemory1(dmap->dmastream, dmap->config->next_cb(dmap, size));
  }
#endif
  dmaStreamSetTransactionSize(dmap->dmastream, size);
  dmaStreamSetMode(dmap->dmastream, dmap->dmamode);
#if STM32_DMA_ADVANCED
  dmaStreamSetFIFO(dmap->dmastream, dmap->fifomode);
#endif
  dmaStreamEnable(dmap->dmastream);

  return true;
}

/**
 * @brief   Stops an ongoing transaction.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @notapi
 */
void dma_lld_stop_transfert(DMADriver *dmap)
{
  dmaStreamDisable(dmap->dmastream);
}

/**
 * @brief   Deactivates the DMA peripheral.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 *
 * @notapi
 */
void dma_lld_stop(DMADriver *dmap)
{
#if CH_KERNEL_MAJOR < 6
  dmaStreamRelease(dmap->dmastream);
#else
  dmaStreamFree(dmap->dmastream);
#endif
}


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   DMA DMA ISR service routine.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void dma_lld_serve_interrupt(DMADriver *dmap, uint32_t flags)
{

  /* DMA errors handling.*/
#if CH_DBG_SYSTEM_STATE_CHECK && STM32_DMA_ADVANCED
  const uint32_t feif_msk = dmap->config->fifo != 0U ? STM32_DMA_ISR_FEIF : 0U;
#else
  const uint32_t feif_msk = 0U;
#endif
  //const uint32_t feif_msk = STM32_DMA_ISR_FEIF;
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF | feif_msk)) != 0U) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    dmaerrormask_t err =
      ( (flags & STM32_DMA_ISR_TEIF)  ? DMA_ERR_TRANSFER_ERROR : 0UL) |
      ( (flags & STM32_DMA_ISR_DMEIF) ? DMA_ERR_DIRECTMODE_ERROR : 0UL);

    if (dmap->config->fifo != 0U) {
#if STM32_DMA_ADVANCED
      dmaerrormask_t fserr = 
	( (flags & feif_msk)  ? DMA_ERR_FIFO_ERROR : 0UL) |
	( getFCR_FS(dmap) == (0b100 << DMA_SxFCR_FS_Pos) ? DMA_ERR_FIFO_EMPTY: 0UL) |
	( getFCR_FS(dmap) == (0b101 << DMA_SxFCR_FS_Pos) ? DMA_ERR_FIFO_FULL: 0UL) ;
      err |= fserr;
#endif
    }


    _dma_isr_error_code(dmap, err);
  } else {
    /* It is possible that the transaction has already be reset by the
       DMA error handler, in this case this interrupt is spurious.*/
    if (dmap->state == DMA_ACTIVE) {
#if __DCACHE_PRESENT
      if (dmap->config->activate_dcache_sync)
	  switch (dmap->config->direction) {
	  case DMA_DIR_M2P : break;
	  case DMA_DIR_P2M : if (dmap->mem0p >= (void *) 0x20000000) {
	      const size_t cacheSize =
		getCrossCacheBoundaryAwareSize(dmap->mem0p, dmap->size *
					       dmap->config->msize);
	      cacheBufferInvalidate(dmap->mem0p, cacheSize);
	    }
	    break;

	  case DMA_DIR_M2M :  if (dmap->periphp >=  (void *) 0x20000000) {
	      const size_t cacheSize =
		getCrossCacheBoundaryAwareSize((void *) dmap->periphp,
					       dmap->size * dmap->config->psize);
	      cacheBufferInvalidate(dmap->periphp, cacheSize);
	    }
	    break;
      }
#endif

      if ((flags & STM32_DMA_ISR_TCIF) != 0) {
        /* Transfer complete processing.*/
        _dma_isr_full_code(dmap);
      } else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
        /* Half transfer processing.*/
        _dma_isr_half_code(dmap);
      }
    }
  }
}

#if STM32_DMA_USE_ASYNC_TIMOUT
/**
 * @brief   DMA ISR service routine.
 *
 * @param[in] dmap      pointer to the @p DMADriver object
 */
void dma_lld_serve_timeout_interrupt(void *arg)
{
  DMADriver *dmap = (DMADriver *) arg;
  if (dmap->config->op_mode != DMA_ONESHOT) {
    chSysLockFromISR();
    chVTSetI(&dmap->vt, dmap->config->timeout,
             &dma_lld_serve_timeout_interrupt, (void *) dmap);
    chSysUnlockFromISR();
  }
  async_timout_enabled_call_end_cb(dmap, FROM_TIMOUT_CODE);
}
#endif

#if  STM32_DMA_USE_DOUBLE_BUFFER
/**
 * @brief   Common ISR code, switch memory pointer in double buffer mode
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only. This function must be called as soon as
 *          DMA has switched buffer.
 *
 * @param[in] dmap        pointer to the @p DMADriver object
 * @param[in] nextBuffer  pointer to a buffer that is set in MEM0xP or MEM1xP
 *			  the one which is not occupied beeing filled
 *                        by DMA.
 * @notapi
 */
void* dma_lld_set_next_double_buffer(DMADriver *dmap, void *nextBuffer)
{
  void *lastBuffer;

  if (dmaStreamGetCurrentTarget(dmap->dmastream)) {
    lastBuffer = (void *) dmap->dmastream->stream->M0AR;
    dmap->dmastream->stream->M0AR = (uint32_t) nextBuffer;
  } else {
    lastBuffer = (void *) dmap->dmastream->stream->M1AR;
    dmap->dmastream->stream->M1AR = (uint32_t) nextBuffer;
  }
  return lastBuffer;
}
#endif
