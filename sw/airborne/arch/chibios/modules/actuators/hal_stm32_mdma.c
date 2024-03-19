#include "hal_stm32_mdma.h"
#include <string.h>

/*
 
  
 */

static void mdma_lld_serve_interrupt(MDMADriver *mdmap, uint32_t flags);

static inline void mdmaChannelSetCBRUR(MDMADriver *mdmap, uint32_t cbrur);
static inline void mdmaChannelSetCLAR(MDMADriver *mdmap, uint32_t clar);
static inline void mdmaChannelSetCMAR(MDMADriver *mdmap, uint32_t cmar);
static inline void mdmaChannelSetCMDR(MDMADriver *mdmap, uint32_t cmdr);
static inline void mdmaChannelSelectBuses(MDMADriver *mdmap, uint32_t sourceDestBuses);

#ifdef MDMA_STATISTICS
static volatile uint32_t isr_call_nb;

void resetIsrCallNb(void) {isr_call_nb = 0;}
uint32_t getIsrCallNb(void) {return isr_call_nb;}
#endif




void mdmaObjectInit(MDMADriver *mdmap)
{
  osalDbgCheck(mdmap != NULL);

  mdmap->state    = MDMA_STOP;
  mdmap->config   = NULL;
  mdmap->first_trigger_src = MDMA_TRIGGER_SOFTWARE_DEFERRED;
#if STM32_MDMA_USE_WAIT == TRUE
  mdmap->thread   = NULL;
#endif
#if STM32_MDMA_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&mdmap->mutex);
#endif
#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
  mdmap->nbTransferError = 0U;
  mdmap->lastError = 0U;
#endif
}


/**
 * @brief   Configures and activates the MDMA peripheral.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 * @param[in] config    pointer to the @p MDMAConfig object.
 * @return              The operation result.
 * @retval true         mdma driver is OK
 * @retval false        incoherencies has been found in config
 * @api
 */
bool mdmaStart(MDMADriver *mdmap, const MDMAConfig *cfg)
{
  osalDbgCheck((mdmap != NULL) && (cfg != NULL));
  osalDbgCheck(cfg->link_array != NULL); 
  osalDbgAssert((mdmap->state == MDMA_STOP) || (mdmap->state == MDMA_READY),
                "invalid state");
  osalDbgAssert((((uint32_t) cfg->link_array) & 0b111) == 0U,
		"link address must be double word aligned");
  osalDbgAssert((cfg->link_array_size > 0U), "invalid linkArraySize");
  osalSysLock();
  mdmap->config = cfg;
  const bool statusOk = mdma_lld_start(mdmap);
  mdmap->state = statusOk ? MDMA_READY : MDMA_ERROR;
  
  osalSysUnlock();
  return statusOk;
}

/**
 * @brief   Deactivates the MDMA peripheral.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @api
 */
void mdmaStop(MDMADriver *mdmap)
{
  osalDbgCheck(mdmap != NULL);

  osalDbgAssert((mdmap->state == MDMA_STOP) || (mdmap->state == MDMA_READY),
                "invalid state");

  mdma_lld_stop(mdmap);

  osalSysLock();
  mdmap->config = NULL;
  mdmap->state  = MDMA_STOP;
  osalSysUnlock();
}





/**
 * @brief   Starts a MDMA transaction.
 * @details Starts one or many asynchronous mdma transaction(s) depending on configuration
 * @post    The callbacks associated to the MDMA config will be invoked
 *          on buffer fill and error events, and timeout events in case
 *          STM32_MDMA_USE_ASYNC_TIMOUT == TRUE
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      mdmap         pointer to the @p MDMADriver object
 * @param[in,out]  source        pointer to the source address
 * @param[in,out]  dest          pointer to the destination adress
 * @param[in]      size          buffer size. 
 * @param[in]      user_data     user_data passed to callback. 
 *
 * @api
 */
bool mdmaStartTransfert(MDMADriver *mdmap, void *user_data)
{
  osalSysLock();
  const bool statusOk = mdmaStartTransfertI(mdmap, user_data);
  osalSysUnlock();
  return statusOk;
}

/**
 * @brief   Starts a MDMA transaction.
 * @details Starts one or many asynchronous mdma transaction(s) depending on configuration
 * @post    The callbacks associated to the MDMA config will be invoked
 *          on buffer fill and error events, and timeout events in case
 *          STM32_MDMA_USE_ASYNC_TIMOUT == TRUE
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      mdmap         pointer to the @p MDMADriver object
 * @param[in,out]  source        pointer to the source address
 * @param[in,out]  dest          pointer to the destination adress
 * @param[in]      size          buffer size. 
 * @param[in]      user_data     user_data passed to callback. 
 *
 * @api
 */
bool  mdmaStartTransfertI(MDMADriver *mdmap, void *user_data)
{
  osalDbgCheckClassI();
  
  osalDbgCheck(mdmap != NULL);

  mdmap->state    = MDMA_ACTIVE;
  mdmap->user_data = user_data;
  return mdma_lld_start_transfert(mdmap);
}


/**
 * @brief   Stops an ongoing transaction.
 * @details This function stops the currently ongoing transaction and returns
 *          the driver in the @p MDMA_READY state. If there was no transaction
 *          being processed then the function does nothing.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @api
 */
void mdmaStopTransfert(MDMADriver *mdmap)
{

  osalDbgCheck(mdmap != NULL);

  osalSysLock();
  osalDbgAssert((mdmap->state == MDMA_READY) || (mdmap->state == MDMA_ACTIVE),
                "invalid state");
  if (mdmap->state != MDMA_READY) {
    mdma_lld_stop_transfert(mdmap);
    mdmap->state = MDMA_READY;
    _mdma_reset_s(mdmap);
  }
  osalSysUnlock();
}

/**
 * @brief   Stops an ongoing transaction.
 * @details This function stops the currently ongoing transaction and returns
 *          the driver in the @p MDMA_READY state. If there was no transaction
 *          being processed then the function does nothing.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @iclass
 */
void mdmaStopTransfertI(MDMADriver *mdmap)
{
  osalDbgCheckClassI();
  osalDbgCheck(mdmap != NULL);
  osalDbgAssert((mdmap->state == MDMA_READY) ||
                (mdmap->state == MDMA_ACTIVE) ||
                (mdmap->state == MDMA_COMPLETE),
                "invalid state");


  if (mdmap->state != MDMA_READY) {
    mdma_lld_stop_transfert(mdmap);
    mdmap->state = MDMA_READY;
    _mdma_reset_i(mdmap);
  }
}

#if (STM32_MDMA_USE_WAIT == TRUE) || defined(__DOXYGEN__)

/**
 * @brief   Performs  a MDMA transaction.
 * @details Performs one synchronous mdma transaction
 * @note    The datas are sequentially written into the buffer
 *          with no gaps.
 *
 * @param[in]      mdmap      pointer to the @p MDMADriver object
 * @param[in,out]  source   pointer to a @p peripheral register address
 * @param[in,out]  dest    pointer to the data buffer
 * @param[in]      size     buffer size.
 *                          
 * @param[in]      user_data can be usefull for callback
 *                          
 * @param[in]      timeout  function will exit after timeout is transaction is not done
 *                          can be TIME_INFINITE (but not TIME_IMMEDIATE)
 * @return              The operation result.
 * @retval MSG_OK       Transaction finished.
 * @retval MSG_RESET    The transaction has been stopped using
 *                      @p mdmaStopTransaction() or @p mdmaStopTransactionI(),
 *                      the result buffer may contain incorrect data.
 * @retval MSG_TIMEOUT  The transaction has been stopped because of hardware
 *                      error or timeout limit reach
 *
 * @api
 */
 msg_t mdmaTransfertTimeout(MDMADriver *mdmap,
			     void *user_data,  sysinterval_t timeout)
{
  msg_t msg;

  osalSysLock();
  osalDbgAssert(mdmap->thread == NULL, "already waiting");
  mdmaStartTransfertI(mdmap, user_data);
  msg = osalThreadSuspendTimeoutS(&mdmap->thread, timeout);
  if (msg != MSG_OK) {
    mdmaStopTransfertI(mdmap);
  }
  osalSysUnlock();
  return msg;
}
#endif

#if (STM32_MDMA_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the MDMA peripheral.
 * @details This function tries to gain ownership to the MDMA bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option
 *          @p MDMA_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @api
 */
void mdmaAcquireBus(MDMADriver *mdmap) {

  osalDbgCheck(mdmap != NULL);

  osalMutexLock(&mdmap->mutex);
}

/**
 * @brief   Releases exclusive access to the MDMA peripheral.
 * @pre     In order to use this function the option
 *          @p MDMA_USE_MUTUAL_EXCLUSION must be enabled.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @api
 */
void mdmaReleaseBus(MDMADriver *mdmap) {

  osalDbgCheck(mdmap != NULL);

  osalMutexUnlock(&mdmap->mutex);
}
#endif /* MDMA_USE_MUTUAL_EXCLUSION == TRUE */

void mdmaAddLinkNode(MDMADriver *mdmap,
		     const MDMANodeConfig *nodeCfg,
		     const mdmatriggersource_t trigger_src,
		     const void *source, void *dest, const size_t block_len)
{
  osalDbgCheck((mdmap != NULL) && (nodeCfg != NULL) &&
	       (source != NULL) && (dest != NULL));

  osalDbgAssert((mdmap->state == MDMA_STOP) || (mdmap->state == MDMA_READY),
                "invalid state");
  osalDbgAssert(mdmap->next_link_array_index < mdmap->config->link_array_size,
		"MDMA link array full");

  osalDbgAssert((nodeCfg->buffer_len > 0U) && (nodeCfg->buffer_len <= 128U),
		"invalid buffer_len");
  osalDbgAssert((nodeCfg->block_repeat > 0U) && (nodeCfg->block_repeat <= 4096U),
		"invalid block_repeat");
  
  const MDMAConfig          *cfg = mdmap->config;
#if (CH_DBG_ENABLE_ASSERTS != FALSE)
  const size_t size = block_len;
  const size_t ssize = 1U << nodeCfg->swidth;
  const size_t dsize = 1U << nodeCfg->dwidth;
  const size_t sburst = 1U << nodeCfg->sburst;
  const size_t dburst = 1U << nodeCfg->dburst;
  const size_t dincos = nodeCfg->dest_incr > 0 ? nodeCfg->dest_incr : -nodeCfg->dest_incr;
  const size_t sincos = nodeCfg->source_incr > 0 ? nodeCfg->source_incr : -nodeCfg->source_incr;
  osalDbgAssert((mdmap->state == MDMA_READY) ||
		(mdmap->state == MDMA_COMPLETE) ||
		(mdmap->state == MDMA_ERROR),
		"not ready");
  
  osalDbgAssert((uint32_t) source % ssize == 0, "source address not aligned");
  osalDbgAssert(size % ssize == 0, "size must me a multiple of source data size");
  osalDbgAssert(size % dsize == 0, "size must me a multiple of destination data size");
  osalDbgAssert(nodeCfg->buffer_len % ssize == 0, "buffer_len must me a multiple of source data size");
  osalDbgAssert(nodeCfg->buffer_len % dsize == 0, "buffer_len must me a multiple of destination data size");
  osalDbgAssert(sburst <= nodeCfg->buffer_len, "source burst must be less than buffer_len");
  osalDbgAssert(dburst <= nodeCfg->buffer_len, "destination burst must be less than buffer_len");
  osalDbgAssert(__builtin_popcount(sincos) <= 1, "source incremment must be a power of 2");
  osalDbgAssert(__builtin_popcount(dincos) <= 1, "destination incremment must be a power of 2");
  if (nodeCfg->bus_selection & MDMA_DESTBUS_TCM) {
    if ((dincos == 8) ||
	(dincos == 0) ||
	(dincos != dsize)) {
      osalDbgAssert(dburst == 1U, "several conditions implies destination "
		    "burst must be single transfert");
    }
  } else { // destination bus is AXI
    if (dincos == 0) {
      osalDbgAssert(dburst <= 16, "several conditions implies destination burst must not exceed 16");
    }
  }
  if (nodeCfg->bus_selection & MDMA_SOURCEBUS_TCM) {
    if ((sincos == 8) ||
	(sincos == 0) ||
	(sincos != ssize)) {
      osalDbgAssert(sburst == 1U, "several conditions implies source "
		    "burst must be single transfert");
    }
  } else { // source bus is AXI
    if (sincos == 0) {
      osalDbgAssert(sburst <= 16, "several conditions implies source burst must not exceed 16");
    }
  }
  
  if (dincos != 0) {
    osalDbgAssert(dincos >= dsize, "destination increment must be greater or equal to destination size");
    if (nodeCfg->bus_selection & MDMA_DESTBUS_TCM) {
      if (dburst != 1) {
	osalDbgAssert(((uint32_t) dest % dincos) == 0, "when dest is AHB and burst is enabled, "
		      "destination address must be aligned with destination increment");
      }
    }
  }
  
  if (sincos != 0) {
    osalDbgAssert(sincos >= ssize, "source increment must be greater or equal to source size");
    if (nodeCfg->bus_selection & MDMA_SOURCEBUS_TCM) {
      if (sburst != 1) {
	osalDbgAssert(((uint32_t) source % sincos) == 0, "when source is AHB and burst is enabled, "
		      "source address must be aligned with source increment");
      }
    }
  }
  
  if (nodeCfg->bus_selection & MDMA_SOURCEBUS_TCM) {
    osalDbgAssert(ssize <= 4, "when source is TCM/AHB source, source width should be inferior or equal to 4");
    osalDbgAssert(sincos != 0, "when source is TCM/AHB source, fixed source address is forbidden");
  }
  if (nodeCfg->bus_selection & MDMA_DESTBUS_TCM) {
    osalDbgAssert(dsize <= 4, "when destination is TCM/AHB, destination width should be inferior or equal to 4"); 
    osalDbgAssert(dincos != 0, "when destination is TCM/AHB destination, fixed destination address is forbidden");
  }
#endif
  
  mdma_lld_set_node_registers(mdmap, trigger_src, nodeCfg);
  mdma_lld_get_link_block(mdmap, nodeCfg, trigger_src, source, dest,
			  block_len,
			  &cfg->link_array[mdmap->next_link_array_index]);
  
  if (mdmap->next_link_array_index == 0) {
    cfg->link_array[0].clar = (uint32_t) NULL;
    mdmap->first_trigger_src = trigger_src;
  } else {
    cfg->link_array[mdmap->next_link_array_index - 1U].clar =
      (uint32_t) &cfg->link_array[mdmap->next_link_array_index];
  }
  mdmap->next_link_array_index++;
}


void mdmaAddLinkNodeMask(MDMADriver *mdmap,
			 const MDMANodeConfig *cfg,
			 const mdmatriggersource_t trigger_src,
			 const void *source, void *dest, const size_t block_len,
			 void *mask_address_register,
			 const uint32_t mask_data_register
			 )
{
  MDMANodeConfig ncfg = *cfg;
  ncfg.mask_data_register = mask_data_register;
  ncfg.mask_address_register = mask_address_register;
  mdmaAddLinkNode(mdmap, &ncfg, trigger_src, source, dest, block_len);
}


void mdmaLinkLoop(MDMADriver *mdmap, const size_t index)
{
  osalDbgCheck(mdmap != NULL);
  const MDMAConfig	    *cfg = mdmap->config;

  osalDbgCheck(cfg->link_array != NULL);
  osalDbgAssert((mdmap->state == MDMA_STOP) || (mdmap->state == MDMA_READY),
                "invalid state");
  osalDbgAssert(index <cfg->link_array_size,
		"MDMA loop index out of bounds");
  cfg->link_array[mdmap->next_link_array_index - 1U].clar =
    (uint32_t) &cfg->link_array[index];
}
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

static inline size_t getCrossCacheBoundaryAwareSize(const void *memp,
						    const size_t dsize)
{
  // L1 cache on F7 and H7 is organised of line of 32 bytes
  // returned size is not 32 bytes aligned by a mask operation
  // because cache management does internal mask and this operation
  // would be useless

  const uint32_t endp = ((uint32_t) memp % CACHE_LINE_SIZE  +
			 dsize % CACHE_LINE_SIZE );
  return endp < CACHE_LINE_SIZE  ? dsize + CACHE_LINE_SIZE  :
    dsize + CACHE_LINE_SIZE *2U;
}


/**
 * @brief   Configures MDMA peripheral without activating.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 * @param[in] mdmap      pointer to the @p MDMANodeConfig object
 *
 * @notapi
 */
void mdma_lld_set_common_registers(MDMADriver *mdmap)
{
  const MDMAConfig          *cfg = mdmap->config;
  mdmap->cache.ccr = STM32_MDMA_CCR_PL(cfg->mdma_priority) |
    STM32_MDMA_CCR_CTCIE |
    ((cfg->error_cb != NULL) ? STM32_MDMA_CCR_TEIE : 0U) |
    ((cfg->buffer_transfert_cb != NULL) ? STM32_MDMA_CCR_TCIE : 0U) |
    ((cfg->trigger_auto & MDMA_TRIGGER_AUTO_BUFFER) ? STM32_MDMA_CCR_TCIE : 0U) |
    ((cfg->block_transfert_cb != NULL) ? STM32_MDMA_CCR_BTIE : 0U) |
    ((cfg->trigger_auto & MDMA_TRIGGER_AUTO_ONE_BLOCK) ? STM32_MDMA_CCR_BTIE : 0U) |
    ((cfg->block_transfert_repeat_cb != NULL) ? STM32_MDMA_CCR_BRTIE : 0U) |
    ((cfg->trigger_auto & MDMA_TRIGGER_AUTO_ALL_BLOCKS) ? STM32_MDMA_CCR_BRTIE : 0U) |
    cfg->endianness_ex;
}

/**
 * @brief   Configures MDMA peripheral without activating.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 * @param[in] mdmap      pointer to the @p MDMANodeConfig object
 *
 * @notapi
 */
void mdma_lld_set_node_registers(MDMADriver *mdmap,
				 const mdmatriggersource_t trigger_src,
				 const MDMANodeConfig *nodeCfg)
{
  uint32_t sinc, sincval, dinc, dincval;
  
  if (nodeCfg->source_incr > 0) {
    sinc = STM32_MDMA_CTCR_SINC_INC;
    sincval = __builtin_ffs(nodeCfg->source_incr) - 1U;
  } else if (nodeCfg->source_incr < 0) {
    sinc = STM32_MDMA_CTCR_SINC_DEC;
    sincval = __builtin_ffs(-nodeCfg->source_incr) - 1U;
  } else {
    sinc = STM32_MDMA_CTCR_SINC_FIXED;
    sincval = 0;
  }
  if (nodeCfg->dest_incr > 0) {
    dinc = STM32_MDMA_CTCR_DINC_INC;
    dincval = __builtin_ffs(nodeCfg->dest_incr) - 1U;
  } else if (nodeCfg->dest_incr < 0) {
    dinc = STM32_MDMA_CTCR_DINC_DEC;
    dincval = __builtin_ffs(-nodeCfg->dest_incr) - 1U;
  } else {
    dinc = STM32_MDMA_CTCR_DINC_FIXED;
    dincval = 0;
  }
  
  mdmap->cache.ctcr = nodeCfg->trigger_mode |
    STM32_MDMA_CTCR_TLEN(nodeCfg->buffer_len - 1U) |
    STM32_MDMA_CTCR_SBURST(nodeCfg->sburst) |
    STM32_MDMA_CTCR_DBURST(nodeCfg->dburst) |
    STM32_MDMA_CTCR_SINCOS(sincval) |
    STM32_MDMA_CTCR_DINCOS(dincval) |
    STM32_MDMA_CTCR_SSIZE(nodeCfg->swidth) |
    STM32_MDMA_CTCR_DSIZE(nodeCfg->dwidth) |
    sinc |
    dinc |
    ((trigger_src >= MDMA_TRIGGER_SOFTWARE_IMMEDIATE) ?
     STM32_MDMA_CTCR_SWRM : 0) |
    nodeCfg->ctcr;
  
  uint32_t src_update=0, dest_update=0;
  if (nodeCfg->block_source_incr < 0) {
    mdmap->cache.opt = MDMA_CBNDTR_BRSUM;
    src_update = -nodeCfg->block_source_incr;
  } else {
    src_update = nodeCfg->block_source_incr;
  }
  
  if (nodeCfg->block_dest_incr < 0) {
    mdmap->cache.opt |= MDMA_CBNDTR_BRDUM;
    dest_update = -nodeCfg->block_dest_incr;
  } else {
    dest_update = nodeCfg->block_dest_incr;
  }
  mdmap->cache.cbrur = src_update | (dest_update << 16U);
  mdmap->cache.brc =  nodeCfg->block_repeat - 1U;
}


/**
 * @brief   Configures and activates the MDMA peripheral.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @notapi
 */
bool mdma_lld_start(MDMADriver *mdmap)
{
  const MDMAConfig *cfg = mdmap->config;
  mdma_lld_set_common_registers(mdmap);
  mdmap->mdma = mdmaChannelAllocI(cfg->channel,
				  (stm32_mdmaisr_t)mdma_lld_serve_interrupt,
				  (void *)mdmap);
  
  osalDbgAssert(mdmap->mdma != NULL, "unable to allocate MDMA channel");
  return mdmap->mdma != NULL;
}


/**
 * @brief   Deactivates the MDMA peripheral.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @notapi
 */
void mdma_lld_stop(MDMADriver *mdmap)
{
  /* Releasing the DMA.*/
  mdmaChannelFreeI(mdmap->mdma);
  mdmap->mdma = NULL;
}

/**
 * @brief   Starts a MDMA transaction.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @notapi
 */
bool  mdma_lld_start_transfert(MDMADriver *mdmap)
{
#ifdef MDMA_STATISTICS
  resetIsrCallNb();
#endif
  memcpy((void *) &mdmap->mdma->channel->CTCR, mdmap->config->link_array, 
	 sizeof(mdmalinkblock_t));
  mdmap->mdma->channel->CCR =  mdmap->cache.ccr;
  mdmaChannelEnableX(mdmap->mdma);

  if (mdmap->first_trigger_src == MDMA_TRIGGER_SOFTWARE_IMMEDIATE) {
    mdma_software_request(mdmap);
  }
  return true;
}


void  mdma_lld_get_link_block(MDMADriver *mdmap, const MDMANodeConfig *cfg,
			      const mdmatriggersource_t trigger_src,
			      const void *source, void *dest,
			      const size_t block_len,
			      mdmalinkblock_t *link_block)
{
  memset(mdmap->mdma->channel, 0, sizeof(*mdmap->mdma->channel));
  mdmaChannelSetSourceX(mdmap->mdma, source);
  mdmaChannelSetDestinationX(mdmap->mdma, dest);
  mdmaChannelSetTransactionSizeX(mdmap->mdma, block_len,
				 mdmap->cache.brc, mdmap->cache.opt);
  mdmaChannelSetModeX(mdmap->mdma, mdmap->cache.ctcr, mdmap->cache.ccr);
  if (trigger_src < MDMA_TRIGGER_SOFTWARE_IMMEDIATE)
    mdmaChannelSetTrigModeX(mdmap->mdma, trigger_src);
  mdmaChannelSelectBuses(mdmap, cfg->bus_selection);
  mdmaChannelSetCBRUR(mdmap, mdmap->cache.cbrur);
  mdmaChannelSetCMDR(mdmap, cfg->mask_data_register);
  mdmaChannelSetCMAR(mdmap, (uint32_t) cfg->mask_address_register);
  memcpy(link_block, (void *)&mdmap->mdma->channel->CTCR,
	 sizeof(mdmalinkblock_t));
}

bool mdma_software_request(MDMADriver *mdmap)
{
  if (mdmap->state != MDMA_ACTIVE) {
    // driver not ready
    return false;
  }
    
  MDMA_Channel_TypeDef  * const chn = mdmap->mdma->channel;
  if ((chn->CTCR & MDMA_CTCR_SWRM) == 0) {
    // software request not enabled
    return false;
  }
  if ((chn->CISR &  MDMA_CISR_CRQA) != 0U) {
    // busy
    return false;
  }
  if ((chn->CCR &  STM32_MDMA_CCR_EN) == 0U) {
    // channel not enabled
    return false;
  }
  chn->CCR |= MDMA_CCR_SWRQ;
  return true;
}
/**
 * @brief   Stops an ongoing transaction.
 *
 * @param[in] mdmap      pointer to the @p MDMADriver object
 *
 * @notapi
 */
void mdma_lld_stop_transfert(MDMADriver *mdmap)
{
  mdmaChannelDisableX(mdmap->mdma);
}


/**
 * @brief   Shared service routine.
 *
 * @param[in] mdmap     pointer to the @p MDMADriver object
 * @param[in] flags     content of the CISR register
 */
static void mdma_lld_serve_interrupt(MDMADriver *mdmap, uint32_t flags) {
  /* DMA errors handling.*/
#ifdef MDMA_STATISTICS
  isr_call_nb++;
#endif
  if ((flags & STM32_MDMA_CISR_TEIF) != 0) {
    
    if(mdmap->config->error_cb != NULL) {
      // TODO : different kind of errors ?
      _mdma_isr_error_code(mdmap, flags >> STM32_MDMA_FLAGS_CESR_Pos);
    }
  }
  
  if ((flags & STM32_MDMA_CISR_CTCIF) != 0) {
    /* Transfer complete processing.*/
    _mdma_isr_transaction_complete(mdmap);
  }
  
  if ((flags & STM32_MDMA_CISR_TCIF) != 0) {
    /*Buffer complete processing.*/
    if (mdmap->config->buffer_transfert_cb != NULL)
      mdmap->config->buffer_transfert_cb(mdmap);
    if (mdmap->config->trigger_auto & MDMA_TRIGGER_AUTO_BUFFER)
       mdmap->mdma->channel->CCR |= MDMA_CCR_SWRQ;
   }
  
  if ((flags & STM32_MDMA_CISR_BTIF) != 0) {
    /* Block complete processing.*/
    if (mdmap->config->block_transfert_cb != NULL)
    mdmap->config->block_transfert_cb(mdmap);
    if (mdmap->config->trigger_auto & MDMA_TRIGGER_AUTO_ONE_BLOCK)
       mdmap->mdma->channel->CCR |= MDMA_CCR_SWRQ;
   }
  
  if ((flags & STM32_MDMA_CISR_BRTIF) != 0) {
    /* Block Repeat complete processing.*/
    if (mdmap->config->block_transfert_repeat_cb != NULL)
      mdmap->config->block_transfert_repeat_cb(mdmap);
    if (mdmap->config->trigger_auto & MDMA_TRIGGER_AUTO_ALL_BLOCKS) {
      mdmap->mdma->channel->CCR |= MDMA_CCR_SWRQ;
    }
  }
}



static inline void mdmaChannelSetCBRUR(MDMADriver *mdmap, uint32_t cbrur)
{
  mdmap->mdma->channel->CBRUR =  cbrur;
}

static inline void mdmaChannelSetCLAR(MDMADriver *mdmap, uint32_t clar)
{
  mdmap->mdma->channel->CLAR =  clar;
}

static inline void mdmaChannelSetCMAR(MDMADriver *mdmap, uint32_t cmar)
{
  mdmap->mdma->channel->CMAR =  cmar;
}

static inline void mdmaChannelSetCMDR(MDMADriver *mdmap, uint32_t cmdr)
{
  mdmap->mdma->channel->CMDR =  cmdr;
}

static inline void mdmaChannelSelectBuses(MDMADriver *mdmap, uint32_t sourceDestBuses)
{
  mdmap->mdma->channel->CTBR |=  sourceDestBuses;
}
