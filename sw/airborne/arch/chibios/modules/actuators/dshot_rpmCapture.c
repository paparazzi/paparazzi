#include "modules/actuators/dshot_rpmCapture.h"
#include <string.h>
#if DSHOT_STATISTICS
#include <stdutil.h>
#endif

#if DSHOT_SPEED == 0
#error dynamic dshot speed is not yet implemented in DSHOT BIDIR
#endif

#ifdef STM32H7XX
static const float TIM_FREQ_MHZ = (STM32_TIMCLK1 / 1e6d);
#else
// MUST FIXE : on F4 and F7, dshot bidir is not compatible with 84MHz timers
static const float TIM_FREQ_MHZ = (STM32_SYSCLK / 1e6d);
#endif
static const float bit1t_us = TIM_FREQ_MHZ * 6.67d * 4 / 5;
static const float speed_factor = DSHOT_SPEED / 150;

static const uint32_t  ERPS_BIT1_DUTY = bit1t_us / speed_factor;
static const uint32_t  TIM_PRESCALER = 1U;

/* gpt based timout is dependant of CPU speed, because
 the more time you spend in the ISR and context switches,
 the less you have to wait for telemetry frame completion */
#if defined STM32H7XX
#define SWTICH_TO_CAPTURE_BASE_TIMOUT 38U
#elif defined STM32F7XX
#define SWTICH_TO_CAPTURE_BASE_TIMOUT 32U
#else
#define SWTICH_TO_CAPTURE_BASE_TIMOUT 28U
#endif


static void startCapture(DshotRpmCapture *drcp);
static void stopCapture(DshotRpmCapture *drcp);
static void initCache(DshotRpmCapture *drcp);
static uint32_t processErpsDmaBuffer(const uint16_t *capture, size_t dmaLen);
static void buildDmaConfig(DshotRpmCapture *drcp);
static void dmaErrCb(DMADriver *dmad, dmaerrormask_t em);
static void gptCb(GPTDriver *gptd);

#if DSHOT_STATISTICS
static volatile dmaerrormask_t lastErr;
static volatile uint32_t       dmaErrs = 0;
#endif

#if !defined __GNUC__ || __GNUC__ < 13
#define nullptr NULL
#endif



static const struct  {
  uint32_t active;
  uint32_t dier;
} activeDier[4] = {
  {CH1_BOTH_EDGES,
   TIM_DIER_CC1DE | TIM_DIER_TDE},
  {CH1_BOTH_EDGES | CH2_BOTH_EDGES,
   TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_TDE},
  {CH1_BOTH_EDGES | CH2_BOTH_EDGES | CH3_BOTH_EDGES,
   TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_TDE},
  {CH1_BOTH_EDGES | CH2_BOTH_EDGES | CH3_BOTH_EDGES | CH4_BOTH_EDGES,
   TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_TDE}
};

static const TimICConfig timicCfgSkel = {
	  .timer = NULL,
	  .capture_cb = NULL,
	  .overflow_cb = NULL,
	  .mode = TIMIC_INPUT_CAPTURE,
	  .active = activeDier[DSHOT_CHANNELS-1].active,
	  .dier = activeDier[DSHOT_CHANNELS-1].dier,
	  .dcr = 0, // direct access, not using DMAR
	  .prescaler = TIM_PRESCALER,
	  .arr =  0xffffffff
};

/*
this driver need additional field in gptDriver structure (in halconf.h): 
one should add following line in the GPT section of halconf.h :
#define GPT_DRIVER_EXT_FIELDS void *user_data;
*/
#ifndef GPT_DRIVER_EXT_FIELDS
#error dshot rpmcapture driver involve defining #define GPT_DRIVER_EXT_FIELDS void *user_data; in halconf.h
#else
// redefine at the needed value to trigger error if defined with another value
#define GPT_DRIVER_EXT_FIELDS void *user_data; 
#endif

static const GPTConfig gptCfg =  {
  .frequency    = 1000U * 1000U, // 1MHz
  .callback     = &gptCb,
  .cr2 = 0,
  .dier = 0
};


/**
 * @brief   Configures and activates the DSHOT ERPS CAPTURE driver.
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @param[in] cfg     pointer to the @p DshotRpmCaptureConfig object.
 * @param[in] timer   pointer to the underlying timer (same one used to output control frame)
 * @api
 */
void dshotRpmCaptureStart(DshotRpmCapture *drcp, const DshotRpmCaptureConfig *cfg,
			  stm32_tim_t	  *timer)
{
  memset(drcp, 0, sizeof(*drcp));
  drcp->config = cfg;
  drcp->icCfg = timicCfgSkel;
  drcp->icCfg.timer = timer;
  initCache(drcp);
}

/**
 * @brief   stop the the DSHOT ERPS CAPTURE driver.
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @api
 */
void dshotRpmCaptureStop(DshotRpmCapture *drcp)
{
  stopCapture(drcp);
}

#if DSHOT_STATISTICS
/**
 * @brief   return dma error counter
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @api
 */
uint32_t dshotRpmGetDmaErr(void) {
return dmaErrs;
}
#endif


/**
 * @brief   capture the DSHOT ERPS frame(s) : one frame for each DSHOT_CHANNELS
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @note    synchronous API, when it returns, the erpm data is available
 * @api
 */
void dshotRpmCatchErps(DshotRpmCapture *drcp)
{
  startCapture(drcp);
  static systime_t ts = 0;
  if (ts == 0)
    ts = chVTGetSystemTimeX();

  memset(drcp->config->dma_capture->dma_buf, 0, sizeof(drcp->config->dma_capture->dma_buf));

  for (size_t i = 0; i < DSHOT_CHANNELS; i++) {
    dmaStartTransfert(&drcp->dmads[i], &drcp->icd.config->timer->CCR[i],
		      drcp->config->dma_capture->dma_buf[i].buf,
		      DSHOT_DMA_DATA_LEN + DSHOT_DMA_EXTRADATA_LEN);
  }

  osalSysLock();
  // dma end callback will resume the thread upon completion of ALL dma transaction
  // else, the timeout will take care of thread resume
  static const sysinterval_t timeoutUs = SWTICH_TO_CAPTURE_BASE_TIMOUT + 
					 (120U * 150U / DSHOT_SPEED);
  //palSetLine(LINE_LA_DBG_1);
  gptStartOneShotI(drcp->config->gptd, timeoutUs);
  //  palClearLine(LINE_LA_DBG_1);
  chThdSuspendS(&drcp->dmads[0].thread);
  //  palSetLine(LINE_LA_DBG_1);
  //  chSysPolledDelayX(1);
  //  palClearLine(LINE_LA_DBG_1);
  
  for (size_t i = 0; i < DSHOT_CHANNELS; i++) 
    dmaStopTransfertI(&drcp->dmads[i]);
  
  osalSysUnlock();
  stopCapture(drcp);

#if DSHOT_STATISTICS
  const rtcnt_t tstart = chSysGetRealtimeCounterX();
#endif

  for (size_t i = 0; i < DSHOT_CHANNELS; i++) {
#if __DCACHE_PRESENT
    if (drcp->config->dcache_memory_in_use == true) {
      cacheBufferInvalidate(drcp->config->dma_capture->dma_buf[i].buf,
			    DSHOT_DMA_DATA_LEN + DSHOT_DMA_EXTRADATA_LEN);
    }
#endif
    drcp->rpms[i] = processErpsDmaBuffer(drcp->config->dma_capture->dma_buf[i].buf,
					 DSHOT_DMA_DATA_LEN + DSHOT_DMA_EXTRADATA_LEN -
					 dmaGetTransactionCounter(&drcp->dmads[i]));
  }
#if DSHOT_STATISTICS
  const rtcnt_t tstop = chSysGetRealtimeCounterX();
  drcp->nbDecode += DSHOT_CHANNELS;
  drcp->accumDecodeTime += (tstop - tstart);
#endif
  
#if defined(DFREQ) && (DFREQ < 10) && (DFREQ != 0)
  DebugTrace("dma out on %s", msg == MSG_OK ? "completion" : "timeout");
#endif
}

#if DSHOT_STATISTICS
/**
 * @brief   trace content of DMA buffer for tuning
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @param[in] index   index of channel [0 .. 4]
 * @api
 */
void dshotRpmTrace(DshotRpmCapture *drcp, uint8_t index)
{
  for (size_t i = 0; i < DSHOT_CHANNELS; i++) {
    if ((index != 0xff) && (index != i))
      continue;
    uint16_t cur = drcp->config->dma_capture->dma_buf[i].buf[0];
    for (size_t j = 1; j < DSHOT_DMA_DATA_LEN; j++) {
      const uint16_t  dur = drcp->config->dma_capture->dma_buf[i].buf[j] - cur;
      cur = drcp->config->dma_capture->dma_buf[i].buf[j];
      chprintf(chp, "[%u] %u, ", i, dur);
    }
    DebugTrace("");
  }
  DebugTrace("");
}
#endif


/*
#                 _ __           _                    _                   
#                | '_ \         (_)                  | |                  
#                | |_) |  _ __   _   __   __   __ _  | |_     ___         
#                | .__/  | '__| | |  \ \ / /  / _` | | __|   / _ \        
#                | |     | |    | |   \ V /  | (_| | \ |_   |  __/        
#                |_|     |_|    |_|    \_/    \__,_|  \__|   \___|        
*/

/**
 * @brief  build dma configuration structure
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @private
 */
static void buildDmaConfig(DshotRpmCapture *drcp)
{
  static const DMAConfig skel = (DMAConfig) {
    .stream = 0,
#if STM32_DMA_SUPPORTS_DMAMUX
    .dmamux = 0,
#else
    .channel = 0,
#endif
    .inc_peripheral_addr = false,
    .inc_memory_addr = true,
    .op_mode = DMA_ONESHOT,
    .end_cb = NULL,
    .error_cb = &dmaErrCb,
    //.error_cb = NULL,
#if STM32_DMA_USE_ASYNC_TIMOUT
    .timeout = TIME_MS2I(100),
#endif
    .direction = DMA_DIR_P2M,
    .dma_priority = 1,
    .irq_priority = 7,
    .psize = 2,
    .msize = 2,
    .pburst = 0,
    .mburst = 0,
    .fifo = 4,
    .periph_inc_size_4 = false,
    .transfert_end_ctrl_by_periph = false
  };
  
  for (size_t i = 0; i < DSHOT_CHANNELS; i++) {
    drcp->dmaCfgs[i] = skel;
    drcp->dmaCfgs[i].stream = drcp->config->dma_streams[i].stream;
#if STM32_DMA_SUPPORTS_DMAMUX
    drcp->dmaCfgs[i].dmamux = drcp->config->dma_streams[i].dmamux;
#else
    drcp->dmaCfgs[i].channel = drcp->config->dma_streams[i].channel;
#endif
  }
}

/**
 * @brief  build dma and timer registers cache
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @private
 */
static void initCache(DshotRpmCapture *drcp)
{
  const DshotRpmCaptureConfig *cfg = drcp->config;
  timIcObjectInit(&drcp->icd);
  timIcStart(&drcp->icd, &drcp->icCfg);
  cfg->gptd->user_data = drcp; // user data in GPT driver

  buildDmaConfig(drcp);
  for (size_t i=0; i < DSHOT_CHANNELS; i++) {
    dmaObjectInit(&drcp->dmads[i]);
    dmaStart(&drcp->dmads[i], &drcp->dmaCfgs[i]);
  }

  timerDmaCache_cache(&drcp->cache, &drcp->dmads[0], drcp->icCfg.timer);
  dshotRpmCaptureStop(drcp);
} 

/**
 * @brief  start the DMA capture
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @private
 */
static void startCapture(DshotRpmCapture *drcp)
{
  gptStart(drcp->config->gptd, &gptCfg);
  timerDmaCache_restore(&drcp->cache, &drcp->dmads[0], drcp->icCfg.timer);
  timIcStartCapture(&drcp->icd);
}

/**
 * @brief  stop the DMA capture
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @private
 */
static void stopCapture(DshotRpmCapture *drcp)
{
   timIcStopCapture(&drcp->icd);
   timIcRccDisable(&drcp->icd);
   gptStop(drcp->config->gptd);
}


/**
 * @brief  convert DMA buffer full of pulse length into raw ERPS frame
 *
 * @param[in] drcp    pointer to the @p DshotRpmCapture object
 * @private
 */
static uint32_t processErpsDmaBuffer(const uint16_t *capture, size_t dmaLen)
{
  static const size_t frameLen = 20U;
  uint32_t erpsVal = 0;
  uint_fast8_t bit = 0x0;
  uint_fast8_t bitIndex = 0;
  uint_fast16_t prec = capture[0];
  
  for (size_t i = 1U; i < dmaLen; i++) {
    const uint_fast16_t len = capture[i] - prec;
    prec = capture[i];

    // GRC encoding garanties that there can be no more than 3 consecutives bits at the same level
    // made some test to replace division by multiplication + shift without any speed gain
    const uint_fast8_t nbConsecutives = (len + (ERPS_BIT1_DUTY / 2U)) / ERPS_BIT1_DUTY;
    if (bit) {
      switch(nbConsecutives) {
      case 1U:	erpsVal |= (0b001 << (frameLen - bitIndex)); break;
      case 2U:	erpsVal |= (0b011 << (frameLen - bitIndex - 1U)); break;
      default:  erpsVal |= (0b111 << (frameLen - bitIndex - 2U)); break;
      }
    }
    bit = ~bit; // flip bit
    bitIndex += nbConsecutives;
  }
  // there can be several high bits hidden in the trailing high level signal
  for (size_t j=bitIndex; j <= frameLen; j++) 
    erpsVal |= (1U << (frameLen - j));
  
  // alternative implementation for the trailing high level signal
  // slower on M4
  /* switch(frameLen - bitIndex) { */
  /* case 0U: erpsVal |= 0b001; break; */
  /* case 1U: erpsVal |= 0b011; break; */
  /* case 2U: erpsVal |= 0b111; break; */
  /* default: erpsVal |= 0b1111; break; */
  /* } */
  
  //  DebugTrace("bit index = %u; erpsVal = 0x%lx", bitIndex, erpsVal);
  return erpsVal;
}

/**
 * @brief  dma error ISR : 
 *
 * @param[in] drcp    pointer to the @p DMADriver
 * @param[in] em      error mask
 * @note	      does nothing if DSHOT_STATISTICS == FALSE
 * @private
 */
static void dmaErrCb(DMADriver *, dmaerrormask_t em)
{
#if DSHOT_STATISTICS
  lastErr = em;
  dmaErrs++;
#else
  (void) em;
#endif
}

/**
 * @brief  dma timeout ISR : 
 *
 * @param[in] gptd    pointer to the @p GPTDriver that trigger the timeout
 * @note	      does nothing if DSHOT_STATISTICS == FALSE
 * @private
 */
static void gptCb(GPTDriver *gptd)
{
  chSysLockFromISR();
  DshotRpmCapture *drcd = (DshotRpmCapture *) gptd->user_data;
  chThdResumeI(&drcd->dmads[0].thread, MSG_TIMEOUT);
  chSysUnlockFromISR();
}
