/**
 * @file    hal_stm32_mdma.h
 * @brief   STM32 MDMA subsystem driver header.
 *
 */

#pragma once

#include <ch.h>
#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define MDMA_STATISTICS 1


#ifdef MDMA_STATISTICS
  uint32_t getIsrCallNb(void);
#endif

#define MDMA_END_ADDR_OF(v) ((void *) ((uint8_t *) (&(v)) + sizeof(typeof(v)) - 4U))
#define MDMA_START_ADDR_OF(v) ((void *) &(v))

  
  /**
   * @brief   Enables synchronous APIs.
   * @note    Disabling this option saves both code and data space.
   */
#if !defined(STM32_MDMA_USE_WAIT) || defined(__DOXYGEN__)
#define STM32_MDMA_USE_WAIT                TRUE
#endif

  


  /**
   * @brief Enables the @p mdmaAcquireBus() and @p mdmaReleaseBus() APIs.
   * @note Disabling this option saves both code and data space.
   */
#if !defined(STM32_MDMA_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define STM32_MDMA_USE_MUTUAL_EXCLUSION    FALSE
#endif



  /**
   * @brief   Driver state machine possible states.
   */
  typedef enum {
    MDMA_UNINIT = 0,                           /**< Not initialized.          */
    MDMA_STOP = 1,                             /**< Stopped.                  */
    MDMA_READY = 2,                            /**< Ready.                    */
    MDMA_ACTIVE = 3,                           /**< Transfering.              */
    MDMA_COMPLETE = 4,                         /**< Transfert complete.       */
    MDMA_ERROR = 5,                            /**< Transfert error.          */
    MDMA_SOFTREQ_ERROR = 6		       /**< software request error.   */
  } mdmastate_t;

  typedef enum {
    MDMA_BURST_1 = 0,
    MDMA_BURST_2  ,
    MDMA_BURST_4  ,
    MDMA_BURST_8  ,
    MDMA_BURST_16 ,
    MDMA_BURST_32 ,
    MDMA_BURST_64 ,
    MDMA_BURST_128
  } mdmaburst_t;

  typedef enum {
    MDMA_WIDTH_1 = 0,
    MDMA_WIDTH_2  ,
    MDMA_WIDTH_4  ,
    MDMA_WIDTH_8  
  } mdmadwidth_t;

  typedef enum {
    MDMA_SOURCEBUS_AXI = 0,
    MDMA_DESTBUS_AXI = 0,
    MDMA_SOURCEBUS_TCM = STM32_MDMA_CTBR_TSEL_SBUS,
    MDMA_DESTBUS_TCM = STM32_MDMA_CTBR_TSEL_DBUS
  } mdmabusselect_t;
 
  /**
   * @brief   Possible MDMA failure causes.
   * @note    Error codes are architecture dependent and should not relied
   *          upon.
   */
  typedef enum {
    MDMA_ERR_TRANSFER_ADDR_MASK	    = MDMA_CESR_TEA,
    MDMA_ERR_DIRECTION		    = MDMA_CESR_TED,
    MDMA_ERR_TRANSFERT_LINK_DATA    = MDMA_CESR_TELD,
    MDMA_ERR_TRANSFERT_MASK	    = MDMA_CESR_TEMD,
    MDMA_ERR_ADDRESS_SIZE_MISMATCH  = MDMA_CESR_ASE,          
    MDMA_ERR_BLOCK_SIZE		    = MDMA_CESR_BSE
  } mdmaerrormask_t;


  /**
   * @brief   MDMA transfert memory mode
   */
  typedef enum {
    MDMA_BUFFER     = STM32_MDMA_CTCR_TRGM_BUFFER,      /**<trigger one buffer transfert   */
    MDMA_ONE_BLOCK  = STM32_MDMA_CTCR_TRGM_BLOCK,       /**<trigger one block  transfert    */
    MDMA_ALL_BLOCKS = STM32_MDMA_CTCR_TRGM_REP_BLOCK,   /**<trigger all blocks transferts */
    MDMA_WHOLE      = STM32_MDMA_CTCR_TRGM_WHOLE	/**<trigger the whole transfert (linked blocks) */
  } mdmatriggermode_t;

  /**
   * @brief   MDMA trigger source
   */
  typedef enum {
    MDMA_TRIGGER_DMA1_STREAM0 = 0,
    MDMA_TRIGGER_DMA1_STREAM1 = 1,
    MDMA_TRIGGER_DMA1_STREAM2 = 2,
    MDMA_TRIGGER_DMA1_STREAM3 = 3,
    MDMA_TRIGGER_DMA1_STREAM4 = 4,
    MDMA_TRIGGER_DMA1_STREAM5 = 5,
    MDMA_TRIGGER_DMA1_STREAM6 = 7,
    MDMA_TRIGGER_DMA1_STREAM7 = 7,
    MDMA_TRIGGER_DMA2_STREAM0 = 8,
    MDMA_TRIGGER_DMA2_STREAM1 = 9,
    MDMA_TRIGGER_DMA2_STREAM2 = 10,
    MDMA_TRIGGER_DMA2_STREAM3 = 11,
    MDMA_TRIGGER_DMA2_STREAM4 = 12,
    MDMA_TRIGGER_DMA2_STREAM5 = 13,
    MDMA_TRIGGER_DMA2_STREAM6 = 14,
    MDMA_TRIGGER_DMA2_STREAM7 = 15,
    MDMA_TRIGGER_LTDC = 16,
    MDMA_TRIGGER_JPEG_INPUT_FIFO_THR = 17,
    MDMA_TRIGGER_JPEG_INPUT_FIFO_NOT_FULL = 18,
    MDMA_TRIGGER_JPEG_OUTPUT_FIFO_THR = 19,
    MDMA_TRIGGER_JPEG_OUTPUT_FIFO_NOT_EMPTY = 20,
    MDMA_TRIGGER_JPEG_END_OF_CONV = 21,
    MDMA_TRIGGER_QUADSPI_FIFO_THR = 22,
    MDMA_TRIGGER_QUADSPI_XFER_COMPLETE = 23,
    MDMA_TRIGGER_DMA2D_CLUT_XFER_COMPLETE = 24,
    MDMA_TRIGGER_DMA2D_XFER_COMPLETE = 25,
    MDMA_TRIGGER_DMA2D_XFER_WATERMARK = 26,
    MDMA_TRIGGER_SDMMC1_END_OF_DATA = 29,
    MDMA_TRIGGER_SOFTWARE_IMMEDIATE = 64,
    MDMA_TRIGGER_SOFTWARE_DEFERRED = 65,
  } mdmatriggersource_t;

  typedef enum {
    MDMA_TRIGGER_AUTO_NONE = 0,
    MDMA_TRIGGER_AUTO_BUFFER = 1<<0, 
    MDMA_TRIGGER_AUTO_ONE_BLOCK = 1<<1, 
    MDMA_TRIGGER_AUTO_ALL_BLOCKS = 1<<2
  }  mdmatriggerauto_t;

  typedef enum {
    MDMA_NO_EXCHANGE = 0,
    MDMA_WORD_EXCHANGE = STM32_MDMA_CCR_WEX, 
    MDMA_HALFWORD_EXCHANGE = STM32_MDMA_CCR_HEX, 
    MDMA_BYTE_EXCHANGE = STM32_MDMA_CCR_BEX
  }  mdmaendianessexchange_t;

  /**
   * @brief   Type of a structure representing an MDMA driver.
   */
  typedef struct MDMADriver MDMADriver;

  /**
   * @brief   MDMA notification callback type.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object triggering the
   *                       callback
   */
  typedef void (*mdmacallback_t)(MDMADriver *mdmap);

  /**
   * @brief   MDMA error callback type.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object triggering the
   *                      callback
   * @param[in] err       MDMA error code
   */
  typedef void (*mdmaerrorcallback_t)(MDMADriver *mdmap, mdmaerrormask_t err);

  /**
   * @brief   MDMA link mode block type
   *
   * @note    After each transaction, the following registers are copied from
   *          a mdmalinkblock_t structure whose address is given is the 
   *           link.address field
   * 
   */
  typedef struct {
    uint32_t ctcr;
    uint32_t cbndtr;
    uint32_t csar; 
    uint32_t cdar ;
    uint32_t cbrur; 
    uint32_t clar; 
    uint32_t ctbr; 
    uint32_t reserved0;
    uint32_t cmar;
    uint32_t cmdr;
  } mdmalinkblock_t;
  
  /*===========================================================================*/
  /* Driver macros.                                                            */
  /*===========================================================================*/
#if (STM32_MDMA_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @name    Low level driver helper macros
   * @{
   */

  /**
   * @brief   Resumes a thread waiting for a mdma transfert completion.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object
   *
   * @notapi
   */
#define _mdma_reset_i(mdmap)				\
  osalThreadResumeI(&(mdmap)->thread, MSG_RESET)

  /**
   * @brief   Resumes a thread waiting for a mdma transfert completion.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object
   *
   * @notapi
   */
#define _mdma_reset_s(mdmap)				\
  osalThreadResumeS(&(mdmap)->thread, MSG_RESET)

  /**
   * @brief   Wakes up the waiting thread.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object
   *
   * @notapi
   */
#define _mdma_wakeup_isr(mdmap) {			\
    osalSysLockFromISR();				\
    osalThreadResumeI(&(mdmap)->thread, MSG_OK);	\
    osalSysUnlockFromISR();				\
  }

  /**
   * @brief   Wakes up the waiting thread with a timeout message.
   *
   * @param[in] mdmap      pointer to the @p MDMADriver object
   *
   * @notapi
   */
#define _mdma_timeout_isr(mdmap) {			\
    osalSysLockFromISR();				\
    osalThreadResumeI(&(mdmap)->thread, MSG_TIMEOUT);	\
    osalSysUnlockFromISR();				\
  }
#else /* !STM32_MDMA_USE_WAIT */
#define _mdma_reset_i(mdmap)
#define _mdma_reset_s(mdmap)
#define _mdma_wakeup_isr(mdmap)
#define _mdma_timeout_isr(mdmap)
#endif /* !STM32_MDMA_USE_WAIT */

  /**
   * @brief   Common ISR code, end of transaction callback
   * @details This code handles the portable part of the ISR code:
   *          - Callback invocation.
   *          .
   * @note    This macro is meant to be used in the low level drivers
   *          implementation only.
   *
   * @param[in] adcp      pointer to the @p ADCDriver object
   *
   * @notapi
   */
  static inline void _mdma_isr_transaction_complete(MDMADriver *mdmap);



  /**
   * @brief   Common ISR code, error event.
   * @details This code handles the portable part of the ISR code:
   *          - Callback invocation.
   *          - Waiting thread timeout signaling, if any.
   *          - Driver state transitions.
   *          .
   * @note    This macro is meant to be used in the low level drivers
   *          implementation only.
   *
   * @param[in] adcp      pointer to the @p ADCDriver object
   * @param[in] err       platform dependent error code
   *
   * @notapi
   */
  static inline void _mdma_isr_error_code(MDMADriver *mdmap, mdmaerrormask_t err);
  bool mdma_software_request(MDMADriver *mdmap);




  typedef struct {
    mdmalinkblock_t *  link_array;
    uint16_t	       link_array_size;
    uint8_t channel:5; // channel 0 .. 15 ; 16 for ANY
    mdmatriggerauto_t	trigger_auto;
        /**
     * @brief   Callback function associated to the stream 
     *          for transaction completion or @p NULL.
     */
    mdmacallback_t        end_cb;

    /**
     * @brief   Callback function associated to the stream 
     *          for block transfert completion or @p NULL.
     */
    mdmacallback_t        buffer_transfert_cb;

    /**
     * @brief   Callback function associated to the stream 
     *          for block transfert completion or @p NULL.
     */
    mdmacallback_t        block_transfert_cb;

  
    /**
     * @brief   Callback function associated to the stream 
     *          for block transfert repeat completion or @p NULL.
     */
    mdmacallback_t        block_transfert_repeat_cb;

  
    /**
     * @brief   Error callback or @p NULL.
     */
    mdmaerrorcallback_t    error_cb;


    /**
     * @brief   MDMA priority (0 .. 3)
     */
    uint8_t		mdma_priority:2;

    /**
     * @brief   activate endianness swap during transaction
     */
    mdmaendianessexchange_t    endianness_ex;
    
  } MDMAConfig;


  /**
   * @brief   MDMA stream configuration structure.
   * @details This implementation-dependent structure describes a MDMA
   *          operation.
   * @note    The use of this configuration structure requires knowledge of
   *          STM32 MDMA registers interface, please refer to the STM32
   *          reference manual for details.
   */
  typedef struct  {
   /**
     * @brief   Enable and give increment (positive) or decrement (negative)
     *          of source address after each transfert
     */
    int8_t              source_incr:5;


    /**
     * @brief   Enable and give increment (positive) or decrement (negative)
     *          of memory address after each transfert
     */
    int8_t              dest_incr:5;

    /**
     * @brief   MDMA memory buffer transfer len (up to 128 bytes)
     */
    uint8_t		buffer_len:7;

    //    uint32_t	        block_len:17; 
    int32_t		block_source_incr:17;
    int32_t		block_dest_incr:17;
    uint16_t	        block_repeat:13; //  1 -> 4096
    /**
     * @brief   single, reperated or linked list
     */
    mdmatriggermode_t	trigger_mode;


  

    /**
     * @brief   MDMA peripheral data granurality in bytes (1,2,4,8)
     */
    mdmadwidth_t		swidth:2; 

    /**
     * @brief   MDMA memory data granurality in bytes (1,2,4,8)
     */
    mdmadwidth_t		dwidth:2;

     /**
     * @brief   MDMA source memory burst (up to 128 bytes)
     */
    mdmaburst_t		sburst:3; // 1->128 power of 2

    /**
     * @brief   MDMA destination memory burst (up to 128 bytes)
     */
    mdmaburst_t		dburst:3; // 1->128 power of 2

    /**
     * @brief   MDMA source and destination bus selection
     * note     The Cortex®-M7 CPU uses the 64-bit AXIM bus to access all memories (excluding ITCM,
                and DTCM) and AHB3, AHB4, APB3 and APB4 peripherals (excluding AHB1, APB1 and
                APB2 peripherals).
     */
    mdmabusselect_t bus_selection;

    /**
     * @brief   MDMA block repeat count (0 - 4095)
     */

    uint32_t	mask_data_register;
    void*	mask_address_register;

    
    /** 
     * @brief   to specify more flags that are in CTCR register
     * @note	this value is ored with calculated value
     */
    uint32_t	ctcr;
  }  MDMANodeConfig;


  /**
   * @brief   Structure representing a MDMA driver.
   */
  struct MDMADriver {
    /**
     * @brief   MDMA stream associated with peripheral or memory
     */
    const stm32_mdma_channel_t  *mdma;

    /**
     * @brief Current configuration data.
     */
    const MDMAConfig	    *config;
    
    size_t		next_link_array_index;
#if STM32_MDMA_USE_WAIT || defined(__DOXYGEN__)
    /**
     * @brief Waiting thread.
     */
    thread_reference_t        thread;
#endif
#if STM32_MDMA_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
    /**
     * @brief Mutex protecting the peripheral.
     */
    mutex_t                   mutex;
#endif /* STM32_MDMA_USE_MUTUAL_EXCLUSION */

    /**
     * @brief	hold MDMA  register for the stream
     */
    struct mdmacache_t {
      uint32_t brc;
      uint32_t ccr;
      uint32_t ctcr;
      uint32_t opt;
      uint32_t cbrur;
    } cache;


#if CH_DBG_SYSTEM_STATE_CHECK
    volatile size_t		     nbTransferError;
    volatile mdmaerrormask_t	     lastError;
#endif
    /**
     * @brief	Driver state
     */
    volatile mdmastate_t		     state;
    mdmatriggersource_t			     first_trigger_src;
    void *user_data;
  };



  void  mdmaObjectInit(MDMADriver *mdmap);
  bool  mdmaStart(MDMADriver *mdmap, const MDMAConfig *cfg);
  void  mdmaStop(MDMADriver *mdmap);

#if STM32_MDMA_USE_WAIT == TRUE
  msg_t mdmaTransfertTimeout(MDMADriver *mdmap, void *user_data, sysinterval_t timeout);
  // helper
  static inline msg_t mdmaTransfert(MDMADriver *mdmap, void *user_data)
  {
    return mdmaTransfertTimeout(mdmap, user_data, TIME_INFINITE);
  }
#endif
#if STM32_MDMA_USE_MUTUAL_EXCLUSION == TRUE
  void mdmaAcquireBus(MDMADriver *mdmap);
  void mdmaReleaseBus(MDMADriver *mdmap);
#endif
  bool  mdmaStartTransfert(MDMADriver *mdmap, void *user_data);
  void  mdmaStopTransfert(MDMADriver *mdmap);
  bool  mdmaStartTransfertI(MDMADriver *mdmap, void *user_data);
  void  mdmaStopTransfertI(MDMADriver *mdmap);
  static inline bool mdmaSoftRequest(MDMADriver *mdmap) {
    return mdma_software_request(mdmap);
  }


  static  inline mdmastate_t mdmaGetState(MDMADriver *mdmap) {return mdmap->state;}

  void mdmaAddLinkNode(MDMADriver *mdmap,
		       const MDMANodeConfig *cfg,
		       const mdmatriggersource_t trigger_src,
		       const void *source, void *dest, const size_t block_len);
 void mdmaAddLinkNodeMask(MDMADriver *mdmap,
			  const MDMANodeConfig *cfg,
			  const mdmatriggersource_t trigger_src,
			  const void *source, void *dest, const size_t block_len,
			  void *mask_data_register,
			  const uint32_t mask_address_register);
  void mdmaLinkLoop(MDMADriver *mdmap, const size_t index);
  // low level driver
  void mdma_lld_set_common_registers(MDMADriver *mdmap);
  void mdma_lld_set_node_registers(MDMADriver *mdmap, const mdmatriggersource_t trigger_src,
			      const MDMANodeConfig *cfg);
  bool  mdma_lld_start(MDMADriver *mdmap);
  void  mdma_lld_stop(MDMADriver *mdmap);


  bool  mdma_lld_start_transfert(MDMADriver *mdmap);


  void  mdma_lld_stop_transfert(MDMADriver *mdmap);
  void  mdma_lld_get_link_block(MDMADriver *mdmap, const MDMANodeConfig *cfg,
				const mdmatriggersource_t trigger_src,
				const void *source, void *dest, const size_t block_len,
				mdmalinkblock_t *link_block);
  

  static inline void _mdma_isr_transaction_complete(MDMADriver *mdmap) {
    /* End transfert.*/
    mdma_lld_stop_transfert(mdmap);
  
    if (mdmap->config->end_cb != NULL) {
      mdmap->state = MDMA_COMPLETE;
      /* Invoke the callback passing the whole buffer.*/
      mdmap->config->end_cb(mdmap);
    
      if (mdmap->state == MDMA_COMPLETE) {
	mdmap->state = MDMA_READY;
      }
    } else {
      mdmap->state = MDMA_READY;
    }
    _mdma_wakeup_isr(mdmap);
  }
  
  

  static inline void _mdma_isr_error_code(MDMADriver *mdmap, mdmaerrormask_t err) {
#if CH_DBG_SYSTEM_STATE_CHECK == TRUE
    if (err & (MDMA_ERR_TRANSFERT_LINK_DATA |
	       MDMA_ERR_TRANSFERT_MASK |
	       MDMA_ERR_DIRECTION))
      mdmap->nbTransferError++;
    mdmap->lastError = err;
#endif
    mdma_lld_stop_transfert(mdmap);

    if (mdmap->config->error_cb != NULL) {
      mdmap->state = MDMA_ERROR;
      mdmap->config->error_cb(mdmap, err);
      if (mdmap->state == MDMA_ERROR)
	mdmap->state = MDMA_READY;
    } else {
      mdmap->state = MDMA_READY;
    }
    _mdma_timeout_isr(mdmap);
  }


#ifdef __cplusplus
}
#endif
