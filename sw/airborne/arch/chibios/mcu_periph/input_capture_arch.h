#pragma once

#include "ch.h"
#include "hal.h"


#ifdef __cplusplus
extern "C" {
#endif

  enum  TimICMode {TIMIC_INPUT_CAPTURE, TIMIC_PWM_IN} ;
  enum  TimICActive {CH1_RISING_EDGE=1<<0, CH1_FALLING_EDGE=1<<1,  CH1_BOTH_EDGES=1<<2,
		 CH2_RISING_EDGE=1<<3, CH2_FALLING_EDGE=1<<4,  CH2_BOTH_EDGES=1<<5,
		 CH3_RISING_EDGE=1<<6, CH3_FALLING_EDGE=1<<7,  CH3_BOTH_EDGES=1<<8,
		 CH4_RISING_EDGE=1<<9, CH4_FALLING_EDGE=1<<10, CH4_BOTH_EDGES=1<<11
  } ;
  enum TimICState {TIMIC_STOP, TIMIC_READY, TIMIC_ACTIVE};
  typedef struct TimICDriver TimICDriver;
  typedef void (*TimICCallbackCapture_t)(const TimICDriver *timicp, uint32_t channel, uint32_t capture);
  typedef void (*TimICCallbackOverflow_t)(const TimICDriver *timicp);

  /**
   * @brief   TimIC Driver configuration structure.
   */
  typedef struct {
    /**
     * @brief   hardware timer pointer (example : &STM32_TIM1)
     */
    stm32_tim_t *timer;
    TimICCallbackCapture_t capture_cb;
    TimICCallbackOverflow_t overflow_cb;
    enum TimICMode	mode;
    uint32_t active;
    uint32_t dier;
    uint32_t dcr;
    uint32_t prescaler:17;
    uint32_t arr;
  } TimICConfig;


  /**
   * @brief   Structure representing a TimIC driver.
   */
  struct TimICDriver {
    /**
     * @brief   Current configuration data.
     */
    const TimICConfig *config;
    /**
     * @brief   mutex to protect data read/write in concurrent context
     */
    uint32_t channel;
    uint32_t dier;
    mutex_t mut;
    enum TimICState state;
  };


  /**
   * @brief   Initializes an input capture driver
   *
   * @param[out]  inputCapturep     pointer to a @p TimICDriver structure
   * @init
   */
  void timIcObjectInit(TimICDriver *timicp);

  /**
   * @brief   start an input capture driver
   *
   * @param[in]  timicp     pointer to a @p TimICDriver structure
   * @param[in]   configp    pointer to a @p TimICConfig structure
   * @brief configure the timer to get input capture data from timer
   */
  void timIcStart(TimICDriver *timicp, const TimICConfig *configp);

  /**
   * @brief   start to capture
   *
   * @param[in]  timicp     pointer to a @p TimICDriver structure
   * @brief start the input capture data from timer
   */
  void timIcStartCapture(TimICDriver *timicp);

 /**
   * @brief   stop to capture
   *
   * @param[in]  timicp     pointer to a @p TimICDriver structure
   * @brief stop the input capture data from timer
   */
  void timIcStopCapture(TimICDriver *timicp);

  /**
   * @brief   stop a quadrature encoder driver
   *
   * @param[in]  timicp     pointer to a @p TimICDriver structure
   * @brief stop and release the timer. After stop, any operation on timicp
   *        will result in undefined behavior and probably hardware fault
   */
   void timIcStop(TimICDriver *timicp);

   void timIcRccEnable(const TimICDriver * const timicp);
   void timIcRccDisable(const TimICDriver * const timicp);



#ifdef __cplusplus
}
#endif
