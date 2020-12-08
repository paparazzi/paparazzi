#pragma once
/**
 * @file    microrlShell.h
 * @brief   Simple CLI shell header.
 *
 * @addtogroup SHELL
 * @{
 */

#include "ch.h"
#include "hal.h"


#ifndef SHELL_DYNAMIC_ENTRIES_NUMBER
#define SHELL_DYNAMIC_ENTRIES_NUMBER 0U
#endif

// legacy compatibility
#define shellCreate(C, S, P) shellCreateFromHeap(C, S, P)

/**
 * @brief   Command handler function type.
 */
typedef void (shellcmd_f)(BaseSequentialStream *chp, int argc, const char * const argv[]);


/**
 * @brief   Custom command entry type.
 */
typedef struct {
  const char            *sc_name;           /**< @brief Command name.       */
  shellcmd_f            *sc_function;        /**< @brief Command function.   */
} ShellCommand;

/**
 * @brief   Shell descriptor type.
 */
typedef struct {
  BaseSequentialStream  *sc_channel;        /**< @brief I/O channel associated
                                                 to the shell.              */
  const ShellCommand    *sc_commands;       /**< @brief Shell extra commands
                                                 table.                     */
} ShellConfig;

#if !defined(__DOXYGEN__)
extern  event_source_t shell_terminated;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void shellInit(void);
#if CH_CFG_USE_HEAP && CH_CFG_USE_DYNAMIC
  thread_t *shellCreateFromHeap(const ShellConfig *scp, size_t size, tprio_t prio);
#endif
  thread_t *shellCreateStatic(const ShellConfig *scp, void *wsp,
			      size_t size, tprio_t prio);
#if  SHELL_DYNAMIC_ENTRIES_NUMBER
  bool shellAddEntry(const ShellCommand sc);
#endif
  bool shellGetLine(BaseSequentialStream *chp, char *line, unsigned size);
  void modeAlternate (void (*) (uint8_t c, uint32_t mode), uint32_t mode);
  void modeShell (void);
#ifdef __cplusplus
}
#endif


/** @} */
