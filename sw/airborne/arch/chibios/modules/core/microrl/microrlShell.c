/**
 * @file    shell.c
 * @brief   Enhanced CLI shell code.
 *
 * @addtogroup SHELL
 * @{
 */

#include <string.h>
#include "microrl/microrlShell.h"
#include "microrl/microrl.h"
#include "printf.h"
//#include "stdutil.h"


#define printScreen(...) {chprintf (chpg, __VA_ARGS__); chprintf (chpg, "\r\n");}

typedef struct {
  void (*altFunc) (uint8_t c, uint32_t mode);
  uint32_t param;
} AltCbParam;
  
static void cmd_info(BaseSequentialStream *lchp, int argc,
		     const char * const argv[]);


/**
 * @brief   Shell termination event source.
 */
 event_source_t shell_terminated;

static MUTEX_DECL(mut);
static microrl_t rl;
static BaseSequentialStream *chpg;
static const ShellCommand *staticCommands = NULL;
static const char * complWorlds[64];
/**
 * @brief   Array of the default  and dynamic commands.
 */
#if SHELL_DYNAMIC_ENTRIES_NUMBER // compatibility with legacy static only behavior
static ShellCommand localCommands[SHELL_DYNAMIC_ENTRIES_NUMBER + 2U] =  {
#else
static const ShellCommand localCommands[] =  {
#endif
  {"info", cmd_info},
  {NULL, NULL}
};

static AltCbParam altCbParam = {.altFunc = NULL, .param = 0};

void microrlPrint (const char * str)
{
  int i = 0;

  while (str[i] != 0) {
    streamPut(chpg, str[i++]);
  }
}

void microrlExecute (int argc,  const char * const *argv)
{
  const ShellCommand *scp = staticCommands;
  const char *name = argv[0];

  chMtxLock(&mut);

  while (scp->sc_name != NULL) {
    if (strcasecmp(scp->sc_name, name) == 0) {
      scp->sc_function(chpg, argc-1, &argv[1]);
      goto exit;
    }
    scp++;
  }

  scp = localCommands;
   while (scp->sc_name != NULL) {
    if (strcasecmp(scp->sc_name, name) == 0) {
      scp->sc_function(chpg, argc-1, &argv[1]);
      goto exit;
    }
    scp++;
  }
   
 exit:
   chMtxUnlock(&mut);
}

const char ** microrlComplet (int argc, const char * const * argv)
{
  uint32_t j = 0;

  complWorlds [0] = NULL;
  chMtxLock(&mut);
  
  // if there is token in cmdline
  if (argc == 1) {
    // get last entered token
    const char *bit = argv[argc-1];
    // iterate through our available token and match it
    for (const ShellCommand *scp = localCommands;
	 scp->sc_name != NULL; scp++) {
      // if token is matched (text is part of our token starting from 0 char)
      if (strstr(scp->sc_name, bit) == scp->sc_name) {
	// add it to completion set
	complWorlds[j++] = scp->sc_name;
      }
    }
    for (const ShellCommand *scp = staticCommands;
	 scp->sc_name != NULL; scp++) {
      // if token is matched (text is part of our token starting from 0 char)
      if (strstr(scp->sc_name, bit) == scp->sc_name) {
	// add it to completion set
	complWorlds[j++] = scp->sc_name;
      }
    }
  } else { // if there is no token in cmdline, just print all available token
    for (const ShellCommand *scp = localCommands; scp->sc_name != NULL; scp++)
      complWorlds[j++] = scp->sc_name;
    for (const ShellCommand *scp = staticCommands; scp->sc_name != NULL; scp++)
      complWorlds[j++] = scp->sc_name;
  }

  // note! last ptr in array always must be NULL!!!
  complWorlds[j] = NULL;
  chMtxUnlock(&mut);
  // return set of variants
  return complWorlds;
}


void microrlSigint (void)
{
  chprintf (chpg, "^C catched!\n\r");
}


static void usage(BaseSequentialStream *lchp, char *p) {

  chprintf(lchp, "Usage: %s\r\n", p);
}



static void cmd_info(BaseSequentialStream *lchp, int argc,  const char * const argv[]) {

  (void)argv;
  if (argc > 0) {
    usage(lchp, "info");
    return;
  }

  /*
    Bits 31:16 REV_ID[15:0] Revision identifier
    This field indicates the revision of the device.
    STM32F405xx/07xx and STM32F415xx/17xx devices:
    0x1000 = Revision A
    0x1001 = Revision Z
    0x1003 = Revision 1
    0x1007 = Revision 2
    0x100F= Revision Y
    STM32F42xxx and STM32F43xxx devices:
    0x1000 = Revision A
    0x1003 = Revision Y
    0x1007 = Revision 1
    0x2001= Revision 3
    Bits 15:12 Reserved, must be kept at reset value.
    Bits 11:0 DEV_ID[11:0]: Device identifier (STM32F405xx/07xx and STM32F415xx/17xx)
    The device ID is 0x413.
    Bits 11:0 DEV_ID[11:0]: Device identifier (STM32F42xxx and STM32F43xxx)
    The device ID is 0x419


    F7
    Bits 31:16 REV_ID[15:0] Revision identifier
    This field indicates the revision of the device:
    0x1000 = Revision A
    0x1001 = Revision Z
    Bits 15:12 Reserved, must be kept at reset value.
    Bits 11:0 DEV_ID[11:0]: Device identifier
    The device ID is 0x449.


    L47x 49x
    Bits 31:16 REV_ID[15:0] Revision identifier
    This field indicates the revision of the device.
    For STM32L475xx/476xx/486xx devices
    0x1000: Rev 1
    0x1001: Rev 2
    0x1003: Rev 3
    0x1007: Rev 4
    For STM32L496xx/4A6xx devices
    0x1000: Rev A
    0x2000: Rev B

    Bits 11:0 DEV_ID[11:0]: Device identifier
    The device ID is:
    0x461 for STM32L496xx/4A6xx devices
    0x415 for STM32L475xx/476xx/486xx devices.
  */
  

  const uint16_t mcu_revid = (DBGMCU->IDCODE &  DBGMCU_IDCODE_REV_ID) >> 16;
  const uint16_t mcu_devid =  DBGMCU->IDCODE &  DBGMCU_IDCODE_DEV_ID;
  char *mcu_devid_str ="not known, please fix microrlShell.c";
  char mcu_revid_chr = '?';

  switch (mcu_devid) {
  case  0x415 : mcu_devid_str = "STM32L475xx/476xx/486xx devices";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = '1'; break;
    case 0x1001 : mcu_revid_chr = '2'; break;
    case 0x1003 : mcu_revid_chr = '3'; break;
    case 0x1007 : mcu_revid_chr = '4'; break;
    }
    break;
  case  0x461 : mcu_devid_str = "STM32L496xx/4A6xx devices";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x2000 : mcu_revid_chr = 'B'; break;
    }
    break;
  case  0x411 : mcu_devid_str = "STM32F2xx and *EARLY* STM32F40x and 41x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    case 0x2000 : mcu_revid_chr = 'B'; break;
    case 0x2001 : mcu_revid_chr = 'Y'; break;
    case 0x2003 : mcu_revid_chr = 'X'; break;
    }
    break;
  case  0x413 : mcu_devid_str = "STM32F40x and 41x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    case 0x1003 : mcu_revid_chr = '1'; break;
    case 0x1007 : mcu_revid_chr = '2'; break;
    case 0x100F : mcu_revid_chr = 'Y'; break;
    }
    break;
  case  0x419 : mcu_devid_str = "STM32F42x and F43x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1003 : mcu_revid_chr = 'Y'; break;
    case 0x1007 : mcu_revid_chr = '1'; break;
    case 0x2001 : mcu_revid_chr = '3'; break;
    }
    break;
  case  0x449 : mcu_devid_str = "STM32F74x and STM32F75x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    }
    break;
  case  0x451 : mcu_devid_str = "STM32F76x and STM32F77x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    }
    break;
  case  0x435 : mcu_devid_str = "STM32L43x";
    switch (mcu_revid) {
    case 0x1000 : mcu_revid_chr = 'A'; break;
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    }
    break;
  case  0x446 : mcu_devid_str = "STM32F303xD/E and STM32F398xE";
    switch (mcu_revid) {
    case 0x1001 : mcu_revid_chr = 'Z'; break;
    case 0x1003 : mcu_revid_chr = 'Y'; break;
    }
    break;
  }
  
  chprintf(lchp, "Kernel:       %s\r\n", CH_KERNEL_VERSION);
#ifdef HAL_VERSION
  chprintf(lchp, "Hal:          %s\r\n", HAL_VERSION);
#endif

#ifdef CH_COMPILER_NAME
  chprintf(lchp, "Compiler:     %s\r\n", CH_COMPILER_NAME);
#endif
#ifdef PORT_COMPILER_NAME
  chprintf(lchp, "Compiler:     %s\r\n", PORT_COMPILER_NAME);
#endif

#ifdef CH_ARCHITECTURE_NAME
  chprintf(lchp, "Architecture: %s\r\n", CH_ARCHITECTURE_NAME);
#endif
#ifdef PORT_ARCHITECTURE_NAME
  chprintf(lchp, "Architecture: %s\r\n", PORT_ARCHITECTURE_NAME);
#endif
  

#ifdef CH_CORE_VARIANT_NAME
  chprintf(lchp, "Core Variant: %s\r\n", CH_CORE_VARIANT_NAME);
#endif
#ifdef PORT_CORE_VARIANT_NAME
  chprintf(lchp, "Core Variant: %s\r\n", PORT_CORE_VARIANT_NAME);
#endif
#ifdef STM32_SYSCLK
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
  chprintf(lchp, "Main STM32_SYSCLK frequency %.2f Mhz\r\n", STM32_SYSCLK/1e6f);
#pragma GCC diagnostic pop
#endif

#ifdef CH_PORT_INFO
  chprintf(lchp, "Port Info:    %s\r\n", CH_PORT_INFO);
#endif
#ifdef PORT_INFO
  chprintf(lchp, "Port Info:    %s\r\n", PORT_INFO);
#endif

#ifdef PLATFORM_NAME
  chprintf(lchp, "Platform:     %s\r\n", PLATFORM_NAME);
#endif

#ifdef BOARD_NAME
  chprintf(lchp, "Board:        %s\r\n", BOARD_NAME);
#endif

  chprintf(lchp, "Chip Revision: %s REV '%c' (0x%x:0x%x)\r\n", mcu_devid_str, mcu_revid_chr, mcu_devid, mcu_revid);

#if (!defined STM32_USE_REVISION_A_FIX) || (STM32_USE_REVISION_A_FIX == 0)
  if ((mcu_devid == 0x413) && (mcu_revid_chr == 'A')) {
    chprintf(lchp, "Chip Revision: %s REV '%c' PLEASE define STM32_USE_REVISION_A_FIX in mcuconf.h !!\r\n",
	     mcu_devid_str, mcu_revid_chr);
  }
#endif
  
#ifdef __DATE__
#ifdef __TIME__
  chprintf(lchp, "Build time:   %s%s%s\r\n", __DATE__, " - ", __TIME__);
#endif
#endif

  chprintf(lchp, "systime= %lu\r\n", (unsigned long)chVTGetSystemTimeX());
}


/**
 * @brief   Shell thread function.
 *
 * @param[in] p         pointer to a @p BaseSequentialStream object
 * @return              Termination reason.
 * @retval MSG_OK       terminated by command.
 * @retval RDY_RESET    terminated by reset condition on the I/O channel.
 */

static THD_FUNCTION(shell_thread, p) {
  msg_t msg = MSG_OK;
  chpg = ((ShellConfig *)p)->sc_channel;
  staticCommands = ((ShellConfig *)p)->sc_commands;
  bool readOk=TRUE;

  
  chRegSetThreadName("Enhanced_shell");
  printScreen ("ChibiOS/RT Enhanced Shell");
  while (!chThdShouldTerminateX() && readOk) {
    uint8_t c;
    if (streamRead(chpg, &c, 1) == 0) {
       readOk=FALSE;
    } else {
      if (altCbParam.altFunc == NULL) {
	microrl_insert_char (&rl, c);
      } else {
	(*altCbParam.altFunc) (c, altCbParam.param);
      }
    }
  }
  /* Atomically broadcasting the event source and terminating the thread,
     there is not a chSysUnlock() because the thread terminates upon return.*/
  printScreen ("exit");
  chSysLock();
  chEvtBroadcastI(&shell_terminated);
  chThdExitS(msg);
}

/**
 * @brief   Shell manager initialization.
 */
void shellInit(void) {
  chEvtObjectInit(&shell_terminated);
  microrl_init (&rl, microrlPrint);
  microrl_set_execute_callback (&rl, &microrlExecute);
  microrl_set_complete_callback (&rl, &microrlComplet);
  microrl_set_sigint_callback (&rl, &microrlSigint);
}

/**
 * @brief   Spawns a new shell.
 * @pre     @p CH_USE_MALLOC_HEAP and @p CH_USE_DYNAMIC must be enabled.
 *
 * @param[in] scp       pointer to a @p ShellConfig object
 * @param[in] size      size of the shell working area to be allocated
 * @param[in] prio      priority level for the new shell
 * @return              A pointer to the shell thread.
 * @retval NULL         thread creation failed because memory allocation.
 */
#if CH_CFG_USE_HEAP && CH_CFG_USE_DYNAMIC
thread_t *shellCreateFromHeap(const ShellConfig *scp, size_t size, tprio_t prio) {
  return chThdCreateFromHeap(NULL, size, "shell", prio, shell_thread, (void *)scp);
}
#endif


/**
 * @brief   Create statically allocated shell thread.
 *
 * @param[in] scp       pointer to a @p ShellConfig object
 * @param[in] wsp       pointer to a working area dedicated to the shell thread stack
 * @param[in] size      size of the shell working area
 * @param[in] prio      priority level for the new shell
 * @return              A pointer to the shell thread.
 */
 thread_t *shellCreateStatic(const ShellConfig *scp, void *wsp,
                          size_t size, tprio_t prio) {
  return chThdCreateStatic(wsp, size, prio, shell_thread, (void *)scp);
}

#if  SHELL_DYNAMIC_ENTRIES_NUMBER
/**
 * @brief   add/change/remove dynamically new shell entries
 * @pre     @p SHELL_DYNAMIC_ENTRIES_NUMBER must be set
 * @note    if sc.sc_name already exists, entry will be overwrite
            if sc.sc_name already exists and sc.sc_function is null, 
            entry will be removed
 * @param[in] sc        ShellCommand object
 * @return              true : OK, false : error table is full.
 */
bool shellAddEntry(const ShellCommand sc)
{
  ShellCommand* lcp = localCommands;
  static const int len = (sizeof(localCommands) / sizeof(localCommands[0]))
    -1U;
  chMtxLock(&mut);
  while ((lcp->sc_function != NULL) &&
	 (strcmp(lcp->sc_name, sc.sc_name) != 0) &&
	 ((lcp - localCommands) < len))
    lcp++;
  
  if ((lcp - localCommands) == len) {
    chMtxUnlock(&mut);
    return false;
  }
  lcp->sc_function = sc.sc_function;
  lcp->sc_name = sc.sc_function == NULL ? NULL : sc.sc_name;
  *(++lcp) = (ShellCommand){NULL, NULL};
  
  chMtxUnlock(&mut);
  return true;
}
#endif

void modeAlternate(void (*funcp) (uint8_t c, uint32_t mode), uint32_t mode)
{
  chMtxLock(&mut);
  altCbParam.altFunc = funcp;
  altCbParam.param = mode;
  chMtxUnlock(&mut);
}

void modeShell(void)
{
  chMtxLock(&mut);
  altCbParam.altFunc = NULL;
  chMtxUnlock(&mut);
  printScreen ("retour au shell");
}



