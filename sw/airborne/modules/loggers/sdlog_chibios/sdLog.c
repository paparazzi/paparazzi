/*
 * Copyright (C) 2013-2016 Gautier Hattenberger, Alexandre Bustico
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file modules/loggers/sdlog_chibios/sdLog.c
 * @brief sdlog API using ChibiOS and Fatfs
 *
 */

#include <string.h>
#include <stdlib.h>
#include <ch.h>
#include <hal.h>
#include <ff.h>
#include "modules/loggers/sdlog_chibios/sdLog.h"
#include "printf.h"
#include "mcu_periph/sdio.h"
#include <ctype.h>


#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define MIN(x , y)  (((x) < (y)) ? (x) : (y))
#define MAX(x , y)  (((x) > (y)) ? (x) : (y))


// Number of files opened simultaneously
// Set to 2 by default since we have the default log file
// and optionally the flight recorder file
//
// WARNING:
// - if _FS_LOCK is set to 0, there is no limit on simultaneously opened file,
//   but no file locking, all fatfs operations should be done from same thread
// - if _FS_LOCK is > 0, each file is in a different subdir, so _FS_LOCK should
//   be set to 2 * nbFile
#ifndef SDLOG_NUM_BUFFER
#define SDLOG_NUM_BUFFER 2
#endif

#ifndef SDLOG_ALL_BUFFERS_SIZE
#error  SDLOG_ALL_BUFFERS_SIZE should be defined in mcuconf.h
#endif

#define SDLOG_WRITE_BUFFER_SIZE (SDLOG_ALL_BUFFERS_SIZE/SDLOG_NUM_BUFFER)

#ifndef SDLOG_MAX_MESSAGE_LEN
#error  SDLOG_MAX_MESSAGE_LEN should be defined in mcuconf.h
#endif

#ifndef SDLOG_QUEUE_BUCKETS
#error  SDLOG_QUEUE_BUCKETS should be defined in mcuconf.h
#endif

#if _FS_REENTRANT == 0
#warning "_FS_REENTRANT = 0 in ffconf.h DO NOT open close file during log"
#endif


#ifdef SDLOG_NEED_QUEUE
#include "modules/loggers/sdlog_chibios/msg_queue.h"

#if defined STM32F4XX
#define NODMA_SECTION ".ram4"
#define DMA_SECTION ".ram0"
#elif  defined STM32F7XX
#define NODMA_SECTION ".ram0"
#define DMA_SECTION ".ram3"
#else
#error "section defined only for STM32F4 and STM32F7"
#endif

static msg_t   queMbBuffer[SDLOG_QUEUE_BUCKETS] __attribute__((section(NODMA_SECTION), aligned(8))) ;
static MsgQueue messagesQueue;

#define WRITE_BYTE_CACHE_SIZE 15 // limit overhead :
// malloc (15+1) occupies 20bytes
typedef struct {
  uint8_t fcntl: 2;
  uint8_t fd: 6;
} FileOp;

struct LogMessage {
  FileOp op;
  char mess[0];
};

struct FilePoolUnit {
  FIL   fil;
  bool  inUse;
  bool  tagAtClose;
  // optimise write byte by caching at send level now that we are based upon tlsf where
  // allocation granularity is 16 bytes
  LogMessage *writeByteCache;
  uint8_t writeByteSeek;
};

static  struct FilePoolUnit fileDes[SDLOG_NUM_BUFFER] = {
  [0 ... SDLOG_NUM_BUFFER - 1] = {
    .fil = {0}, .inUse = false, .tagAtClose = false,
    .writeByteCache = NULL, .writeByteSeek = 0
  }
};

typedef enum {
  FCNTL_WRITE = 0b00,
  FCNTL_FLUSH = 0b01,
  FCNTL_CLOSE = 0b10,
  FCNTL_EXIT =  0b11
} FileFcntl;



#define LOG_MESSAGE_PREBUF_LEN (SDLOG_MAX_MESSAGE_LEN+sizeof(LogMessage))
#endif //  SDLOG_NEED_QUEUE


static FATFS fatfs; /* File system object */

#ifdef SDLOG_NEED_QUEUE
static size_t logMessageLen(const LogMessage *lm);
static size_t logRawLen(const size_t len);
static SdioError sdLoglaunchThread(void);
static SdioError sdLogStopThread(void);
static thread_t *sdLogThd = NULL;
static SdioError  getNextFIL(FileDes *fd);

#if (CH_KERNEL_MAJOR > 2)
static void thdSdLog(void *arg) ;
#else
static msg_t thdSdLog(void *arg) ;
#endif

#endif //  SDLOG_NEED_QUEUE




static int32_t uiGetIndexOfLogFile(const char *prefix, const char *fileName) ;
static inline SdioError flushWriteByteBuffer(const FileDes fd);

SdioError sdLogInit(uint32_t *freeSpaceInKo)
{
  DWORD clusters = 0;
  FATFS *fsp = NULL;

#ifdef SDLOG_NEED_QUEUE
  msgqueue_init(&messagesQueue, &HEAP_DEFAULT, queMbBuffer, SDLOG_QUEUE_BUCKETS);
#endif

  if (!sdc_lld_is_card_inserted(NULL)) {
    return  SDLOG_NOCARD;
  }


  sdio_connect();
  chThdSleepMilliseconds(10);
  sdio_disconnect();

  if (sdio_connect() == false) {
    return  SDLOG_NOCARD;
  }

#if _FATFS < 8000
  FRESULT rc = f_mount(0, &fatfs);
#else
  FRESULT rc = f_mount(&fatfs, "", 0);
#endif

  if (rc != FR_OK) {
    return SDLOG_FATFS_ERROR;
  }

  if (freeSpaceInKo != NULL) {
    f_getfree("/", &clusters, &fsp);
    *freeSpaceInKo = clusters * (uint32_t)fatfs.csize / 2;
  }

#ifdef SDLOG_NEED_QUEUE
  for (uint8_t i = 0; i < SDLOG_NUM_BUFFER; i++) {
    fileDes[i].inUse = fileDes[i].tagAtClose = false;
    fileDes[i].writeByteCache = NULL;
    fileDes[i].writeByteSeek = 0;
  }

  return sdLoglaunchThread();
#else
  return SDLOG_OK;
#endif

}


SdioError sdLogFinish(void)
{
#if _FATFS < 8000
  FRESULT rc = f_mount(0, NULL);
#else
  FRESULT rc = f_mount(NULL, "", 0);
#endif
  if (rc != FR_OK) {
    return SDLOG_FATFS_ERROR;
  }

  // if we mount, unmount, don't disconnect sdio
  /* if (sdio_disconnect () == false) */
  /*   return  SDLOG_NOCARD; */

  return  SDLOG_OK ;
}



#ifdef SDLOG_NEED_QUEUE
SdioError sdLogOpenLog(FileDes *fd, const char *directoryName, const char *prefix,
                       bool appendTagAtClose)
{
  FRESULT rc; /* fatfs result code */
  SdioError sde; /* sdio result code */
  //DIR dir; /* Directory object */
  //FILINFO fno; /* File information object */
  char fileName[32];

  sde = getNextFIL(fd);
  if (sde != SDLOG_OK) {
    return sde;
  }

  sde = getFileName(prefix, directoryName, fileName, sizeof(fileName), +1);
  if (sde != SDLOG_OK) {
    // sd card is not inserted, so logging task can be deleted
    return SDLOG_FATFS_ERROR;
  }


  rc = f_open(&fileDes[*fd].fil, fileName, FA_WRITE | FA_CREATE_ALWAYS);
  if (rc) {
    fileDes[*fd].inUse = false;
    return SDLOG_FATFS_ERROR;
  } else {
    fileDes[*fd].tagAtClose = appendTagAtClose;
  }

  return SDLOG_OK;
}


SdioError sdLogCloseAllLogs(bool flush)
{
  FRESULT rc = 0; /* Result code */



  //    do not flush what is in ram, close as soon as possible
  if (flush == false) {
    // stop worker thread then close file
    sdLogStopThread();
    for (FileDes fd = 0; fd < SDLOG_NUM_BUFFER; fd++) {
      if (fileDes[fd].inUse) {
        FIL *fileObject = &fileDes[fd].fil;

        FRESULT trc = f_close(fileObject);
        fileDes[fd].inUse = false;
        if (!rc) {
          rc = trc;
        }
      }
    }

    if (rc) {
      return SDLOG_FATFS_ERROR;
    }

    // flush ram buffer then close
  } else { // flush == true
    if (sdLogThd == NULL) {
      // something goes wrong, log thread is no more working
      return SDLOG_NOTHREAD;
    }

    // queue flush + close order, then stop worker thread
    for (FileDes fd = 0; fd < SDLOG_NUM_BUFFER; fd++) {
      if (fileDes[fd].inUse) {
        flushWriteByteBuffer(fd);
        sdLogCloseLog(fd);
      }
    }

    LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
    if (lm == NULL) {
      return SDLOG_QUEUEFULL;
    }

    lm->op.fcntl = FCNTL_EXIT;

    if (msgqueue_send(&messagesQueue, lm, sizeof(LogMessage), MsgQueue_REGULAR) < 0) {
      return SDLOG_QUEUEFULL;
    } else {
      chThdWait(sdLogThd);
      sdLogThd = NULL;
    }

  }
  return SDLOG_OK;
}



SdioError sdLogWriteLog(const FileDes fd, const char *fmt, ...)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }

  flushWriteByteBuffer(fd);

  va_list ap;
  va_start(ap, fmt);

  LogMessage *lm = tlsf_malloc_r(&HEAP_DEFAULT, LOG_MESSAGE_PREBUF_LEN);
  if (lm == NULL) {
    return SDLOG_QUEUEFULL;
  }

  lm->op.fcntl = FCNTL_WRITE;
  lm->op.fd = fd & 0x1f;

  chvsnprintf(lm->mess, SDLOG_MAX_MESSAGE_LEN - 1,  fmt, ap);
  lm->mess[SDLOG_MAX_MESSAGE_LEN - 1] = 0;
  va_end(ap);

  const size_t msgLen =  logMessageLen(lm);
  lm = tlsf_realloc_r(&HEAP_DEFAULT, lm, msgLen);
  if (lm == NULL) {
    return SDLOG_QUEUEFULL;
  }

  if (msgqueue_send(&messagesQueue, lm, msgLen, MsgQueue_REGULAR) < 0) {
    return SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}

SdioError sdLogFlushLog(const FileDes fd)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }

  flushWriteByteBuffer(fd);
  LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
  if (lm == NULL) {
    return SDLOG_QUEUEFULL;
  }

  lm->op.fcntl = FCNTL_FLUSH;
  lm->op.fd = fd & 0x1f;

  if (msgqueue_send(&messagesQueue, lm, sizeof(LogMessage), MsgQueue_REGULAR) < 0) {
    return SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}

SdioError sdLogCloseLog(const FileDes fd)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }

  LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
  if (lm == NULL) {
    return SDLOG_QUEUEFULL;
  }

  lm->op.fcntl = FCNTL_CLOSE;
  lm->op.fd = fd & 0x1f;

  if (msgqueue_send(&messagesQueue, lm, sizeof(lm), MsgQueue_REGULAR) < 0) {
    return SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}





static inline SdioError flushWriteByteBuffer(const FileDes fd)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }

  if (unlikely(fileDes[fd].writeByteCache != NULL)) {
    if (msgqueue_send(&messagesQueue, fileDes[fd].writeByteCache,
                      sizeof(LogMessage) + fileDes[fd].writeByteSeek,
                      MsgQueue_REGULAR) < 0) {
      return SDLOG_QUEUEFULL;
    }
    fileDes[fd].writeByteCache = NULL;
  }
  return SDLOG_OK;
}

SdioError sdLogWriteRaw(const FileDes fd, const uint8_t *buffer, const size_t len)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }

  flushWriteByteBuffer(fd);
  LogMessage *lm = tlsf_malloc_r(&HEAP_DEFAULT, logRawLen(len));
  if (lm == NULL) {
    return SDLOG_QUEUEFULL;
  }

  lm->op.fcntl = FCNTL_WRITE;
  lm->op.fd = fd & 0x1f;
  memcpy(lm->mess, buffer, len);

  if (msgqueue_send(&messagesQueue, lm, logRawLen(len), MsgQueue_REGULAR) < 0) {
    return SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}



SdioError sdLogWriteByte(const FileDes fd, const uint8_t value)
{
  if ((fd >= SDLOG_NUM_BUFFER) || (fileDes[fd].inUse == false)) {
    return SDLOG_FATFS_ERROR;
  }
  LogMessage *lm;

  if (fileDes[fd].writeByteCache == NULL) {
    lm = tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage) + WRITE_BYTE_CACHE_SIZE);
    if (lm == NULL) {
      return SDLOG_QUEUEFULL;
    }

    lm->op.fcntl = FCNTL_WRITE;
    lm->op.fd = fd & 0x1f;

    fileDes[fd].writeByteCache = lm;
    fileDes[fd].writeByteSeek = 0;
  } else {
    lm = fileDes[fd].writeByteCache;
  }

  lm->mess[fileDes[fd].writeByteSeek++] = value;

  if (fileDes[fd].writeByteSeek == WRITE_BYTE_CACHE_SIZE) {
    return flushWriteByteBuffer(fd);
  }
  return SDLOG_OK;
}




/* enregistrer les fichiers ouverts de manière à les fermer
   si necessaire
   */
static THD_WORKING_AREA(waThdSdLog, 1024);
SdioError sdLoglaunchThread()
{
  chThdSleepMilliseconds(100);

  sdLogThd = chThdCreateStatic(waThdSdLog, sizeof(waThdSdLog),
                               NORMALPRIO + 1, thdSdLog, NULL);
  if (sdLogThd == NULL) {
    return SDLOG_INTERNAL_ERROR;
  } else {
    return SDLOG_OK;
  }
}

SdioError sdLogStopThread(void)
{
  SdioError retVal = SDLOG_OK;

  if (sdLogThd == NULL) {
    return SDLOG_NOTHREAD;
  }

  LogMessage lm;

  // ask for closing (after flushing) all opened files
  for (uint8_t i = 0; i < SDLOG_NUM_BUFFER; i++) {
    if (fileDes[i].inUse) {
      flushWriteByteBuffer(i);
      lm.op.fcntl = FCNTL_CLOSE;
      lm.op.fd = i & 0x1f;
      if (msgqueue_copy_send(&messagesQueue, &lm, sizeof(LogMessage), MsgQueue_OUT_OF_BAND) < 0) {
        retVal = SDLOG_QUEUEFULL;
      }
    }
  }

  lm.op.fcntl = FCNTL_EXIT;
  if (msgqueue_copy_send(&messagesQueue, &lm, sizeof(LogMessage), MsgQueue_OUT_OF_BAND) < 0) {
    retVal = SDLOG_QUEUEFULL;
  }

  chThdTerminate(sdLogThd);
  chThdWait(sdLogThd);
  sdLogThd = NULL;
  return retVal;
}
#endif


SdioError getFileName(const char *prefix, const char *directoryName,
                      char *nextFileName, const size_t nameLength, const int indexOffset)
{
  DIR dir; /* Directory object */
  FRESULT rc; /* Result code */
  FILINFO fno; /* File information object */
  int32_t fileIndex ;
  int32_t maxCurrentIndex = 0;
  char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
  char lfn[_MAX_LFN + 1];
  fno.lfname = lfn;
  fno.lfsize = sizeof lfn;
#endif
  const size_t directoryNameLen = MIN(strlen(directoryName), 128);
  const size_t slashDirNameLen = directoryNameLen + 2;
  char slashDirName[slashDirNameLen];
  strlcpy(slashDirName, "/", slashDirNameLen);
  strlcat(slashDirName, directoryName, slashDirNameLen);

  rc = f_opendir(&dir, directoryName);
  if (rc != FR_OK) {
    rc = f_mkdir(slashDirName);
    if (rc != FR_OK) {
      return SDLOG_FATFS_ERROR;
    }
    rc = f_opendir(&dir, directoryName);
    if (rc != FR_OK) {
      return SDLOG_FATFS_ERROR;
    }
  }

  for (;;) {
    rc = f_readdir(&dir, &fno); /* Read a directory item */
    if (rc != FR_OK || fno.fname[0] ==  0) { break; } /* Error or end of dir */
#if _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
    if (fn[0] == '.') { continue; }

    if (!(fno.fattrib & AM_DIR)) {
      //      DebugTrace ("fno.fsize=%d  fn=%s\n", fno.fsize, fn);
      fileIndex = uiGetIndexOfLogFile(prefix, fn);
      maxCurrentIndex = MAX(maxCurrentIndex, fileIndex);
    }
  }
  if (rc) {
    return SDLOG_FATFS_ERROR;
  }

  rc = f_closedir(&dir);
  if (rc) {
    return SDLOG_FATFS_ERROR;
  }

  if (maxCurrentIndex < NUMBERMAX) {
    chsnprintf(nextFileName, nameLength, NUMBERFMF,
               directoryName, prefix, maxCurrentIndex + indexOffset);
    return SDLOG_OK;
  } else {
    chsnprintf(nextFileName, nameLength, "%s\\%s%.ERR",
               directoryName, prefix);
    return SDLOG_LOGNUM_ERROR;
  }
}

SdioError removeEmptyLogs(const char *directoryName, const char *prefix, const size_t sizeConsideredEmpty)
{
  DIR dir; /* Directory object */
  FRESULT rc; /* Result code */
  FILINFO fno; /* File information object */
  char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
  char lfn[_MAX_LFN + 1];
  fno.lfname = lfn;
  fno.lfsize = sizeof lfn;
#endif

  rc = f_opendir(&dir, directoryName);
  if (rc != FR_OK) {
    return SDLOG_FATFS_NOENT;
  }

  for (;;) {
    rc = f_readdir(&dir, &fno); /* Read a directory item */
    if (rc != FR_OK || fno.fname[0] ==  0) { break; } /* Error or end of dir */
#if _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
    if (fn[0] == '.') { continue; }

    if (!(fno.fattrib & AM_DIR)) {
      //      DebugTrace ("fno.fsize=%d  fn=%s\n", fno.fsize, fn);
      if ((strncmp(fn, prefix, strlen(prefix)) == 0) && (fno.fsize <= sizeConsideredEmpty)) {
        char absPathName[128];
        strlcpy(absPathName, directoryName, sizeof(absPathName));
        strlcat(absPathName, "/", sizeof(absPathName));
        strlcat(absPathName, fn, sizeof(absPathName));
        rc = f_unlink(absPathName);
        if (rc) {
          break;
        }
      }
    }
  }

  if (rc) {
    return SDLOG_FATFS_ERROR;
  }

  rc = f_closedir(&dir);
  if (rc) {
    return SDLOG_FATFS_ERROR;
  }

  return SDLOG_OK;
}

/*
#                 _____           _                    _
#                |  __ \         (_)                  | |
#                | |__) |  _ __   _   __   __   __ _  | |_     ___
#                |  ___/  | '__| | |  \ \ / /  / _` | | __|   / _ \
#                | |      | |    | |   \ V /  | (_| | \ |_   |  __/
#                |_|      |_|    |_|    \_/    \__,_|  \__|   \___|
*/







int32_t uiGetIndexOfLogFile(const char *prefix, const char *fileName)
{
  const size_t len = strlen(prefix);

  // if filename does not began with prefix, return 0
  if (strncmp(prefix, fileName, len) != 0) {
    return 0;
  }

  // we point on the first char after prefix
  const char *suffix = &(fileName[len]);

  // we test that suffix is valid (at least begin with digit)
  if (!isdigit((int) suffix[0])) {
    //      DebugTrace ("DBG> suffix = %s", suffix);
    return 0;
  }

  return (int32_t) atoi(suffix);
}


#ifdef SDLOG_NEED_QUEUE
#if (CH_KERNEL_MAJOR > 2)
static void thdSdLog(void *arg)
#else
static msg_t thdSdLog(void *arg)
#endif
{
  (void) arg;
  struct PerfBuffer {
    uint8_t buffer[SDLOG_WRITE_BUFFER_SIZE];
    uint16_t size;
  } ;

  UINT bw;
  static struct PerfBuffer perfBuffers[SDLOG_NUM_BUFFER] __attribute__((section(DMA_SECTION), aligned(8))) = {
    [0 ... SDLOG_NUM_BUFFER - 1] = {.buffer = {0}, .size = 0}
  };

  // FIXME above initialization doesn't seem to work for all GCC version
  // for now, also doing the good old way.
  memset(perfBuffers, 0, SDLOG_NUM_BUFFER * sizeof(struct PerfBuffer));

  chRegSetThreadName("thdSdLog");
  while (!chThdShouldTerminateX()) {
    LogMessage *lm;
    const int32_t retLen = (int32_t)(msgqueue_pop(&messagesQueue, (void **) &lm));
    if (retLen > 0) {
      FIL *fo =  &fileDes[lm->op.fd].fil;
      uint8_t *const perfBuffer = perfBuffers[lm->op.fd].buffer;

      switch (lm->op.fcntl) {

        case FCNTL_FLUSH:
        case FCNTL_CLOSE: {
          const uint16_t curBufFill = perfBuffers[lm->op.fd].size;
          if (fileDes[lm->op.fd].inUse) {
            if (curBufFill) {
              f_write(fo, perfBuffer, curBufFill, &bw);
              perfBuffers[lm->op.fd].size = 0;
            }
            if (lm->op.fcntl ==  FCNTL_FLUSH) {
              f_sync(fo);
            } else { // close
              if (fileDes[lm->op.fd].tagAtClose) {
                f_write(fo, "\r\nEND_OF_LOG\r\n", 14, &bw);
              }
              f_close(fo);
              fileDes[lm->op.fd].inUse = false; // store that file is closed
            }
          }
        }
        break;

        case FCNTL_EXIT:
          chThdExit(SDLOG_OK);
          break; /* To exit from thread when asked : chThdTerminate
      then send special message with FCNTL_EXIT   */


        case FCNTL_WRITE: {
          const uint16_t curBufFill = perfBuffers[lm->op.fd].size;
          if (fileDes[lm->op.fd].inUse) {
            const int32_t messLen = retLen - (int32_t)(sizeof(LogMessage));
            if (messLen < (SDLOG_WRITE_BUFFER_SIZE - curBufFill)) {
              // the buffer can accept this message
              memcpy(&(perfBuffer[curBufFill]), lm->mess, (size_t)(messLen));
              perfBuffers[lm->op.fd].size = (uint16_t)((perfBuffers[lm->op.fd].size) + messLen);
            } else {
              // fill the buffer
              const int32_t stayLen = SDLOG_WRITE_BUFFER_SIZE - curBufFill;
              memcpy(&(perfBuffer[curBufFill]), lm->mess, (size_t)(stayLen));
              FRESULT rc = f_write(fo, perfBuffer, SDLOG_WRITE_BUFFER_SIZE, &bw);
              f_sync(fo);
              if (rc) {
                chThdExit(SDLOG_FATFS_ERROR);
              } else if (bw != SDLOG_WRITE_BUFFER_SIZE) {
                chThdExit(SDLOG_FSFULL);
              }

              memcpy(perfBuffer, &(lm->mess[stayLen]), (uint32_t)(messLen - stayLen));
              perfBuffers[lm->op.fd].size = (uint16_t)(messLen - stayLen); // curBufFill
            }
          }
        }
      }
      tlsf_free_r(&HEAP_DEFAULT, lm);
    } else {
      chThdExit(SDLOG_INTERNAL_ERROR);
    }
  }
#if (CH_KERNEL_MAJOR == 2)
  return SDLOG_OK;
#endif
}

static size_t logMessageLen(const LogMessage *lm)
{
  return sizeof(LogMessage) + strnlen(lm->mess, SDLOG_MAX_MESSAGE_LEN);
}

static size_t logRawLen(const size_t len)
{
  return sizeof(LogMessage) + len;
}

static SdioError  getNextFIL(FileDes *fd)
{
  // if there is a free slot in fileDes, use it
  // else, if all slots are buzy, maximum open files limit
  // is reach.
  for (FileDes i = 0; i < SDLOG_NUM_BUFFER; i++) {
    if (fileDes[i].inUse ==  false) {
      *fd = i;
      fileDes[i].inUse = true;
      return SDLOG_OK;
    }
  }
  return SDLOG_FDFULL;
}

#endif
