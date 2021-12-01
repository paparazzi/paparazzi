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

#ifndef MIN
#define MIN(x , y)  (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x , y)  (((x) > (y)) ? (x) : (y))
#endif
#define IS_POWER_OF_TWO(s) ((s) && !((s) & ((s) - 1)))

#ifndef SDLOG_NUM_FILES
#error  SDLOG_NUM_FILES should be defined in mcuconf.h
#endif


#ifndef FFCONF_DEF
#define  FFCONF_DEF _FATFS
#endif

#if FFCONF_DEF < 8000
#error  upgrade FATFS to 0.14 at least
#else // FFCONF_DEF > 8000
#if FF_FS_LOCK != 0 && FF_FS_LOCK < SDLOG_NUM_FILES
#error  if FF_FS_LOCK is not zero, it should be equal of superior to SDLOG_NUM_FILES
#endif
#endif

#ifndef  SDLOG_ALL_BUFFERS_SIZE
#error SDLOG_ALL_BUFFERS_SIZE should be defined in mcuconf.h
#endif

#if  SDLOG_ALL_BUFFERS_SIZE > 65536
#error constraint 512 <=  SDLOG_ALL_BUFFERS_SIZE <= 65536 not meet
#endif

#define SDLOG_WRITE_BUFFER_SIZE (SDLOG_ALL_BUFFERS_SIZE/SDLOG_NUM_FILES)

#ifndef SDLOG_MAX_MESSAGE_LEN
#error  SDLOG_MAX_MESSAGE_LENshould be defined in mcuconf.h
#endif

#ifndef SDLOG_QUEUE_BUCKETS
#error  SDLOG_QUEUE_BUCKETS should be defined in mcuconf.h
#endif

#if FF_FS_REENTRANT == 0
#warning "FF_FS_REENTRANT = 0 in ffconf.h DO NOT open close file during log"
#endif

#if SDLOG_WRITE_BUFFER_SIZE < 512
#error SDLOG_ALL_BUFFERS_SIZE / SDLOG_NUM_FILES cannot be < 512
#endif

#if (!(IS_POWER_OF_TWO (SDLOG_WRITE_BUFFER_SIZE)))
#error SDLOG_ALL_BUFFERS_SIZE / SDLOG_NUM_FILES should be a POWER OF 2
#endif

#ifdef SDLOG_NEED_QUEUE
#include "modules/loggers/sdlog_chibios/msg_queue.h"

#include "mcu_periph/ram_arch.h"


/*
  The buffers that do DMA are the caches (named buf) in the FIL and FATFS struct of fatfs library
  It's the only buffers that have to reside in DMA capable memory.

  The buffer associated with message queue, and the cache buffer for caching file write
  could reside in non DMA  capable memory.

  stm32f4 : regular sram : 128ko, dma,     slow
            ccm sram     :  64ko,  no_dma, fast

  stm32f7 : regular sram  : 256ko, dma only possible if data cache are explicitely flushed, fast
            dtcm sram     : 64ko, dma, slow (no cache)
 */



static msg_t   IN_STD_SECTION_CLEAR(queMbBuffer[SDLOG_QUEUE_BUCKETS]);
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
  uint32_t autoFlushPeriod;
  systime_t lastFlushTs;
  bool  inUse;
  bool  tagAtClose;
  // optimise write byte by caching at send level now that we are based upon tlsf where
  // allocation granularity is 16 bytes
  LogMessage *writeByteCache;
  uint8_t writeByteSeek;
};

static  struct FilePoolUnit IN_DMA_SECTION(fileDes[SDLOG_NUM_FILES]) = {
  [0 ... SDLOG_NUM_FILES - 1] = {
    .fil = {{0}}, .inUse = false, .tagAtClose = false,
    .writeByteCache = NULL, .writeByteSeek = 0
  }
};

static volatile size_t nbBytesWritten = 0;
static SdioError storageStatus = SDLOG_OK;

typedef enum {
  FCNTL_WRITE = 0b00,
  FCNTL_FLUSH = 0b01,
  FCNTL_CLOSE = 0b10,
  FCNTL_EXIT =  0b11
} FileFcntl;

struct  _SdLogBuffer {
  LogMessage *lm;
  size_t     len;
  uint32_t   offset;
} ;


#define LOG_MESSAGE_PREBUF_LEN (SDLOG_MAX_MESSAGE_LEN+sizeof(LogMessage))
#endif //  SDLOG_NEED_QUEUE

/* File system object */
static IN_DMA_SECTION(FATFS fatfs);

#ifdef SDLOG_NEED_QUEUE
static size_t logMessageLen(const LogMessage *lm);
static size_t logRawLen(const size_t len);
static SdioError sdLoglaunchThread(void);
static SdioError sdLogStopThread(void);
static thread_t *sdLogThd = NULL;
static SdioError  getNextFIL(FileDes *fd);
static void removeFromQueue(const size_t nbMsgToRFemov);
static void cleanQueue(const bool allQueue);
static SdioError sdLogExpandLogFile(const FileDes fileObject, const size_t sizeInMo,
				    const bool preallocate);

static void thdSdLog(void *arg) ;

#endif //  SDLOG_NEED_QUEUE

static int32_t uiGetIndexOfLogFile(const char *prefix, const char *fileName) ;
static inline SdioError flushWriteByteBuffer(const FileDes fd);

SdioError sdLogInit(uint32_t *freeSpaceInKo)
{
  DWORD clusters = 0;
  FATFS *fsp = NULL;
  nbBytesWritten = 0;

  // if init is already done, return ERROR
  if (sdLogThd != NULL) {
    *freeSpaceInKo = 0;
    return  storageStatus = SDLOG_WAS_LAUNCHED;
  }

#ifdef SDLOG_NEED_QUEUE
  msgqueue_init(&messagesQueue, &HEAP_DEFAULT, queMbBuffer, SDLOG_QUEUE_BUCKETS);
#endif

  if (!sdc_lld_is_card_inserted(NULL)) {
    return   storageStatus = SDLOG_NOCARD;
  }


  sdio_connect();
  chThdSleepMilliseconds(10);
  sdio_disconnect();

  if (sdio_connect() == FALSE) {
    return  storageStatus = SDLOG_NOCARD;
  }

#if FFCONF_DEF < 8000
  FRESULT rc = f_mount(0, &fatfs);
#else
  FRESULT rc = f_mount(&fatfs, "/", 1);
#endif

  if (rc != FR_OK) {
    return storageStatus = SDLOG_FATFS_ERROR;
  }

  if (freeSpaceInKo != NULL) {
    f_getfree("/", &clusters, &fsp);
    *freeSpaceInKo = clusters * (uint32_t)fatfs.csize / 2;
  }

#ifdef SDLOG_NEED_QUEUE
  for (uint8_t i = 0; i < SDLOG_NUM_FILES; i++) {
    fileDes[i].inUse = fileDes[i].tagAtClose = false;
    fileDes[i].writeByteCache = NULL;
    fileDes[i].writeByteSeek = 0;
  }

  storageStatus = sdLoglaunchThread();
#else
  storageStatus = SDLOG_OK;
#endif
  return storageStatus;

}


SdioError sdLogFinish(void)
{
#if FFCONF_DEF < 8000
  FRESULT rc = f_mount(0, NULL);
#else
  FRESULT rc = f_mount(NULL, "", 0);
#endif
  if (rc != FR_OK) {
    return storageStatus = SDLOG_FATFS_ERROR;
  }

  // if we mount, unmount, don't disconnect sdio
  /* if (sdio_disconnect () == FALSE) */
  /*   return  SDLOG_NOCARD; */

  return  storageStatus = SDLOG_OK;
}



#ifdef SDLOG_NEED_QUEUE
SdioError sdLogOpenLog(FileDes *fd, const char *directoryName, const char *prefix,
                       const uint32_t autoFlushPeriod, const bool appendTagAtClose,
		       const size_t sizeInMo, const bool preallocate, char *fileName, const size_t nameLength)
{
  FRESULT rc; /* fatfs result code */
  SdioError sde = SDLOG_OK; /* sdio result code */
  //DIR dir; /* Directory object */
  //FILINFO fno; /* File information object */

  /* local file descriptor
     using fd is a bad idea since fd is set before fatfs objets are coherents
     in a multithreaded application where sdLogXXX are done before sdLogWriteLog is done
     we can have a race condition. setting fd only when fatfs files are opened resolve the problem
   */
  FileDes ldf;

  sde = getNextFIL(&ldf);
  if (sde != SDLOG_OK) {
    return storageStatus = sde;
  }

  sde = getFileName(prefix, directoryName, fileName, nameLength, +1);
  if (sde != SDLOG_OK) {
    // sd card is not inserted, so logging task can be deleted
    return storageStatus = SDLOG_FATFS_ERROR;
  }


  rc = f_open(&fileDes[ldf].fil, fileName, FA_WRITE | FA_CREATE_ALWAYS);
  if (rc) {
    fileDes[ldf].inUse = false;
    return storageStatus = SDLOG_FATFS_ERROR;
  } else {
    fileDes[ldf].tagAtClose = appendTagAtClose;
    fileDes[ldf].autoFlushPeriod = autoFlushPeriod;
    fileDes[ldf].lastFlushTs = 0;
    sde = sdLogExpandLogFile(ldf, sizeInMo, preallocate);
  }
  
  *fd = ldf;
  return storageStatus = sde;
}

SdioError sdLogCloseAllLogs(bool flush)
{
  FRESULT rc = 0; /* Result code */
  
  //    do not flush what is in ram, close as soon as possible
  if (flush == false) {
    UINT bw;
    // stop worker thread then close file
    cleanQueue(true);
    sdLogStopThread();

    for (FileDes fd = 0; fd < SDLOG_NUM_FILES; fd++) {
      if (fileDes[fd].inUse) {
        FIL *fo = &fileDes[fd].fil;
        if (fileDes[fd].tagAtClose) {
          f_write(fo, "\r\nEND_OF_LOG\r\n", 14, &bw);
          nbBytesWritten += bw;
        }
        FRESULT trc = f_close(fo);
        fileDes[fd].inUse = false;
        if (!rc) {
          rc = trc;
        }
      }
    }

    if (rc) {
      return storageStatus = SDLOG_FATFS_ERROR;
    }

    // flush ram buffer then close
  } else { // flush == true
    if (sdLogThd == NULL) {
      // something goes wrong, log thread is no more working
      return storageStatus = SDLOG_NOTHREAD;
    }

    // queue flush + close order, then stop worker thread
    for (FileDes fd = 0; fd < SDLOG_NUM_FILES; fd++) {
      if (fileDes[fd].inUse) {
        flushWriteByteBuffer(fd);
        sdLogCloseLog(fd);
      }
    }

    LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
    if (lm == NULL) {
      return  storageStatus = SDLOG_MEMFULL;
    }

    lm->op.fcntl = FCNTL_EXIT;

    if (msgqueue_send(&messagesQueue, lm, sizeof(LogMessage), MsgQueue_REGULAR) < 0) {
      return storageStatus = SDLOG_QUEUEFULL;
    } else {
      chThdWait(sdLogThd);
      sdLogThd = NULL;
    }

  }
  return storageStatus = SDLOG_OK;
}

SdioError sdLogFlushAllLogs(void)
{
  SdioError status =  SDLOG_OK;
  if (sdLogThd == NULL) {
    // something goes wrong, log thread is no more working
    return storageStatus = SDLOG_NOTHREAD;
  }

  // queue flush + close order, then stop worker thread
  for (FileDes fd = 0; fd < SDLOG_NUM_FILES; fd++) {
    if (fileDes[fd].inUse) {
      status = sdLogFlushLog(fd);
      if (status != SDLOG_OK) {
        break;
      }
    }
  }

  return storageStatus = status;
}


#define FD_CHECK(fd)  if ((fd < 0) || (fd >= SDLOG_NUM_FILES) \
                          || (fileDes[fd].inUse == false))    \
                        return SDLOG_FATFS_ERROR


SdioError sdLogExpandLogFile(const FileDes fd, const size_t sizeInMo,
                             const bool preallocate)
{
  FD_CHECK(fd);

  // expand with opt=1 : pre allocate file now
  const FRESULT rc = f_expand(&fileDes[fd].fil, sizeInMo * 1024 * 1024, preallocate);
  return (rc == FR_OK) ? SDLOG_OK : SDLOG_CANNOT_EXPAND;
}

SdioError sdLogWriteLog(const FileDes fd, const char *fmt, ...)
{
  FD_CHECK(fd);

  const SdioError status = flushWriteByteBuffer(fd);
  storageStatus = status;
  if (status != SDLOG_OK) {
    return status;
  }

  va_list ap;
  va_start(ap, fmt);

  LogMessage *lm = tlsf_malloc_r(&HEAP_DEFAULT, LOG_MESSAGE_PREBUF_LEN);
  if (lm == NULL) {
    va_end(ap);
    return storageStatus = SDLOG_MEMFULL;
  }

  lm->op.fcntl = FCNTL_WRITE;
  lm->op.fd = fd & 0x1f;

  chvsnprintf(lm->mess, SDLOG_MAX_MESSAGE_LEN - 1,  fmt, ap);
  lm->mess[SDLOG_MAX_MESSAGE_LEN - 1] = 0;
  va_end(ap);

  const size_t msgLen =  logMessageLen(lm);
  lm = tlsf_realloc_r(&HEAP_DEFAULT, lm, msgLen);
  if (lm == NULL) {
    return storageStatus = SDLOG_MEMFULL;
  }

  if (msgqueue_send(&messagesQueue, lm, msgLen, MsgQueue_REGULAR) < 0) {
    return storageStatus = SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}

SdioError sdLogFlushLog(const FileDes fd)
{
  FD_CHECK(fd);

  const SdioError status = flushWriteByteBuffer(fd);
  storageStatus = status;
  if (status != SDLOG_OK) {
    return status;
  }

  // give room to send a flush order if the queue is full
  cleanQueue(false);

  LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
  if (lm == NULL) {
    return storageStatus = SDLOG_MEMFULL;
  }

  lm->op.fcntl = FCNTL_FLUSH;
  lm->op.fd = fd & 0x1f;

  if (msgqueue_send(&messagesQueue, lm, sizeof(LogMessage), MsgQueue_REGULAR) < 0) {
    return  storageStatus = SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}

SdioError sdLogCloseLog(const FileDes fd)
{
  FD_CHECK(fd);

  cleanQueue(false);
  LogMessage *lm =  tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage));
  if (lm == NULL) {
    return storageStatus = SDLOG_MEMFULL;
  }

  lm->op.fcntl = FCNTL_CLOSE;
  lm->op.fd = fd & 0x1f;

  if (msgqueue_send(&messagesQueue, lm, sizeof(LogMessage), MsgQueue_REGULAR) < 0) {
    return storageStatus = SDLOG_QUEUEFULL;
  }

  return storageStatus = SDLOG_OK;
}





static inline SdioError flushWriteByteBuffer(const FileDes fd)
{
  FD_CHECK(fd);

  if (unlikely(fileDes[fd].writeByteCache != NULL)) {
    if (msgqueue_send(&messagesQueue, fileDes[fd].writeByteCache,
                      sizeof(LogMessage) + fileDes[fd].writeByteSeek,
                      MsgQueue_REGULAR) < 0) {
      return storageStatus = SDLOG_QUEUEFULL;
    }
    fileDes[fd].writeByteCache = NULL;
  }
  return storageStatus = SDLOG_OK;
}

SdioError sdLogWriteRaw(const FileDes fd, const uint8_t *buffer, const size_t len)
{
  FD_CHECK(fd);

  const SdioError status = flushWriteByteBuffer(fd);
  storageStatus = status;
  if (status != SDLOG_OK) {
    return status;
  }

  LogMessage *lm = tlsf_malloc_r(&HEAP_DEFAULT, logRawLen(len));
  if (lm == NULL) {
    return storageStatus = SDLOG_MEMFULL;
  }

  lm->op.fcntl = FCNTL_WRITE;
  lm->op.fd = fd & 0x1f;
  memcpy(lm->mess, buffer, len);

  if (msgqueue_send(&messagesQueue, lm, logRawLen(len), MsgQueue_REGULAR) < 0) {
    return storageStatus = SDLOG_QUEUEFULL;
  }

  return SDLOG_OK;
}

SdioError sdLogAllocSDB(SdLogBuffer **sdb, const size_t len)
{
  *sdb = tlsf_malloc_r(&HEAP_DEFAULT, logRawLen(len));
  if (*sdb == NULL) {
    return storageStatus = SDLOG_MEMFULL;
  }

  LogMessage *lm = tlsf_malloc_r(&HEAP_DEFAULT, logRawLen(len));
  if (lm == NULL) {
    tlsf_free_r(&HEAP_DEFAULT, *sdb);
    return storageStatus = SDLOG_MEMFULL;
  }

  (*sdb)->lm = lm;
  (*sdb)->len = len;
  (*sdb)->offset = 0;
  return storageStatus = SDLOG_OK;
}

char *sdLogGetBufferFromSDB(SdLogBuffer *sdb)
{
  return  sdb->lm->mess + sdb->offset;
}

bool   sdLogSeekBufferFromSDB(SdLogBuffer *sdb, uint32_t offset)
{
  if ((sdb->offset + offset) < sdb->len) {
    sdb->offset += offset;
    return true;
  } else {
    return false;
  }
}

size_t sdLogGetBufferLenFromSDB(SdLogBuffer *sdb)
{
  return sdb->len - sdb->offset;
}

SdioError sdLogWriteSDB(const FileDes fd, SdLogBuffer *sdb)
{
  SdioError status = SDLOG_OK;

  if ((fd < 0) || (fd >= SDLOG_NUM_FILES) || (fileDes[fd].inUse == false)) {
    status = SDLOG_FATFS_ERROR;
    goto fail;
  }

  status = flushWriteByteBuffer(fd);
  if (status != SDLOG_OK) {
    goto fail;
  }


  sdb->lm->op.fcntl = FCNTL_WRITE;
  sdb->lm->op.fd = fd & 0x1f;

  if (msgqueue_send(&messagesQueue, sdb->lm, logRawLen(sdb->len), MsgQueue_REGULAR) < 0) {
    // msgqueue_send take care of freeing lm memory even in case of failure
    // just need to free sdb memory
    status = SDLOG_QUEUEFULL;
  }

  goto exit;

fail:
  tlsf_free_r(&HEAP_DEFAULT, sdb->lm);

exit:
  tlsf_free_r(&HEAP_DEFAULT, sdb);
  return storageStatus = status;
}




SdioError sdLogWriteByte(const FileDes fd, const uint8_t value)
{
  FD_CHECK(fd);
  LogMessage *lm;

  if (fileDes[fd].writeByteCache == NULL) {
    lm = tlsf_malloc_r(&HEAP_DEFAULT, sizeof(LogMessage) + WRITE_BYTE_CACHE_SIZE);
    if (lm == NULL) {
      return storageStatus = SDLOG_MEMFULL;
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
    const SdioError status = flushWriteByteBuffer(fd);
    // message is not sent so allocated buffer will not be released by receiver side
    // instead of freeing buffer, we just reset cache seek.
    if (status == SDLOG_QUEUEFULL) {
      fileDes[fd].writeByteSeek = 0;
      return storageStatus = SDLOG_QUEUEFULL;
    }
  }
  return  storageStatus = SDLOG_OK;
}

/*
  if fatfs use stack for working buffers, stack size should be reserved accordingly
 */
#define WA_LOG_BASE_SIZE 1024
#if FF_USE_LFN == 2
#if FF_FS_EXFAT
static IN_DMA_SECTION_NOINIT(THD_WORKING_AREA(waThdSdLog, WA_LOG_BASE_SIZE+((FF_MAX_LFN+1)*2)+(19*32)));
#else
static IN_DMA_SECTION_NOINIT(THD_WORKING_AREA(waThdSdLog, WA_LOG_BASE_SIZE+((FF_MAX_LFN+1)*2)));
#endif
#else
static THD_WORKING_AREA(waThdSdLog, WA_LOG_BASE_SIZE);
#endif

SdioError sdLoglaunchThread ()
{
  chThdSleepMilliseconds(100);

  sdLogThd = chThdCreateStatic(waThdSdLog, sizeof(waThdSdLog),
                               NORMALPRIO + 1, thdSdLog, NULL);
  if (sdLogThd == NULL) {
    return storageStatus = SDLOG_INTERNAL_ERROR;
  } else {
    return  storageStatus = SDLOG_OK;
  }
}

SdioError sdLogStopThread(void)
{
  SdioError retVal = SDLOG_OK;

  storageStatus = retVal;
  if (sdLogThd == NULL) {
    return storageStatus = SDLOG_NOTHREAD;
  }

  LogMessage lm;

  /* // ask for closing (after flushing) all opened files */
  /* for (uint8_t i=0; i<SDLOG_NUM_FILES; i++) { */
  /*   if (fileDes[i].inUse) { */
  /*     flushWriteByteBuffer (i); */
  /*     lm.op.fcntl = FCNTL_CLOSE; */
  /*     lm.op.fd = i & 0x1f; */
  /*     if (msgqueue_copy_send (&messagesQueue, &lm, sizeof(LogMessage), MsgQueue_OUT_OF_BAND) < 0) { */
  /*  retVal= SDLOG_QUEUEFULL; */
  /*     } */
  /*   } */
  /* } */

  lm.op.fcntl = FCNTL_EXIT;
  if (msgqueue_copy_send(&messagesQueue, &lm, sizeof(LogMessage), MsgQueue_OUT_OF_BAND) < 0) {
    retVal = SDLOG_QUEUEFULL;
  }

  chThdWait(sdLogThd);
  sdLogThd = NULL;
  return  storageStatus = retVal;
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


  const size_t directoryNameLen = MIN(strlen(directoryName), 128);
  const size_t slashDirNameLen = directoryNameLen + 2;
  char slashDirName[slashDirNameLen];
  strlcpy(slashDirName, "/", slashDirNameLen);
  strlcat(slashDirName, directoryName, slashDirNameLen);

  rc = f_opendir(&dir, directoryName);
  if (rc != FR_OK) {
    rc = f_mkdir(slashDirName);
    if (rc != FR_OK) {
      return storageStatus = SDLOG_FATFS_ERROR;
    }
    rc = f_opendir(&dir, directoryName);
    if (rc != FR_OK) {
      return storageStatus = SDLOG_FATFS_ERROR;
    }
  }

  for (;;) {
    rc = f_readdir(&dir, &fno); /* Read a directory item */
    if (rc != FR_OK || fno.fname[0] ==  0) { break; } /* Error or end of dir */


    if (fno.fname[0] == '.') { continue; }

    if (!(fno.fattrib & AM_DIR)) {
      //      DebugTrace ("fno.fsize=%d  fn=%s\n", fno.fsize, fn);
      fileIndex = uiGetIndexOfLogFile(prefix, fno.fname);
      maxCurrentIndex = MAX(maxCurrentIndex, fileIndex);
    }
  }
  if (rc) {
    return storageStatus = SDLOG_FATFS_ERROR;
  }

  rc = f_closedir(&dir);
  if (rc) {
    return  storageStatus = SDLOG_FATFS_ERROR;
  }

  if (maxCurrentIndex < NUMBERMAX) {
    chsnprintf(nextFileName, nameLength, NUMBERFMF,
               directoryName, prefix, maxCurrentIndex + indexOffset);
    return storageStatus = SDLOG_OK;
  } else {
    chsnprintf(nextFileName, nameLength, "%s\\%s%.ERR",
               directoryName, prefix);
    return storageStatus = SDLOG_LOGNUM_ERROR;
  }
}

SdioError removeEmptyLogs(const char *directoryName, const char *prefix, const size_t sizeConsideredEmpty)
{
  DIR dir; /* Directory object */
  FRESULT rc; /* Result code */
  FILINFO fno; /* File information object */


  rc = f_opendir(&dir, directoryName);
  if (rc != FR_OK) {
    return storageStatus = SDLOG_FATFS_NOENT;
  }

  for (;;) {
    rc = f_readdir(&dir, &fno); /* Read a directory item */
    if (rc != FR_OK || fno.fname[0] ==  0) { break; } /* Error or end of dir */

    if (fno.fname[0] == '.') { continue; }

    if (!(fno.fattrib & AM_DIR)) {
      //      DebugTrace ("fno.fsize=%d  fn=%s\n", fno.fsize, fn);
      if ((strncmp(fno.fname, prefix, strlen(prefix)) == 0) && (fno.fsize <= sizeConsideredEmpty)) {
        char absPathName[128];
        strlcpy(absPathName, directoryName, sizeof(absPathName));
        strlcat(absPathName, "/", sizeof(absPathName));
        strlcat(absPathName, fno.fname, sizeof(absPathName));
        rc = f_unlink(absPathName);
        if (rc) {
          break;
        }
      }
    }
  }

  if (rc) {
    return storageStatus = SDLOG_FATFS_ERROR;
  }

  rc = f_closedir(&dir);
  if (rc) {
    return storageStatus = SDLOG_FATFS_ERROR;
  }

  return  storageStatus = SDLOG_OK;
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
static void cleanQueue(const bool allQueue)
{
  if (allQueue == false) {
    do {
      struct tlsf_stat_t stat;
      tlsf_stat_r(&HEAP_DEFAULT, &stat);
      const size_t freeRam = stat.mfree;
      chSysLock();
      const bool queue_full = (chMBGetFreeCountI(&messagesQueue.mb) <= 0);
      chSysUnlock();
      //    DebugTrace ("sdLogCloseLog freeRam=%d queue_full=%d", freeRam, queue_full);
      if ((freeRam < 200) || (queue_full == true)) {
        removeFromQueue(1);
      } else {
        break;
      }
    } while (true);
  } else {
    removeFromQueue(SDLOG_QUEUE_BUCKETS);
  }
}

static void removeFromQueue(const size_t nbMsgToRFemove)
{
  /* struct tlsf_stat_t stat; */

  /* tlsf_stat_r (&HEAP_DEFAULT, &stat); */
  /* size_t freeRam = stat.mfree; */
  /* chSysLock(); */
  /* size_t queueBuckets = chMBGetFreeCountI(&messagesQueue.mb); */
  /* chSysUnlock(); */

  /* DebugTrace ("Before removeFromQueue (%d) : ram=%d buck=%d", nbMsgToRFemove, freeRam, queueBuckets); */

  LogMessage *lm = NULL;
  for (size_t i = 0; i < nbMsgToRFemove; i++) {
    const int32_t retLen = (int32_t)(msgqueue_pop_timeout(&messagesQueue, (void **) &lm, TIME_IMMEDIATE));
    if (retLen < 0) {
      break;
    }
    tlsf_free_r(&HEAP_DEFAULT, lm);
  }

  /* tlsf_stat_r (&HEAP_DEFAULT, &stat); */
  /* freeRam = stat.mfree; */
  /* chSysLock(); */
  /* queueBuckets = chMBGetFreeCountI(&messagesQueue.mb); */
  /* chSysUnlock(); */

  /* DebugTrace ("After removeFromQueue (%d) : ram=%d buck=%d", nbMsgToRFemove, freeRam, queueBuckets); */
}


static void thdSdLog(void *arg)
{
  (void) arg;
  struct PerfBuffer {
    // each element of buffer should be word aligned for sdio efficient write
    ALIGNED_VAR(4) uint8_t buffer[SDLOG_WRITE_BUFFER_SIZE] ;
    uint16_t size;
  } ;

  UINT bw;
  static IN_DMA_SECTION_CLEAR(struct PerfBuffer perfBuffers[SDLOG_NUM_FILES]);
  storageStatus = SDLOG_OK;
  chRegSetThreadName("thdSdLog");
  while (true) {
    LogMessage *lm = NULL;
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
              nbBytesWritten += bw;
              perfBuffers[lm->op.fd].size = 0;
            }
            if (lm->op.fcntl ==  FCNTL_FLUSH) {
              f_sync(fo);
            } else { // close
              if (fileDes[lm->op.fd].tagAtClose) {
                f_write(fo, "\r\nEND_OF_LOG\r\n", 14, &bw);
                nbBytesWritten += bw;
              }
              f_close(fo);
              fileDes[lm->op.fd].inUse = false; // store that file is closed
            }
          }
        }
        break;

        case FCNTL_EXIT:
          tlsf_free_r(&HEAP_DEFAULT, lm); // to avoid a memory leak
          chThdExit(storageStatus = SDLOG_NOTHREAD);
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
              nbBytesWritten += bw;
              // if there an autoflush period specified, flush to the mass storage media
              // if timer has expired and rearm.
              if (fileDes[lm->op.fd].autoFlushPeriod) {
                const systime_t now = chVTGetSystemTimeX();
                if ((now - fileDes[lm->op.fd].lastFlushTs) >
                    (fileDes[lm->op.fd].autoFlushPeriod * CH_CFG_ST_FREQUENCY)) {
                  f_sync(fo);
                  fileDes[lm->op.fd].lastFlushTs = now;
                }
              }
              if (rc) {
                //chThdExit(storageStatus = SDLOG_FATFS_ERROR);
		storageStatus = SDLOG_FATFS_ERROR;
              } else if (bw != SDLOG_WRITE_BUFFER_SIZE) {
                chThdExit(storageStatus = SDLOG_FSFULL);
              }

              memcpy(perfBuffer, &(lm->mess[stayLen]), (uint32_t)(messLen - stayLen));
              perfBuffers[lm->op.fd].size = (uint16_t)(messLen - stayLen); // curBufFill
            }
          }
        }
      }
      tlsf_free_r(&HEAP_DEFAULT, lm);
    } else {
      chThdExit(storageStatus = SDLOG_INTERNAL_ERROR);
    }
  }
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
  for (FileDes i = 0; i < SDLOG_NUM_FILES; i++) {
    if (fileDes[i].inUse ==  false) {
      *fd = i;
      fileDes[i].inUse = true;
      return SDLOG_OK;
    }
  }
  return SDLOG_FDFULL;
}

size_t sdLogGetNbBytesWrittenToStorage(void)
{
  return nbBytesWritten;
}

SdioError sdLogGetStorageStatus(void)
{
  return storageStatus;
}


#endif
