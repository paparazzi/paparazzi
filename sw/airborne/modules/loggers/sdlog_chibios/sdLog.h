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
 * @file modules/loggers/sdlog_chibios/sdLog.h
 * @brief sdlog API using ChibiOS and Fatfs
 *
 */

#pragma once
#include "std.h"
#include "mcuconf.h"

//#include "ff.h"
#include <stdarg.h>

#define NUMBERLEN 4
#define NUMBERMAX 9999
#define NUMBERFMF "%s\\%s%.04d.LOG"

#ifdef __cplusplus
extern "C" {
#endif

/*
  This module is highly coupled with fatfs, and mcuconf.h
  several MACRO should be defined before use

    FATFS (ffconf.h) :
    ° _FS_LOCK : number of simultaneously open file
    ° _FS_REENTRANT : If you need to open / close file during log, this should be set to 1 at
                       the expense of more used cam and cpu.
                      If you open all files prior to log data on them, it should be left to 0

 MCUCONF.H (or any other header included before sdLog.h
 ° SDLOG_ALL_BUFFERS_SIZE : (in bytes) cache buffer size shared between all opened log file
 ° SDLOG_MAX_MESSAGE_LEN  : (in bytes) maximum length of a message
 ° SDLOG_QUEUE_BUCKETS    : number of entries in queue

 use of the api :
 sdLogInit (initialize peripheral,  verify sdCard availibility)
 sdLogOpenLog : open file
 sdLogWriteXXX
 sdLogFlushLog : flush buffer (optional)
 sdLogCloseLog
 sdLogFinish

 and asynchronous emergency close (power outage detection by example) :
 sdLogCloseAllLogs
 sdLogFinish

 */


#if SDLOG_ALL_BUFFERS_SIZE == 0 || SDLOG_MAX_MESSAGE_LEN == 0 || \
    SDLOG_QUEUE_BUCKETS  == 0
#undef SDLOG_NEED_QUEUE
#else
#define SDLOG_NEED_QUEUE
#endif


#ifdef SDLOG_NEED_QUEUE
typedef struct LogMessage LogMessage;
#endif

typedef enum {
  SDLOG_OK,
  SDLOG_NOCARD,
  SDLOG_FATFS_ERROR,
  SDLOG_FATFS_NOENT,
  SDLOG_FSFULL,
  SDLOG_FDFULL,
  SDLOG_QUEUEFULL,
  SDLOG_NOTHREAD,
  SDLOG_INTERNAL_ERROR,
  SDLOG_LOGNUM_ERROR,
  SDLOG_WAS_LAUNCHED
} SdioError;

typedef struct _SdLogBuffer SdLogBuffer;
typedef int8_t FileDes;



/**
 * @brief initialise sdLog
 * @details init sdio peripheral, verify sdCard is inserted, check and mount filesystem,
 *          launch worker thread
 *          This function is available even without thread login facility : even
 *          if SDLOG_XXX macro are zeroed
 * @param[out]  freeSpaceInKo : if pointer in nonnull, return free space on filesystem
 * @return status (always check status)
 */
SdioError sdLogInit(uint32_t *freeSpaceInKo);

/**
 * @brief get last used name for a pattern, then add offset and return valid filename
 * @details for log file, you often have a pattern and a version. To retreive last
 *          file for reading, call function with indexOffset=0. To get next
 *          available file for writing, call function with indexOffset=1
 *          This function is available even without thread login facility : even
 *          if SDLOG_XXX macro are zeroed
 * @param[in] prefix : the pattern for the file : example LOG_
 * @param[in] directoryName : root directory where to find file
 * @param[out]  nextFileName : file with path ready to be used for f_open system call
 * @param[in] nameLength : length of previous buffer
 * @param[in] indexOffset : use 0 to retrieve last existent filename, 1 for next filename
 * @return status (always check status)
 */
SdioError getFileName(const char *prefix, const char *directoryName,
                      char *nextFileName, const size_t nameLength, const int indexOffset);




/**
 * @brief remove spurious log file left on sd
 * @details when tuning firmware, log files are created at each tries, and we consider
 *          that empty or nearly empty log are of no value
 *          this function remove log file whose size is less than a given value
 *
 * @param[in] prefix : the pattern for the file : example LOG_
 * @param[in] directoryName : root directory where to find file
 * @param[in] sizeConsideredEmpty : file whose size is less or equal to that value will be removed
 * @return status (always check status)
 */
SdioError removeEmptyLogs(const char *directoryName, const char *prefix,
                          const size_t sizeConsideredEmpty);
/**
 * @brief unmount filesystem
 * @details unmount filesystem, free sdio peripheral
 *          This function is available even without thread login facility : even
 *          if SDLOG_XXX macro are zeroed
 * @return status (always check status)
 */
SdioError sdLogFinish(void);


#ifdef SDLOG_NEED_QUEUE
/**
 * @brief open new log file
 * @details always open new file with numeric index
 * @param[out]  fileObject : file descriptor : small integer between 0 and _FS_REENTRANT-1
 * @param[in] directoryName : name of directory just under ROOT, created if nonexistant
 * @param[in] fileName : the name will be appended with 3 digits number
 * @param[in] appendTagAtClose : at close, a marker will be added to prove that the file is complete
 *            and not corrupt. useful for text logging purpose, but probably not wanted fort binary
 *            files.
 * @return status (always check status)
 */
SdioError sdLogOpenLog(FileDes *fileObject, const char *directoryName, const char *fileName,
                       bool appendTagAtClose);


/**
 * @brief flush ram buffer associated with file to sdCard
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @return status (always check status)
 */
SdioError sdLogFlushLog(const FileDes fileObject);


/**
 * @brief flush ram buffer then close file.
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @return status (always check status)
 */
SdioError sdLogCloseLog(const FileDes fileObject);

/**
 * @brief close all opened logs then stop worker thread
 * @param[in] flush : if true : flush all ram buffers before closing (take more time)
 *      if false : close imediatly files without flushing buffers,
 *                 more chance to keep filesystem integrity in case of
 *                 emergency close after power outage is detected
 * @return status (always check status)
 */
SdioError sdLogCloseAllLogs(bool flush);


/**
 * @brief log text
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @param[in] fmt : format and args in printf convention
 * @return status (always check status)
 */
SdioError sdLogWriteLog(const FileDes fileObject, const char *fmt, ...);


/**
 * @brief log binary data
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @param[in] buffer : memory pointer on buffer
 * @param[in] len : number of bytes to write
 * @return status (always check status)
 */
SdioError sdLogWriteRaw(const FileDes fileObject, const uint8_t *buffer, const size_t len);

/**
 * @brief log binary data limiting buffer copy by preallocating space
 * @param[in] len : number of bytes to write
 * @param[out] sdb : pointer to opaque object pointer containing buffer
 *                   there is two accessor functions (below) to access
 *                   buffer ptr and buffer len.
 * @details usage of the set of 4 functions :
 *          SdLogBuffer *sdb;
 *          sdLogAllocSDB (&sdb, 100);
 *          memcpy (getBufferFromSDB(sdb), SOURCE, offset);
 *          sdLogSeekBufferFromSDB (sdb, offset);
 *          sdLogWriteSDB (file, sdb);
 * @return status (always check status)
 */
SdioError sdLogAllocSDB(SdLogBuffer **sdb, const size_t len);

/**
 * @brief return a pointer of the writable area of a preallocated
 *        message + offset managed by sdLogSeekBufferFromSDB
 * @param[in] sdb : pointer to opaque object containing buffer
 *                  and previously filled by sdLogAllocSDB
 * @return pointer to writable area
 */
char *sdLogGetBufferFromSDB(SdLogBuffer *sdb);


/**
 * @brief manage internal offset in user buffer
 * @param[in] sdb : pointer to opaque object containing buffer
 *            and previously filled by sdLogAllocSDB
 *            offset : increment internal offset with this value
 * @return true if offset is withing internal buffer boundary
 *         false if offset is NOT withing internal buffer boundary, in this case,
 *         no keek is done
 */
bool sdLogSeekBufferFromSDB(SdLogBuffer *sdb, uint32_t offset);


/**
 * @brief return len of the writable area of a preallocated message (this take into account
 *        the offset)
 * @param[in] sdb : pointer to opaque object containing buffer
 *            and previously filled by sdLogAllocSDB
 * @return len of writable area
 */
size_t sdLogGetBufferLenFromSDB(SdLogBuffer *sdb);

/**
 * @brief send a preallocted message after it has been filled
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @param[in] sdb : pointer to opaque object containing buffer
 *                  and previously filled by sdLogAllocSDB
 * @return status (always check status)
*/
SdioError sdLogWriteSDB(const FileDes fd, SdLogBuffer *sdb);


/**
 * @brief log one byte of binary data
 * @param[in] fileObject : file descriptor returned by sdLogOpenLog
 * @param[in] value : byte to log
 * @return status (always check status)
 */
SdioError sdLogWriteByte(const FileDes fileObject, const uint8_t value);
#endif


#ifdef __cplusplus
}
#endif


