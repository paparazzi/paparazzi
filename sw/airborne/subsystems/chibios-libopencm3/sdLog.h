#ifndef __SD_LOG_H__
#define __SD_LOG_H__

#include "ff.h"
#include <stdarg.h>

typedef struct LogMessage LogMessage;

typedef enum {
  SDLOG_OK,
  SDLOG_NOCARD,
  SDLOG_FATFS_ERROR,
  SDLOG_FSFULL,
  SDLOG_QUEUEFULL,
  SDLOG_NOTHREAD,
  SDLOG_INTERNAL_ERROR
} SdioError;

SdioError sdLogInit (uint32_t* freeSpaceInKo);
SdioError sdLogFinish (void);
SdioError sdLogOpenLog (FIL *fileObject, const char* directoryName, const char* fileName);
SdioError sdLoglaunchThread (const bool_t binaryLog);
SdioError sdLogWriteLog (FIL *fileObject, const char* fmt, ...);
SdioError sdLogWriteRaw (FIL *fileObject, const uint8_t* buffer, const size_t len);
SdioError sdLogWriteByte (FIL *fileObject, const uint8_t value);
SdioError sdLogCloseLog (FIL *fileObject);
SdioError sdLogStopThread (void);


#endif // __SD_LOG_H__
