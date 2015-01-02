#ifndef OVERO_FILE_LOGGER_H
#define OVERO_FILE_LOGGER_H

#include <stdio.h>

struct FileLogger {
  FILE *outfile;
};

extern struct FileLogger file_logger;

extern void file_logger_init(char *filename);
extern void file_logger_periodic(void);
extern void file_logger_exit(void);

#endif /* OVERO_FILE_LOGGER_H */

