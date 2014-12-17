#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <sys/resource.h>


#include "onboard_transport.h"
#include "downlink_transport.h"

#define LOG_BUFLEN 512
#define PATH_LEN 256
#define FILENAME_LEN 64
#define TIMESTAMP_SCALE 10000

static void put_bytes(void *impl, enum DownlinkDataType data_type, uint8_t len __attribute__((unused)),
                      const void *bytes)
{
  struct onboard_transport *onboard = (struct onboard_transport *) impl;
  uint32_t length = 0;

  if (data_type == DL_TYPE_ARRAY_LENGTH) {
    onboard->array_length = *((const uint8_t *) bytes);
    return;
  }

  while (length++ <= onboard->array_length) {
    if (onboard->array_length > 0 && length > 1) {
      onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, ",");
    } else {
      onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, " ");
    }
    switch (data_type) {
      case DL_TYPE_UINT8:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%hhu",
                                        * (const uint8_t *)bytes);
        bytes = (const uint8_t *) bytes + 1;
        break;
      case DL_TYPE_UINT16:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%hu",
                                        * (const uint16_t *)bytes);
        bytes = (const uint16_t *) bytes + 2;
        break;
      case DL_TYPE_UINT32:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%u",
                                        * (const uint32_t *)bytes);
        bytes = (const uint32_t *) bytes + 4;
        break;
      case DL_TYPE_UINT64:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%llu",
                                        *(const uint64_t *)bytes);
        bytes = (const uint64_t *) bytes + 8;
        break;
      case DL_TYPE_INT8:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%hhi",
                                        * (const int8_t *)bytes);
        bytes = (const int8_t *) bytes + 1;
        break;
      case DL_TYPE_INT16:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%hi",
                                        * (const int16_t *)bytes);
        bytes = (const int16_t *) bytes + 2;
        break;
      case DL_TYPE_INT32:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%i",
                                        * (const int32_t *)bytes);
        bytes = (const int32_t *) bytes + 4;
        break;
      case DL_TYPE_INT64:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%lli",
                                        *(const int64_t *)bytes);
        bytes = (const int64_t *) bytes + 8;
        break;
      case DL_TYPE_FLOAT:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%#f",
                                        *(const float *)bytes);
        bytes = (const float *) bytes + 4;
        break;
      case DL_TYPE_DOUBLE:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "%#f",
                                        *(const double *)bytes);
        bytes = (const double *) bytes + 8;
        break;
      case DL_TYPE_TIMESTAMP:
        onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx,
                                        "%u.%04u", (*(const uint32_t *)bytes) / TIMESTAMP_SCALE, (*(const uint32_t *)bytes) % TIMESTAMP_SCALE);
        bytes = (const uint32_t *) bytes + 4;
        break;
      case DL_TYPE_ARRAY_LENGTH:
        break;
    }
  }
}

static void start_message(void *impl, char *name, uint8_t msg_id __attribute__((unused)),
                          uint8_t payload_len __attribute__((unused)))
{
  uint8_t ac_id = AC_ID;
  struct onboard_transport *onboard = (struct onboard_transport *) impl;
  onboard->buffer_idx = 0;
  onboard->array_length = 0;

  put_bytes(onboard, DL_TYPE_TIMESTAMP, 4, (uint8_t *) onboard->timestamp);
  put_bytes(onboard, DL_TYPE_UINT8, 1, (uint8_t *) &ac_id);
  onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, " %s",
                                  name);
}

static void end_message(void *impl)
{
  struct onboard_transport *onboard = (struct onboard_transport *) impl;
  onboard->buffer_idx += snprintf(onboard->buffer + onboard->buffer_idx, ONBOARD_BUFFER_LEN - onboard->buffer_idx, "\n");
  if (write(onboard->fd, onboard->buffer, onboard->buffer_idx) < 0) {
    onboard->overrun++;
  }
}

static void overrun(void *impl)
{
  struct onboard_transport *onboard = (struct onboard_transport *) impl;
  onboard->overrun++;
}

static void count_bytes(void *onboard __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{

}

static int check_free_space(void *onboard __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
  return TRUE;
}

static uint8_t size_of(void *onboard __attribute__((unused)), uint8_t len)
{
  return len + 2;
}

static int open_piped(char *filepath)
{
  int fd;
  int pipe_fd[2];
  int flags;
  ssize_t count;
  char buffer[LOG_BUFLEN];

  if (pipe(pipe_fd) == -1) {
    perror("onboard transport: pipe");
  }

  fd = open(filepath, O_CREAT | O_RDWR, 0644);
  if (fd < 0) {
    perror("onboard log: open");
  }

  if (fork() == 0) {
    // This is the child, close the write side of the pipe
    close(pipe_fd[1]);
    int retval;

    /* Lower our priority -- logging is not that important */
    if (setpriority(PRIO_PROCESS, 0, 10) < 0) {
      fprintf(stderr, "Couldn't renice logger for some reason!\n");
    }

    // copy from the read side of the pipe to the log
    while (1) {
      count = read(pipe_fd[0], buffer, LOG_BUFLEN);
      if (count < 0) {
        // error, presumably the pipe is closed
        break;
      }
      retval = write(fd, buffer, count);
    }
  } else {
    // This is the parent, close the read side of the pipe
    close(pipe_fd[0]);

    // Close the log file
    close(fd);

    // set non blocking on the write side of the pipe
    flags = fcntl(pipe_fd[1], F_GETFL);
    fcntl(pipe_fd[1], F_SETFL, flags | O_NONBLOCK);

    // return the write side of the pipe
    fd = pipe_fd[1];
  }

  return fd;
}

static void make_filename(char *filename)
{
  time_t t;
  t = time(NULL);
  struct tm *tmp;

  tmp = localtime(&t);
  if (tmp == NULL) {
    perror("localtime");
  }

  // format DM_HHMM_SS
  if (strftime(filename, FILENAME_LEN, "log_%d_%H%M_%S.data", tmp) == 0) {
    fprintf(stderr, "strftime returned 0");
  }
}

struct DownlinkTransport *onboard_transport_new(char *filepath, uint32_t *timestamp)
{
  struct DownlinkTransport *tp = calloc(1, sizeof(struct DownlinkTransport));
  struct onboard_transport *onboard = calloc(1, sizeof(struct onboard_transport));

  char full_filename[PATH_LEN];
  char filename[FILENAME_LEN];

  strncpy(full_filename, filepath, PATH_LEN);
  make_filename(filename);
  strncat(full_filename, filename, FILENAME_LEN);
  int fd = open_piped(full_filename);

  tp->impl = onboard;
  onboard->fd = fd;
  onboard->timestamp = timestamp;

  tp->StartMessage = start_message;
  tp->EndMessage = end_message;
  tp->PutBytes = put_bytes;

  tp->Overrun = overrun;
  tp->CountBytes = count_bytes;
  tp->SizeOf = size_of;
  tp->CheckFreeSpace = check_free_space;

  return tp;
}
