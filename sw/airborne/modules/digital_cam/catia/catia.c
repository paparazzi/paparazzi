#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include "serial.h"
#include "chdk_pipe.h"
#include "protocol.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 8
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
// Search&Rescue Onboard Detection Application
#define SODA "/root/develop/allthings_obc2014/src/soda/soda"

static void *handle_msg_shoot(void *ptr);
static inline void send_msg_image_buffer(void);
static inline void send_msg_status(void);

static volatile int is_shooting, image_idx, image_count, shooting_idx, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char *argv[])
{
  pthread_t shooting_threads[MAX_PROCESSING_THREADS];
  char c;
  int i;

  // Initialization
  printf("CATIA:\tStarting Camera Application Triggering Image Analysis\n");
  chdk_pipe_init();
  int ret = serial_init("/dev/ttySAC0");
  if (ret < 0) {
    printf("CATIA:\tfailed to open /dev/ttySAC0\n");
    return -1;
  }
  pthread_mutex_init(&mut, NULL);
  socket_init(1);

  // Initial settings
  is_shooting = 0;
  mora_protocol.status = 0;
  image_idx = 0;
  image_count = 0;
  shooting_idx = 0;
  shooting_count = 0;
  shooting_thread_count = 0;

  // MAIN loop
  while (1) {

    // Read the serial
    if (read(fd, &c, 1) > 0) {
      parse_mora(&mora_protocol, c);
    } else if (errno != 11) {
      printf("CATIA:\nSerial error: %d\n" , errno);
    }

    // Parse serial commands
    if (mora_protocol.msg_received) {
      // Process Only Once
      mora_protocol.msg_received = FALSE;

      // Shoot an image if not busy
      if (mora_protocol.msg_id == MORA_SHOOT) {
        // Parse the shoot message
        union dc_shot_union *shoot = (union dc_shot_union *) malloc(sizeof(union dc_shot_union));
        for (i = 0; i < MORA_SHOOT_MSG_SIZE; i++) {
          shoot->bin[i] = mora_protocol.payload[i];
        }
        printf("CATIA:\tSHOT %d,%d\n", shoot->data.nr, shoot->data.phi);

        pthread_create(&shooting_threads[(shooting_idx++ % MAX_PROCESSING_THREADS)], NULL, handle_msg_shoot, (void *)shoot);
        send_msg_status();
      }

      // Fill the image buffer (happens busy because needs fd anyway)
      if (mora_protocol.msg_id == MORA_BUFFER_EMPTY) {
        send_msg_image_buffer();
      }
    }

    // Read the socket
    if (socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if (image_count < MAX_IMAGE_BUFFERS) {
        image_count++;
      }
    }

  }

  // Close
  close(fd);
  chdk_pipe_deinit();

  printf("CATIA:\tShutdown\n");
  return 0;
}

static void *handle_msg_shoot(void *ptr)
{
  char filename[MAX_FILENAME], soda_call[512];
  union dc_shot_union *shoot = (union dc_shot_union *) ptr;

  // Test if can shoot
  pthread_mutex_lock(&mut);
  if (is_shooting) {
    pthread_mutex_unlock(&mut);
    printf("CATIA-%d:\tShooting: too fast\n", shoot->data.nr);

    free(shoot);
    return NULL;
  }

  is_shooting = 1;
  shooting_count++;
  shooting_thread_count++;
  pthread_mutex_unlock(&mut);

  printf("CATIA-%d:\tShooting: start\n", shoot->data.nr);
  chdk_pipe_shoot(filename);
  printf("CATIA-%d:\tShooting: got image %s\n", shoot->data.nr, filename);

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  pthread_mutex_unlock(&mut);

  //Parse the image
  sprintf(soda_call, "%s %s %d %d %d %d %d %d %d %d %d %d", SODA, filename,
          shoot->data.nr, shoot->data.lat, shoot->data.lon, shoot->data.alt, shoot->data.phi, shoot->data.theta, shoot->data.psi,
          shoot->data.vground, shoot->data.course, shoot->data.groundalt);
  printf("CATIA-%d:\tCalling '%s'\n", shoot->data.nr, soda_call);
  short int ret = system(soda_call);
  printf("CATIA-%d:\tShooting: soda return %d of image %s\n", shoot->data.nr, ret, filename);

  pthread_mutex_lock(&mut);
  shooting_thread_count--;
  pthread_mutex_unlock(&mut);

  free(shoot);
}

static inline void send_msg_image_buffer(void)
{
  int i;

  // Check if image is available
  if (image_count > 0) {
    printf("CATIA:\thandle_msg_buffer: Send %d\n", image_idx);
    // Send the image
    image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
    image_count--;

    MoraHeader(MORA_PAYLOAD, MORA_PAYLOAD_MSG_SIZE);
    for (i = 0; i < IMAGE_SIZE; i++) {
      MoraPutUint8(image_buffer[image_idx][i]);
    }
    MoraTrailer();
  }
}

static inline void send_msg_status(void)
{
  int i;
  struct mora_status_struct status_msg;
  char *buffer = (char *) &status_msg;

  pthread_mutex_lock(&mut);
  status_msg.cpu = 0;
  status_msg.threads = shooting_thread_count;
  status_msg.shots = shooting_count;
  status_msg.extra = 0;
  pthread_mutex_unlock(&mut);

  MoraHeader(MORA_STATUS, MORA_STATUS_MSG_SIZE);
  for (i = 0; i < MORA_STATUS_MSG_SIZE; i++) {
    MoraPutUint8(buffer[i]);
  }
  MoraTrailer();
}
