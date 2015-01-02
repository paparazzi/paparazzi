/* execill - How a parent and child might communicate. */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

#include "chdk_pipe.h"

#define READ 0
#define WRITE 1
#define MAX_FILENAME 255
#define SHELL "/root/develop/allthings_obc2014/src/popcorn/popcorn.sh"


const char *setup =
  "lua props=require(\"propcase\");print(\"SetupScript\");set_prop(props.ISO_MODE,3200);set_prop(props.FLASH_MODE,2);set_prop(props.RESOLUTION,0);set_prop(props.DATE_STAMP,0);set_prop(props.AF_ASSIST_BEAM,0);set_prop(props.QUALITY,0);print(\"Ready\");\n";

static int fo, fi;
static void wait_for_cmd(int timeout);
static void wait_for_img(char *filename, int timeout);
static pid_t popen2(const char *command, int *infp, int *outfp);

/*void main(int argc, char ** argv, char ** envp)
{
  int i;
  char filename[MAX_FILENAME];

  // Initialize chdk pipe
  chdk_pipe_init();

  // Start taking photos
  for(i=0; i < 3; i++) {
    chdk_pipe_shoot(filename);
    printf("Shot image: %s\n", filename);
  }

  // Initialize chdk pipe
  chdk_pipe_deinit();
}*/

/**
 * Initialize the CHDK pipe
 */
void chdk_pipe_init(void)
{
  /* Check if SHELL is started */
  if (popen2(SHELL, &fi, &fo) <= 0) {
    perror("Can't start SHELL");
    exit(1);
  }
  wait_for_cmd(10);

  /* Connect to the camera */
  write(fi, "connect\n", 8);
  wait_for_cmd(10);

  /* Kill all running scripts */
  //write(fi, "killscript\n", 11);
  //wait_for_cmd(10);

  /* Start recording mode */
  write(fi, "rec\n", 4);
  wait_for_cmd(10);

  /* Start rsint mode */
  write(fi, setup, strlen(setup));
  wait_for_cmd(strlen(setup));
}

/**
 * Deinitialize CHDK pipe
 */
void chdk_pipe_deinit(void)
{
  /* Stop rsint mode */
  //write(fi, "q\n", 2);
  //wait_for_cmd(10);

  /* Quit SHELL */
  write(fi, "quit\n", 3);
}

/**
 * Shoot an image
 */
void chdk_pipe_shoot(char *filename)
{
  write(fi, "rs /root\n", 9);
  wait_for_img(filename, 10);
}

/**
 * Wait for the image to be available
 * TODO: add timeout
 */
static void wait_for_img(char *filename, int timeout)
{
  int hash_cnt = 0;
  char ch;
  int filename_idx = 0;

  while (hash_cnt < 4) {
    if (read(fo, &ch, 1)) {
      if (ch == '#') {
        hash_cnt++;
      } else if (hash_cnt >= 2 && ch != '#') {
        filename[filename_idx++] = ch;
      }
    }
  }

  filename[filename_idx] = 0;
  wait_for_cmd(timeout);
}

/**
 * Wait for the commandline to be available
 * TODO: add timeout
 */
static void wait_for_cmd(int timeout)
{
  char ch;
  do {
    read(fo, &ch, 1);
  } while (ch != '>');
}

/**
 * Open a process with stdin and stdout
 */
static pid_t popen2(const char *command, int *infp, int *outfp)
{
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0) {
    return -1;
  }

  pid = fork();

  if (pid < 0) {
    return pid;
  } else if (pid == 0) {
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", command, NULL);
    perror("execl");
    exit(1);
  }

  if (infp == NULL) {
    close(p_stdin[WRITE]);
  } else {
    *infp = p_stdin[WRITE];
  }

  if (outfp == NULL) {
    close(p_stdout[READ]);
  } else {
    *outfp = p_stdout[READ];
  }

  return pid;
}
