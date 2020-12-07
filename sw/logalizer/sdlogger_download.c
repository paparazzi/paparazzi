#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#ifdef __APPLE__
#include <libkern/OSByteOrder.h>
#define be32toh(x) OSSwapBigToHostInt32(x)
#else
#include <endian.h>
#endif

#define PRINT(string,...) fprintf(stderr, "[sdlogger_download->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#ifdef DEBUG
#define DEBUG_PRINT PRINT
#else
#define DEBUG_PRINT(...)
#endif

#define PPRZ_STX 0x99

/* File pointer for temp.tlm */
FILE *fp;

/* File pointer for serial link */
int fd = 0;

/* Aircraft ID */
int ac_id = 0;

/* Paths */
char sd2log[256];
char pycommand[256];

/* Setting associated with sdlogger_spi.command */
unsigned char setting = 0;

/* Global variable to check need for input */
static volatile bool need_input = true;

/* Global states enum and definition */
enum global_states {
  Idle,
  WaitingForIndexRequestConfirmation,
  ReadingIndexBlock,
  GotIndex,
  Downloading
};
enum global_states global_state = Idle;

/* Counter for reading bytes in the index block */
int index_cnt = 0;

/* PPRZ message parser states */
enum normal_parser_states {
  SearchingPPRZ_STX,
  ParsingLength,
  ParsingSenderId,
  SkippingTwoBytes,
  ParsingMsgId,
  ParsingMsgPayload,
  CheckingCRCA,
  CheckingCRCB
};

struct normal_parser_t {
  enum normal_parser_states state;
  unsigned char length;
  int counter;
  unsigned char sender_id;
  unsigned char msg_id;
  unsigned char payload[100];
  unsigned char crc_a;
  unsigned char crc_b;
};

struct normal_parser_t parser;

/* Struct and definition of log info (read from index block) */
struct log_info_t {
  uint32_t address;
  uint32_t length;
  unsigned int reserved;
};

struct log_index_t {
  uint32_t next_available_address;
  uint8_t last_completed_log;
  struct log_info_t logs[42];
} __attribute__((packed));

/* Log index initialization */
bool index_available = false;
struct log_index_t log_index;

/* Identifier of the log that is currently being downloaded */
uint8_t current_download = 0;


/* Function definitions */
void process_command(char *command);

void new_logfile(void)
{
  fp = fopen("temp.tlm", "w+");
}

void close_logfile(void)
{
  fclose(fp);
}


/* For ctrl+c */
static volatile int keep_running = 1;
static volatile int searching = 1;
void intHandler(int dummy) {
  printf("\n");
  keep_running = 0;
}

unsigned int get_baud(unsigned int baud_rate)
{
  unsigned int BAUD = 0;
  switch (baud_rate)
  {
#ifdef B921600
    case 921600:
      BAUD = B921600;
      break;
#endif
#ifdef B460800
    case 460800:
      BAUD = B460800;
      break;
#endif
    case 230400:
      BAUD = B230400;
      break;
    case 115200:
      BAUD = B115200;
      break;
    default:
      printf("Baud rate not recognized, using default B57600\n");
    case 57600:
      BAUD = B57600;
      break;
    case 38400:
      BAUD = B38400;
      break;
    case 19200:
      BAUD  = B19200;
      break;
    case 9600:
      BAUD  = B9600;
      break;
    case 4800:
      BAUD  = B4800;
      break;
    case 2400:
      BAUD  = B2400;
      break;
    case 1800:
      BAUD  = B1800;
      break;
    case 1200:
      BAUD  = B1200;
      break;
    case 600:
      BAUD  = B600;
      break;
    case 300:
      BAUD  = B300;
      break;
    case 200:
      BAUD  = B200;
      break;
    case 150:
      BAUD  = B150;
      break;
    case 134:
      BAUD  = B134;
      break;
    case 110:
      BAUD  = B110;
      break;
    case 75:
      BAUD  = B75;
      break;
    case 50:
      BAUD  = B50;
      break;
  }  //end of switch baud_rate
  return BAUD;
}

int serial_init(char *port_name, unsigned int baud)
{
  struct termios orig_termios, cur_termios;

  int br = get_baud(baud);

  fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1) {
    printf("opening modem serial device : fd < 0\n");
    return -1;
  }

  if (tcgetattr(fd, &orig_termios)) {
    printf("getting modem serial device attr\n");
    return -2;
  }
  cur_termios = orig_termios;

  /* input modes - turn off input processing */
  cur_termios.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR
                           | ICRNL | IXON | IXANY | IXOFF | IMAXBEL);
  /* pas IGNCR sinon il vire les 0x0D */
  cur_termios.c_iflag |= BRKINT;

  /* output_flags - turn off output processing */
  cur_termios.c_oflag  &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);

  /* control modes */
  cur_termios.c_cflag &= ~(CSIZE | CSTOPB | CREAD | PARENB | PARODD | HUPCL | CLOCAL);
  cur_termios.c_cflag |= CREAD | CS8 | CLOCAL;
  cur_termios.c_cflag &= ~(CRTSCTS);

  /* local modes */
  cur_termios.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | FLUSHO | PENDIN);
  cur_termios.c_lflag |= NOFLSH;

  if (cfsetspeed(&cur_termios, br)) {
    printf("setting modem serial device speed\n");
    return -3;
  }

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) {
    printf("setting modem serial device attr\n");
    return -4;
  }

  return 0;
}

void open_port(const char* device) {
  fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) {
    fprintf(stderr, "open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  // setup connection options
  struct termios options;

  // get the current options
  tcgetattr(fd, &options);

  // set local mode, enable receiver, set comm. options:
  // 8 data bits, no parity, 57600 Baud AND 2 STOP BITS @#!
  options.c_cflag = CLOCAL | CREAD | CS8 | B57600;// | CSTOPB;

  // write options back to port
  tcsetattr(fd, TCSANOW, &options);
}

void close_port(void)
{
  close(fd);
}

void write_command(float value)
{
  unsigned char msg[14];
  unsigned char crc_a = 0;
  unsigned char crc_b = 0;
  unsigned char length = 14;
  unsigned char *pc;
  pc = (unsigned char*)&value;

  msg[0]  = PPRZ_STX;
  msg[1]  = length;
  crc_a += length; crc_b += crc_a;

  msg[2]  = 0; // Sender ID
  crc_a += 0; crc_b += crc_a;

  msg[3] = 0xFF; // Receiver ID (broadcast)
  crc_a += 0xFF; crc_b += crc_a;

  msg[4] = 0x02; // Class/Component
  crc_a += 0x02; crc_b += crc_a;

  msg[5]  = 4; // MSG_ID = SETTING
  crc_a += 4; crc_b += crc_a;

  msg[6]  = setting; // setting index
  crc_a += setting; crc_b += crc_a;

  msg[7]  = ac_id; // AC_ID
  crc_a += ac_id; crc_b += crc_a;

  msg[8]  = pc[0]; // value
  crc_a += msg[8]; crc_b += crc_a;
  msg[9]  = pc[1]; // value
  crc_a += msg[9]; crc_b += crc_a;
  msg[10]  = pc[2]; // value
  crc_a += msg[10]; crc_b += crc_a;
  msg[11]  = pc[3]; // value
  crc_a += msg[11]; crc_b += crc_a;
  msg[12] = crc_a;
  msg[13] = crc_b;

  if (write(fd, msg, length) != length) {
    fprintf(stderr, "write_command: could not write %d bytes", length);
  }
}

/*
 * PPRZ-message: ABCxxxxxxxDE
    A PPRZ_STX (0x99)
    B LENGTH (A->E)
    C PPRZ_DATA
      0 SENDER_ID
      1 MSG_ID
      2 MSG_PAYLOAD
      . DATA (messages.xml)
    D PPRZ_CHECKSUM_A (sum[B->C])
    E PPRZ_CHECKSUM_B (sum[ck_a])
*/
void parse_single_byte(unsigned char byte)
{
  DEBUG_PRINT("parser.state %d\n", parser.state);
  switch (parser.state) {

    case SearchingPPRZ_STX:
      if (byte == PPRZ_STX) {
        DEBUG_PRINT("Got PPRZ_STX\n");
        parser.crc_a = 0;
        parser.crc_b = 0;
        parser.counter = 1;
        parser.state = ParsingLength;
      }
      break;

    case ParsingLength:
      parser.length = byte;
      parser.crc_a += byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingSenderId;
      break;

    case ParsingSenderId:
      parser.sender_id = byte;
      parser.crc_a += byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = SkippingTwoBytes;
      break;

    case SkippingTwoBytes:
      parser.crc_a += byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      if (parser.counter == 5) {
        parser.state = ParsingMsgId;
      }
      break;

    case ParsingMsgId:
      parser.msg_id = byte;
      parser.crc_a += byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      parser.state = ParsingMsgPayload;
      break;

    case ParsingMsgPayload:
      parser.payload[parser.counter-6] = byte;
      parser.crc_a += byte;
      parser.crc_b += parser.crc_a;
      parser.counter++;
      if (parser.counter == parser.length - 2) {
        parser.state = CheckingCRCA;
      }
      break;

    case CheckingCRCA:
      DEBUG_PRINT("CRCA: %d vs %d\n", byte, parser.crc_a);
      if (byte == parser.crc_a) {
        parser.state = CheckingCRCB;
      }
      else {
        parser.state = SearchingPPRZ_STX;
      }
      break;

    case CheckingCRCB:
      DEBUG_PRINT("CRCB: %d vs %d\n", byte, parser.crc_b);
      if (byte == parser.crc_b && parser.msg_id == 31) {
        DEBUG_PRINT("MSG ID: %d \t"
               "SENDER_ID: %d\t"
               "LEN: %d\t"
               "SETTING: %d\n",
               parser.msg_id,
               parser.sender_id,
               parser.length,
               parser.payload[0]);
        DEBUG_PRINT("Request confirmed\n");

        /* Check what to do next if the command was received */
        if (global_state == WaitingForIndexRequestConfirmation
            && parser.payload[0] == setting) {
          global_state = ReadingIndexBlock;
          index_cnt = 0;
        }
        if (global_state == GotIndex
            && parser.payload[0] == setting) {
          global_state = Downloading;
          new_logfile();
          index_cnt = 0;
        }
      }
      parser.state = SearchingPPRZ_STX;
      break;

    default:
      /* Should never get here */
      break;
  }

}

void parse_index_byte(unsigned char byte)
{
  DEBUG_PRINT("parse_index_byte %d/512: %x\n", index_cnt, byte);
  unsigned char *pc;
  pc = (unsigned char*)&log_index;
  pc[index_cnt] = byte;
  index_cnt++;
  if (index_cnt >= 512) {
    /* Plot results: */
    printf("\n\nNumber of Logs: %u\n", log_index.last_completed_log);
    printf("Log number \t Address \t Length [bytes]\n");
    for (uint8_t i = 0; i < log_index.last_completed_log; i++) {
      printf("%u \t\t 0x%08x \t %u \n",
             i+1,
             be32toh(log_index.logs[i].address),
             be32toh(log_index.logs[i].length)*512);
    }
    searching = false;
    global_state = GotIndex;
  }
}

void parse_download_byte(unsigned char byte)
{
  static long long dcnt = 0;
  dcnt++;
  DEBUG_PRINT("Got \t (%lld) \t 0x%02x\n", dcnt, byte);
  fputc(byte, fp);

  /* Show progress */
  if (dcnt % 512 == 0) {
    printf(".");
    fflush(stdout);
  }

  /* Show progress every 10% */
  if ( dcnt % (be32toh(log_index.logs[current_download-1].length)*512/10) == 0 ){
    int percent = (dcnt+1)*100/(be32toh(log_index.logs[current_download-1].length)*512);
    printf("%i%%", percent);
    fflush(stdout);
  }

  if (dcnt >= be32toh(log_index.logs[current_download-1].length)*512){
    /* Download finished */
    printf("\nDownloaded log %u\n", current_download);
    /* Close temp file */
    close_logfile();
    /* Process data into log format */
    if (system(sd2log) != 0) {
      fprintf(stderr, "Could not run sd2log to process data!\n");
    }
    /* Remove file */
    remove("temp.tlm");
    /* Reset and get ready for next command */
    need_input = true;
    dcnt = 0;
  }
}

void parse_bytes(const unsigned char *buff, int len)
{
  /* For each received byte call the appropriate function */
  for (int i = 0; i < len; i++) {
    switch (global_state) {

      /* Parse as pprz message */
      case Idle:
      case GotIndex:
      case WaitingForIndexRequestConfirmation:
        parse_single_byte(buff[i]);
        break;

      /* Read as index-block format */
      case ReadingIndexBlock:
        parse_index_byte(buff[i]);
        break;

      /* Download raw log data */
      case Downloading:
        parse_download_byte(buff[i]);
        break;

      default:
        break;
    }
  }
  return;
}

bool in_download_range(int download_id)
{
  if (download_id <= log_index.last_completed_log &&
      download_id > 0) {
    return true;
  } else {
    return false;
  }
}

void process_command(char *command)
{
  char *token;
  char *search = " ";
  strtok(command, "\n");

  token = strtok(command, search);

  if (strcmp(token, "download") == 0) {
    token = strtok(NULL, search);
    int download_id = atoi(token);
    if (in_download_range(download_id)) {
      printf("Downloading %i", download_id);
      write_command(download_id);
      current_download = download_id;
      global_state = GotIndex;
    }
    else {
      printf("Log ID %i outside available range\n", download_id);
      need_input = true;
    }
  }
  else if (strcmp(token, "exit") == 0) {
    keep_running = false;
  }
  else {
    printf("Possible commands:\n"
           "  download <lognumber>\n"
           "  help\n"
           "  exit\n");
    need_input = true;
  }
}

/* Main function */
int main ( int argc, char** argv)
{

  /* Default settings for serial connection */
  char *port = "/dev/ttyUSB0";
  int baud = 57600;

  /* Serial read buffer */
  int bytes;
  unsigned char buff[32];

  /* Parse arguments */
  int c;
  while ((c = getopt (argc, argv, "a:p:b:h")) != -1) {
    switch (c)
    {
      case 'a':
        ac_id = atoi(optarg);
        break;
      case 'p':
        port = optarg;
        break;
      case 'b':
        baud = atoi(optarg);
        break;
      case 'h':
      default:
        printf("usage: sdlogger_download [options]\n"
               "  options:\n"
               "    -a \tAircraft ID\n"
               "    -p \tPort (default: /dev/ttyUSB0).\n"
               "    -b \tBaudrate (default: 57600).\n"
               "    -h \tHelp, shows this message.\n");
        exit(0);
    }
  }

  /* Obtain sd2log directory */
  char *pprz_home;
  pprz_home = getenv( "PAPARAZZI_HOME" );

  char *sd2log_home = "/sw/logalizer/sd2log temp.tlm";

  // check if the pprz_home path fits into the sd2log buffer
  if (strlen(pprz_home) < (sizeof(sd2log) - strlen(sd2log_home))) {
    strcat(sd2log, pprz_home);
    strcat(sd2log, sd2log_home);
  }
  else {
    // pprz_home path too long for the buffer
    // exit to prevent buffer overflow
    printf("PPRZ_HOME path too long for the buffer, exiting...\n");
    exit(-1);
  }



  /* Get the setting ID with a python script */
  /* TODO: would be nicer to have a C xml parser */
  FILE *in = NULL;
  //strcat(pycommand, pprz_home);
  //strcat(pycommand, "/sw/logalizer/sdlogger_get_setting_id.py %u sdlogger_spi.command");
  char new_command[256];
  strcat(pycommand, "python sdlogger_get_setting_id.py %u sdlogger_spi.command");
  sprintf(new_command, pycommand, ac_id);
  strcpy(pycommand, new_command);

  char returnvalue[128];
  in = popen(pycommand, "r");
  if (fgets(returnvalue, 128, in) == NULL) {
    fprintf(stderr, "Aborting: Could not get id from sdlogger_get_setting_id.py\n");
    exit(0);
  }
  setting = atoi(returnvalue);
  pclose(in);
  if (setting == 0) {
    printf("Aborting: %s\n", returnvalue);
    exit(0);
  }

  /* Enable Ctrl+C */
  signal(SIGINT, intHandler);

  /* Open serial port */
  printf("Opening port %s with baudrate B%i\n", port, baud);
  if(serial_init(port, baud) < 0){
    return -1;
  }

  /* Keep polling for logger */
  printf("Searching for logger..");
  time_t counter = time(0);
  time_t timeout = 1; // every second
  while (keep_running && searching) {
    if (time(0) - counter >= timeout) {
      printf(".");
      fflush(stdout);
      write_command(255);
      global_state = WaitingForIndexRequestConfirmation;
      counter = time(0);
    }
    else {
      bytes = read(fd, (unsigned char*) buff, 32);
      parse_bytes(buff, bytes);
      usleep(5000);
    }
  }

  char command[128];
  strcpy(command, "help");
  /* Show available commands */
  printf("\n");
  process_command(command);


  while (keep_running) {
    if (need_input) {
      need_input = false;
      /* Test user input */
      printf("Command> ");
      if (fgets(command, 128, stdin) != NULL) {
        process_command(command);
      }
    }
    else {
      bytes = read(fd, (unsigned char*) buff, 32);
      parse_bytes(buff, bytes);
    }
  }

  printf("Closing app\n");

  /* Close serial port */
  close_port();

  return 0;
}

