
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "cmd_def.h"
#include "uart.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>

#include <pthread.h>
#include <signal.h>

int sock[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int addr_len, bytes_read, sin_size, bytes_recv;
unsigned char recv_data[1024], data_buf[8][1024], send_buf[8][1024];
struct sockaddr_in send_addr[8], rec_addr[8];
struct hostent *host;
unsigned int send_port , recv_port;

struct timeval tm;
double mytime = 0, old_time = 0;
int count[8] = {0, 0, 0, 0, 0, 0, 0, 0}, send_count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int rssi_count = 0;
int last0 = 0, last1 = 0;

int ac_id[8] = { -1, -1, -1, -1, -1, -1, -1, -1};

unsigned int  insert_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0},
                              extract_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0},
                                  send_insert_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0},
                                      send_extract_idx[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// #define DEBUG

#define CLARG_PORT 1
#define CLARG_ACTION 2
#define UART_TIMEOUT 1000

#define MAX_DEVICES 8
int connected_devices = 0;
//bd_addr found_devices[MAX_DEVICES];
int connected[] = {0, 0, 0, 0, 0, 0, 0, 0};

int connect_all = 0;
uint8 MAC_ADDR[] = {0x00, 0x00, 0x2d, 0x80, 0x07, 0x00};

// {0x00, 0x00, 0x1e, 0x80, 0x07, 0x00}; // begining of all dongle adresses

// {0x00,0x00,0x2d,0x80,0x07,0x00};   // begining of all modules adresses

enum actions {
  action_none,
  action_scan,
  action_connect,
  action_info,
  action_connect_all,
  action_broadcast
};
enum actions action = action_none;

typedef enum {
  state_disconnected,
  state_connecting,
  state_connected,
  state_finding_services,
  state_finding_attributes,
  state_listening_measurements,
  state_scanning,
  state_finish,
  state_last
} states;
states state = state_disconnected;

const char *state_names[state_last] = {
  "disconnected",
  "connecting",
  "connected",
  "finding_services",
  "finding_attributes",
  "listening_measurements",
  "scanning",
  "finish"
};

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff

#define DRONE_SERVICE_UUID            0x77cc
#define DRONE_DATA_UUID         0x9a66
#define DRONE_DATA_CONFIG_UUID        0x2902
#define DRONE_BROADCAST_UUID        0x25ec

uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 drone_handle_start = 0,
       drone_handle_end = 0,
       drone_handle_measurement = 0,
       drone_handle_configuration = 0,
       drone_handle_broadcast = 0;

bd_addr connect_addr;

void usage(char *exe)
{
  printf("Missing arguments, Possible call structure:\n");
  printf("[lists available com ports] %s list\n", exe);
  printf("[Connect direct to one drone]%s <COMx> <mac address> <recieve upd base port> <send udp base port>\n", exe);
  printf("[Scan and connect to all drones] %s <COMx> <scan> <recieve upd base port> <send udp base port>\n", exe);
}

void change_state(states new_state)
{
#ifdef DEBUG
  printf("DEBUG: State changed: %s --> %s\n", state_names[state], state_names[new_state]);
#endif
  state = new_state;
}

void *send_msg()
{
  printf("Bluegiga comms thread started\n");
  uint8_t device = 0, bt_msg_len = 0;
  uint16_t diff = 0;
  while (state != state_finish) {
    if (action == action_broadcast) {
      diff = (insert_idx[0] - extract_idx[0] + 1024) % 1024;
      if (diff) {
        bt_msg_len = diff < 31 ? diff : 31;
        //ble_cmd_attclient_attribute_write(device, drone_handle_measurement, bt_msg_len[device], &data_buf[device][extract_idx[device]]);

        ble_cmd_gap_set_adv_data(0, bt_msg_len, &data_buf[0][extract_idx[0]]);
        ble_cmd_gap_set_mode(gap_user_data, gap_non_connectable);
        extract_idx[device] = (extract_idx[device] + bt_msg_len) % 1024;

        usleep(2000); // advertisement interval set at 320ms so pause for shorter before turning off
        ble_cmd_gap_set_mode(0, 0);   // stop advertising
      }
    } else {
      device = 0;
      while (ac_id[device] != -1 && device < 8) {
        diff = (insert_idx[device] - extract_idx[device] + 1024) % 1024;
        if (diff) {
          bt_msg_len = diff < 18 ? diff : 18; //diff < 27 ? diff : 27;
          //ble_cmd_attclient_attribute_write(device, drone_handle_measurement, bt_msg_len[device], &data_buf[device][extract_idx[device]]);

          ble_cmd_attclient_write_command(device, drone_handle_measurement, bt_msg_len, &data_buf[device][extract_idx[device]]);
          extract_idx[device] = (extract_idx[device] + bt_msg_len) % 1024;
        }
        device++;
        usleep(10000);  // ~100Hz, max spi speed on lisa
      }
    }
  }
  pthread_exit(NULL);
}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return Zero if addresses are equal
 */
int cmp_bdaddr(bd_addr first, bd_addr second)
{
  int i;
  for (i = 0; i < sizeof(bd_addr); i++) {
    if (first.addr[i] != second.addr[i]) { return 1; }
  }
  return 0;
}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return returns how many bytes are the same
 */
int cmp_addr(const uint8 first[], const uint8 second[])
{
  int i;
  for (i = 5; i >= 0; i--) {
    if (first[i] != second[i]) { return 5 - i; }
  }
  return 6;
}

// print address, performs LSB to MSB
void print_bdaddr(bd_addr bdaddr)
{
  printf("%02x:%02x:%02x:%02x:%02x:%02x",
         bdaddr.addr[5],
         bdaddr.addr[4],
         bdaddr.addr[3],
         bdaddr.addr[2],
         bdaddr.addr[1],
         bdaddr.addr[0]);
}

// print raw incoming packet for degugging
void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
  printf("Incoming packet: ");
  int i;
  for (i = 0; i < sizeof(*hdr); i++) {
    printf("%02x ", ((unsigned char *)hdr)[i]);
  }
  for (i = 0; i < hdr->lolen; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

// function used to write to UART
void output(uint8 len1, uint8 *data1, uint16 len2, uint8 *data2)
{
  if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
    printf("ERROR: Writing to serial port failed\n");
    exit(1);
  }
}

// read and parse message from usb dongle over uart
void *read_message()
{
  printf("reading thread started\n");
  unsigned char data[256]; // enough for BLE
  struct ble_header hdr;
  int r;

  while (state != state_finish) {

    r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
    if (!r) {
      //printf("ERROR: UART timeout. Error code:%d\n", r);
      continue; // return NULL; // timeout
    } else if (r < 0) {
      printf("ERROR: Reading header failed. Error code:%d\n", r);
      pthread_exit(NULL);
    }

    if (hdr.lolen) {
      r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
      if (r <= 0) {
        printf("ERROR: Reading data failed. Error code:%d\n", r);
        pthread_exit(NULL);
      }
    }

    const struct ble_msg *msg = ble_get_msg_hdr(hdr);

#ifdef DEBUG
    print_raw_packet(&hdr, data);
#endif

    if (!msg) {
      printf("ERROR: Unknown message received\n");
      pthread_exit(NULL);
    }
    msg->handler(data);

    //usleep(500);
  }

  pthread_exit(NULL);

  //return 0;
}

void *send_paparazzi_comms()
{
  printf("send Comms started\n");
  unsigned char device = 0;
  unsigned int diff = 0;
  while (state != state_finish) {
    device = 0;
    while (ac_id[device] != -1 && device < 8) {
      diff = (send_insert_idx[device] - send_extract_idx[device] + 1024) % 1024;
      if (sock[device] && diff) {
        // printf("diff % d\n", diff);
        sendto(sock[device], &send_buf[device][send_extract_idx[device]], diff, MSG_WAITALL,
               (struct sockaddr *)&send_addr[device], sizeof(struct sockaddr));
        send_extract_idx[device] = (send_extract_idx[device] + diff) % 1024;
      }
      device++;
    }
    usleep(10000);
  }
  pthread_exit(NULL);
}

// enable indications or notifications
void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
  ble_cmd_sm_encrypt_start(connection_handle, 1);   // encrpyt connection (not required)

  uint8 configuration[] = {0x01, 0x00};       //  {0x01, 0x00} = notify, {0x02, 0x00} indicate
  ble_cmd_attclient_attribute_write(connection_handle, drone_handle_configuration, 2, &configuration);
}

// bt callback for get info request
void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
  printf("Build: %u, protocol_version: %u, hardware: ", msg->build, msg->protocol_version);
  switch (msg->hw) {
    case 0x01: printf("BLE112"); break;
    case 0x02: printf("BLED112"); break;
    default: printf("Unknown: %d", msg->hw);
  }
  printf("\n");

  if (action == action_info) { change_state(state_finish); }
}

// bt callback advertiser found
void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
  uint8_t i;
  char *name = NULL;

  // Check if this device already found, if not add to the list

  if (!connect_all) {
    printf("New device found: ");

    // Parse data
    for (i = 0; i < msg->data.len;) {
      int8 len = msg->data.data[i++];
      if (!len) { continue; }
      if (i + len > msg->data.len) { break; } // not enough data
      uint8 type = msg->data.data[i++];
      switch (type) {
        case 0x09:  // device name
          name = malloc(len);
          memcpy(name, msg->data.data + i, len - 1);
          name[len - 1] = '\0';
      }
      i += len - 1;
    }

    print_bdaddr(msg->sender);
    // printf(" RSSI:%d", msg->rssi);

    printf(" Name:");
    if (name) { printf("%s", name); }
    else { printf("Unknown"); }
    printf("\n");

    free(name);
  }

  // automatically connect if reponding device has appropriate mac address hearder
  if (connect_all && cmp_addr(msg->sender.addr, MAC_ADDR) >= 4) {
    printf("Trying to connect to "); print_bdaddr(msg->sender); printf("\n");
    //change_state(state_connecting);
    // connection interval unit 1.25ms
    // connection interval must be divisible by number of connection * 2.5ms and larger than minimum (7.5ms)

    ble_cmd_gap_connect_direct(&msg->sender.addr, gap_address_type_public, 6, 16, 100, 9);
  }
}

// bt connection status update
void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
  // updated connection
  if (msg->flags & connection_parameters_change) {
    printf("Connection %d parameters updated, interval %fms\n", msg->connection, msg->conn_interval * 1.25);
  }

  // Encrypted previous connection
  else if (msg->flags & connection_encrypted) {
    printf("Connection with %d is encrypted\n", msg->connection);
  }

  // Connection request completed
  else if (msg->flags & connection_completed) {
    if (msg->connection + 1 > connected_devices) { connected_devices++; }
    //change_state(state_connected);
    printf("Connected, nr: %d, connection interval: %d = %fms\n", msg->connection, msg->conn_interval,
           msg->conn_interval * 1.25);
    connected[msg->connection] = 1;

    if (rec_addr[msg->connection].sin_family != AF_INET && send_port && recv_port) {
      if ((sock[msg->connection] = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(1);
      }

      send_addr[msg->connection].sin_family = AF_INET;
      send_addr[msg->connection].sin_port = htons(send_port + msg->connection);
      send_addr[msg->connection].sin_addr = *((struct in_addr *)host->h_addr);
      bzero(&(send_addr[msg->connection].sin_zero), 8);
      sin_size = sizeof(struct sockaddr);

      rec_addr[msg->connection].sin_family = AF_INET;
      rec_addr[msg->connection].sin_port = htons(recv_port + msg->connection);
      rec_addr[msg->connection].sin_addr = *((struct in_addr *)host->h_addr);
      bzero(&(rec_addr[msg->connection].sin_zero), 8);
      sin_size = sizeof(struct sockaddr);

      if (bind(sock[msg->connection], (struct sockaddr *)&rec_addr[msg->connection],
               sizeof(struct sockaddr)) == -1) {
        perror("Bind failed");
        exit(1);
      }
      printf("Comms port opened on port: %d %d\n", send_port + msg->connection, recv_port + msg->connection);
    }

    // Handle for Drone Data configuration already known
    if (drone_handle_configuration) {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
      if (connect_all) {
        ble_cmd_gap_discover(gap_discover_generic);
      }
    }
    // Find primary services
    else {
      change_state(state_finding_services);
      ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);
    }
  }
}

// bt attribute group indentified
void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
  if (msg->uuid.len == 0) { return; }
  uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

  if (state == state_finding_services) {
    printf("length: %d uuid: %x\n", msg->uuid.len, uuid);
  }

  // First data service found
  if (state == state_finding_services && uuid == DRONE_SERVICE_UUID && drone_handle_start == 0) {
    drone_handle_start = msg->start;
    drone_handle_end = msg->end;
  }
}

// attribute procedure complete
// called when server completes client request
void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
  if (state == state_finding_services) {
    // Data service not found
    if (drone_handle_start == 0) {
      printf("No Drone service found\n");
      change_state(state_finish);
    }
    // Find drone service attributes
    else {
      change_state(state_finding_attributes);
      ble_cmd_attclient_find_information(msg->connection, drone_handle_start, drone_handle_end);
    }
  } else if (state == state_finding_attributes) {
    // Client characteristic configuration not found
    if (drone_handle_configuration == 0) {
      printf("No Client Characteristic Configuration found for Drone Data service\n");
      change_state(state_finish);
    }
    // Enable drone notifications
    else {
      change_state(state_listening_measurements);
      enable_indications(msg->connection, drone_handle_configuration);
      if (connect_all) {
        ble_cmd_gap_discover(gap_discover_generic);
      }
    }
  }

  // preivous message parsed on device, device now ready for next message
  //else if (state == state_listening_measurements) {
  //  extract_idx[msg->connection] = (extract_idx[msg->connection] + bt_msg_len[msg->connection]) % 1024;
  //  send_msg(msg->connection, 1);
  //}
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
//TODO: add different rule for 16
  if (msg->uuid.len == 2 || msg->uuid.len == 16) {
    uint32 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];
    if (uuid == DRONE_DATA_CONFIG_UUID) {
      drone_handle_configuration = msg->chrhandle;
    } else if (uuid == DRONE_DATA_UUID) {
      drone_handle_measurement = msg->chrhandle;
    } else if (uuid == DRONE_BROADCAST_UUID) {
      drone_handle_broadcast = msg->chrhandle;
    }
  }
}

void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
  /*
    gettimeofday(&tm, NULL); //Time zone struct is obsolete, hence NULL
    mytime = (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;

    count[msg->connection]++;
    if(mytime - old_time > 1){
      int i = 0;
      while(ac_id[i]!=-1 && i < 8){
        printf("Connection %d\nspeed: %dbps message frequency = %dHz\n",i,count[i]*20*8,count[i]);
        printf("send speed: %dbps buffer size: %d\n",send_count[i],(insert_idx[i] - extract_idx[i])%1024);
        printf("extract idx: %d, insert idx: %d\n",extract_idx[i], insert_idx[i]);

        send_count[i] = 0;
        count[i] = 0;
        i++;
      }
      old_time = mytime;
    }*/

  if (ac_id[msg->connection] == -1 && msg->value.data[0] == 0x99) {
    ac_id[msg->connection] = msg->value.data[2];
    printf("\nAC_ID = %d\n", ac_id[msg->connection]);
  }

  //memcpy(&send_buf[msg->connection][send_insert_idx[msg->connection]], msg->value.data, msg->value.len);
  //send_insert_idx[msg->connection] = (send_insert_idx[msg->connection] + msg->value.len)%1024;
  if (sock[msg->connection])
    sendto(sock[msg->connection], msg->value.data, msg->value.len, MSG_DONTWAIT,
           (struct sockaddr *)&send_addr[msg->connection], sizeof(struct sockaddr));

}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
  // reset connection parameters
  connected[msg->connection] = 0;

  extract_idx[msg->connection] = 0;
  insert_idx[msg->connection] = 0;

  // remove found device from list
  //change_state(state_disconnected);
  printf("Connection %d terminated\n" , msg->connection);
  if (!connect_all) {
    ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 6, 16, 100, 9);
  }
  //change_state(state_connecting);

  //change_state(state_finish);
}

void ble_rsp_connection_update(const struct ble_msg_connection_update_rsp_t *msg)
{
  printf("Connection update result from %d: %x\n", msg->connection, msg->result);
}

int kbhit(void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET(STDIN_FILENO, &rdfs);

  select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}

void ble_rsp_system_address_get(const struct ble_msg_system_address_get_rsp_t *msg)
{
  printf("My address: ");
  print_bdaddr(msg->address);
  printf("\n");
}

void *recv_paparazzi_comms()
{
  printf("Paparazzi comms thread started\n");
  unsigned char device = 0;
  while (state != state_finish) {
    if (state == state_listening_measurements) {
      device = 0;
      while (ac_id[device] != -1 && device < 8) {
        if (connected[device] && sock[device]) {
          bytes_recv = recvfrom(sock[device], recv_data, 1024, MSG_DONTWAIT, (struct sockaddr *)&rec_addr[device],
                                (socklen_t *)&sin_size);
          if (bytes_recv > 0) {
            send_count[device] += bytes_recv;
            //          for (i = 0; i<bytes_recv; i++)
            memcpy(&data_buf[device][insert_idx[device]], recv_data, bytes_recv);

            //send_msg(device, 0);
            insert_idx[device] = (insert_idx[device] + bytes_recv) % 1024;
          }
        }
        device++;
      }
    }
    usleep(5000);
  }
  pthread_exit(NULL);
}

void intHandler(int dummy)
{
  change_state(state_finish);
  usleep(5000);
  printf("event handled\n");
}

int main(int argc, char *argv[])
{
  // signal(SIGINT, intHandler);

  pthread_t threads[3];

  send_port = recv_port = 0;

  char *uart_port = "";
  gettimeofday(&tm, NULL);

  old_time = (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;

  //ble_cmd_sm_set_bondable_mode(1);

  host = (struct hostent *) gethostbyname((char *)"127.0.0.1");

  // Not enough command-line arguments
  if (argc <= CLARG_PORT) {
    usage(argv[0]);
    return 1;
  }

  // COM port argument
  if (argc > CLARG_PORT) {
    if (strcmp(argv[CLARG_PORT], "list") == 0) {
      uart_list_devices();
      return 1;
    } else if (strcmp(argv[CLARG_PORT], "-h") == 0 || strcmp(argv[CLARG_PORT], "--help") == 0) {
      usage(argv[0]);
    } else {
      uart_port = argv[CLARG_PORT];
    }
  }

  // Action argument
  if (argc > CLARG_ACTION) {
    int i;
    for (i = 0; i < strlen(argv[CLARG_ACTION]); i++) {
      argv[CLARG_ACTION][i] = tolower(argv[CLARG_ACTION][i]);
    }

    if (strcmp(argv[CLARG_ACTION], "scan") == 0) {
      action = action_scan;
    } else if (strcmp(argv[CLARG_ACTION], "info") == 0) {
      action = action_info;
    } else if (strcmp(argv[CLARG_ACTION], "broadcast") == 0) {
      action = action_broadcast;
      if (argc > CLARG_ACTION + 2) {
        send_port = atoi(argv[CLARG_ACTION + 1]);
        recv_port = atoi(argv[CLARG_ACTION + 2]);
      } else {
        usage(argv[0]);
        return 1;
      }
    } else if (strcmp(argv[CLARG_ACTION], "all") == 0) {
      connect_all = 1;
      action = action_scan;
      if (argc > CLARG_ACTION + 2) {
        send_port = atoi(argv[CLARG_ACTION + 1]);
        recv_port = atoi(argv[CLARG_ACTION + 2]);
      } else {
        usage(argv[0]);
        return 1;
      }
    } else {
      int i;
      short unsigned int addr[6];
      if (sscanf(argv[CLARG_ACTION],
                 "%02hx:%02hx:%02hx:%02hx:%02hx:%02hx",
                 &addr[5],
                 &addr[4],
                 &addr[3],
                 &addr[2],
                 &addr[1],
                 &addr[0]) == 6) {

        for (i = 0; i < 6; i++) {
          connect_addr.addr[i] = addr[i];
        }
        action = action_connect;
        if (argc > CLARG_ACTION + 2) {
          send_port = atoi(argv[CLARG_ACTION + 1]);
          recv_port = atoi(argv[CLARG_ACTION + 2]);
        } else {
          usage(argv[0]);
          return 1;
        }
      }
    }
  }
  if (action == action_none) {
    usage(argv[0]);
    return 1;
  }

  bglib_output = output;

  if (uart_open(uart_port)) {
    printf("ERROR: Unable to open serial port - %s\n", strerror(errno));
    return 1;
  }

  // Reset dongle to get it into known state
  ble_cmd_system_reset(0);
  uart_close();
  do {
    usleep(500000); // 0.5s
  } while (uart_open(uart_port));

  // start read thread
  pthread_create(&threads[0], NULL, read_message, NULL);

  // get the mac address of the dongle
  ble_cmd_system_address_get();

  // Execute action
  if (action == action_scan) {
    ble_cmd_gap_discover(gap_discover_generic);
  } else if (action == action_info) {
    ble_cmd_system_get_info();
  } else if (action == action_connect) {
    printf("Trying to connect\n");
    change_state(state_connecting);
    ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 8, 16, 100, 0);
  } else if (action == action_broadcast) {
    ble_cmd_gap_set_adv_parameters(0x200, 0x200,
                                   0x07);    // advertise interval scales 625us, min, max, channels (0x07 = 3, 0x03 = 2, 0x04 = 1)
  }

  pthread_create(&threads[1], NULL, send_msg, NULL);
  pthread_create(&threads[2], NULL, recv_paparazzi_comms, NULL);

  // Message loop
  while (state != state_finish) {
//    if (kbhit()) {
//      getchar();
//      break;
//    }
    usleep(1000);
  }

  change_state(state_finish);

  ble_cmd_gap_end_procedure();

  uart_close();

  pthread_exit(NULL);
}
