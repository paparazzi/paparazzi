#include <signal.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>

#include <getopt.h>

#include <glib.h>

#include "wavecard.h"
#include "wavecard_glib.h"
#include "wavecard_glib_utils.h"


struct WcGlib wc_glib;
static void wc_glib_setup_sig_handler( void );
static void wc_glib_quit( int i );
static guint usage (int argc, char** argv);
static guint parse_options (int argc, char** argv);
static gboolean timeout_callback(gpointer data);
void request_remote_rssi ( void );
void request_self_address ( void );

#define NEXT_RSSI() {							\
    wc_glib.requested_remote_rssi++;					\
    wc_glib.requested_remote_rssi %= get_nb_hosts();			\
    if (wc_glib.requested_remote_rssi == wc_glib.self_id) {		\
      wc_glib.requested_remote_rssi++;					\
      wc_glib.requested_remote_rssi %= get_nb_hosts();			\
    }									\
  }

void on_wc_res_read_radio_param() {
  g_message("got WC_RES_READ_RADIO_PARAM");
  if (wc_payload[1] == 0) {
    switch (wc_glib.requested_radio_param) {
    case WC_RADIO_PARAM_AWAKENING_PERIOD :
      g_message("awakening period %d", wc_payload[2]);
      break;
    case WC_RADIO_PARAM_WAKE_UP_TYPE:
      g_message("wake_up_type %d", wc_payload[2]);
      break;
    case WC_RADIO_PARAM_RADIO_ACKNOLEDGE :
      g_message("radio_acknoledge %d", wc_payload[2]);
      break;
    case WC_RADIO_PARAM_WAKE_UP_LENGTH : {
      uint16_t* len = (uint16_t*)&wc_payload[2];
      g_message("wake_up_length %d", *len);
    }
      break;
    case WC_RADIO_PARAM_RADIO_ADDRESS : {
      memcpy(wc_glib.self_addr, wc_payload, WC_ADDR_LEN);
      wc_glib.status = WC_STATUS_GOT_SELFADDRESS;
      GString* str = g_string_new("self_address ");
      wc_glib_append_addr(str, &wc_payload[2]);
      g_string_append_printf(str, " %s ", get_host_by_addr(&wc_payload[2])->name);
      g_message(str->str);
      g_string_free(str, TRUE);
      request_remote_rssi();
    }
      break;
    case WC_RADIO_PARAM_SWITCH_MODE_STATUS:
      g_message("switch_mode_status %s", wc_payload[2]?"ON":"OFF");
      break;
    default:
      g_message("radio param %02x  %d", wc_glib.requested_radio_param, wc_payload[2]);
      break;
    }
  }
  else
    g_message("WC_RES_READ_RADIO_PARAM failed");
}
 
static void priv_parse_payload() {
  printf("got_message ");
  int i;
  for(i = 0; i < wc_length - 3; i++)
    printf("%02x ", wc_payload[i]);
  printf("\n"); 
  switch (wc_payload[0]) {
 
  case WC_RES_READ_RADIO_PARAM :
    on_wc_res_read_radio_param();
    g_source_remove(wc_glib.timeout_id);
    break;
  case WC_RES_WRITE_RADIO_PARAM :
    g_message("WC_RES_WRITE_RADIO_PARAM %s", wc_payload[1]?"FAILED":"OK");
    break;
  case WC_RES_SEND_SERVICE :
    g_message("got WC_RES_SEND_SERVICE");
    break;
  case  WC_SERVICE_RESPONSE :
    g_message("got WC_SERVICE_RESPONSE");
    printf("addr : "); wc_glib_print_addr(&wc_payload[1]);
    printf("\n");
    g_message("acquitement %d", wc_payload[7]);
    g_message("type %d", wc_payload[8]);
    g_message("rssi %d", wc_payload[9]);
    g_message("periode reveil %d", wc_payload[10]);
    g_message("type_2 %d", wc_payload[11]);
    break;

  case WC_RES_SEND_FRAME :
    g_message("got WC_RES_SEND_FRAME");
    break;

  case WC_RES_READ_REMOTE_RSSI : {
    g_source_remove(wc_glib.timeout_id);
    float percent = (float)wc_payload[1] / (float)0x2F * 100.;
    GTimeVal now;
    g_get_current_time(&now);
    g_message("%ld.%06ld got WC_RES_READ_REMOTE_RSSI %s %.1f", now.tv_sec, now.tv_usec, get_host_by_id(wc_glib.requested_remote_rssi)->name, percent ); 
    NEXT_RSSI();
    request_remote_rssi();
  }
    break;
    
  case WC_RES_READ_LOCAL_RSSI : 
    g_message("got WC_RES_READ_LOCAL_RSSI %d", wc_payload[1] );
    break;

  case WC_RECEIVED_FRAME :
    g_message("got WC_RECEIVED_FRAME");
    GString* str = g_string_new("sender_address ");
    wc_glib_append_addr(str, &wc_payload[1]);
    g_string_append_printf(str, "data : %*s ", wc_length - WC_ADDR_LEN, &wc_payload[1+WC_ADDR_LEN]);     
    g_message(str->str);
    g_string_free(str, TRUE);	
    break;

  case WC_ACK :
    g_message("got ACK");
    break;

  case WC_NAK :
    g_message("got NAK");
    break;

  default:
    g_message("received unhandled %02x", wc_payload[0]);
  }
}

static gboolean on_data_received(GIOChannel *source, GIOCondition condition, gpointer data) {
  guchar buf[128];
  gsize len;
  GError *err = NULL;
  g_io_channel_read_chars(source, buf, 3, &len, &err);
  int i;
  //  g_message("received");
  //  print_both(len, buf);
  for (i=0; i<len; i++)
    parse_wc(buf[i]);

  if (wc_msg_received) {
    parse_payload();
    priv_parse_payload();
  }
  return TRUE;
}

void request_self_address ( void ) {
  wc_glib.requested_radio_param = WC_RADIO_PARAM_RADIO_ADDRESS;
  WcSendReadRadioParamReq(WC_RADIO_PARAM_RADIO_ADDRESS);
  wc_glib.status = WC_STATUS_SELFADDRESS_REQUESTED;
  wc_glib.timeout_id = g_timeout_add(1000, timeout_callback, in_ch);
}

void request_remote_rssi ( void ) {
  WcSendReqReadRemoteRssi(get_host_by_id(wc_glib.requested_remote_rssi)->addr);
  wc_glib.status = WC_STATUS_REMOTE_RSSI_REQUESTED;
  wc_glib.timeout_id = g_timeout_add(4000, timeout_callback, in_ch);
}

uint8_t hello[] = "HELLO WORLD W";

static gboolean timeout_callback(gpointer data) {
  switch (wc_glib.status) {
  case WC_STATUS_SELFADDRESS_REQUESTED :
    g_message("local wavecard not responding");
    request_self_address();
    break;
  case WC_STATUS_REMOTE_RSSI_REQUESTED :
    g_message("distant wavecard not responding %s", get_host_by_id(wc_glib.requested_remote_rssi)->name);
    NEXT_RSSI();
    request_remote_rssi ();
    break;
  }
  
  return FALSE;

  static uint8_t foo1 = 1;
  if ( foo1 == 1) {
    wc_glib.requested_radio_param = WC_RADIO_PARAM_RADIO_ADDRESS;
    WcSendReadRadioParamReq(WC_RADIO_PARAM_RADIO_ADDRESS);
  }
  else if ( foo1 == 2) {
    wc_glib.requested_radio_param = WC_RADIO_PARAM_SWITCH_MODE_STATUS;
    //WcSendWriteRadioParamReq(WC_RADIO_PARAM_SWITCH_MODE_STATUS, 0);
    WcSendReadRadioParamReq(WC_RADIO_PARAM_SWITCH_MODE_STATUS);
  }
  else if ( foo1 == 3) {
    //wc_glib.requested_radio_param = WC_RADIO_PARAM_RADIO_ACKNOLEDGE;
    //WcSendWriteRadioParamReq(WC_RADIO_PARAM_RADIO_ACKNOLEDGE, 1);
    WcSendReadRadioParamReq(WC_RADIO_PARAM_RADIO_ACKNOLEDGE);
  }
  //  else if ( foo1 == 2) {
    // WcSendReqSelectChannel(0x01);
  //    WcSendReqReadChannel();
  //  }
  else if ( foo1 == 4) {
    //WcSendReqSelectPhyconfig(0x00a3);
    WcSendReqReadPhyconfig();
  }
  else if ( foo1 == 5) {
    wc_glib.requested_remote_rssi = 0;
    WcSendReqReadRemoteRssi(get_host_by_id(wc_glib.requested_remote_rssi)->addr);
  }
  //  else if (foo1 == 5) {
  //    WcSendReqReadLocalRssi(wc_glib.distant_addr);
  //  }
  //  else if (foo1 == 6) {
    //      WcSendReqSendService(wc_glib.distant_addr, 0x20);
  //  }
  // else if (foo1 == 7) {
  //    WcSendFirmwareReq();
  //  }
  else if (foo1 > 7) {
    static uint8_t foo = 1;
    hello[12] = foo++;
    //    if (!wc_glib.options.silent) WcSendMsg(wc_glib.distant_addr, strlen(hello), hello);
  } 
  foo1++;

  return TRUE;
}

int main (int argc, char** argv) {

  if (parse_options(argc, argv))
    return usage(argc, argv);

  wc_glib_setup_sig_handler();
  wc_glib.in_ch = open_serial_port(wc_glib.options.serial_device);
  in_ch = wc_glib.in_ch;
  g_io_add_watch (wc_glib.in_ch, G_IO_IN , on_data_received, NULL);
  //  g_timeout_add(2000, timeout_callback, in_ch);
  wc_glib.ml =  g_main_loop_new(NULL, FALSE);
  request_self_address();
  g_main_loop_run(wc_glib.ml);
  
  return 0;
}

static void wc_glib_setup_sig_handler( void ) {
  struct sigaction act;
  act.sa_handler = wc_glib_quit;
  act.sa_restorer = NULL;
  if (sigaction(SIGINT, &act, NULL) < 0)
    g_message("Could not install signal handler for quitting\n i will be unable to close log\n");
}

static void wc_glib_quit( int i ) {
  printf("\nquitting\n");
  g_main_loop_quit(wc_glib.ml);
}

static guint usage (int argc, char** argv) {
  g_message("%s [-h] [-d serial_device]", argv[0]);
  return -1;
}

static guint parse_options (int argc, char** argv) {
  const gchar *optstr = "h:d:s:p";
  gchar ch;
  wc_glib.options.serial_device = "/dev/ttyS0";
  wc_glib.options.silent = FALSE;
  while ((ch=getopt(argc,argv,optstr)) != -1) {
    switch(ch) {
    case 'h':
      return -1;
    case 'd':
      wc_glib.options.serial_device = strdup(optarg);
      break;
    case 's':
      wc_glib.options.silent = TRUE;
      break;
    case 'p':
      wc_glib.options.poll_mode = TRUE;
      break;
    case '?':
      printf("unrecognized option: %c\n",optopt);
      return -1;
    default:
      printf("error?  condition unaccounted for?\n");
      break;
    }
  }
  g_message("%s starting", argv[0]);
  g_message("  serial device : %s", wc_glib.options.serial_device);
  return 0;
}
