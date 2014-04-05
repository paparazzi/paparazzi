/*
 * Copyright (C) 2014
 *
 * Created by: Savas Sen - ENAC UAV Lab
 *
 * This file is part of paparazzi..
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
 *
 */

/*
 * Server application for remote devices such as Android.
 *
 * It forwards paparazzi ground class messages to remote app
 * and receives commands from allowed clients to control the aircraft
 *
 * Several clients can be connected at the same time with full or restricted access
 */

#include <glib.h>
#include <gio/gio.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <libxml/xmlreader.h>

// assuming local path from sw/ground_segment/tmtc
char defaultPprzFolder[] = "../../..";

char defaultAppPass[] = "1234"; //4 char password to control ac's over app "pass ground stg stg stg..
char* AppPass;

#define BUFLEN 2048
#define MAXCLIENT 20
#define MAXIPLEN 50
#define MAXNAMELENGTH 500
#define MAXDEVICENUMB 255

#define MAXWPNUMB 50  //NUMBER OF WP PER AC (MAX)
#define MAXWPNUMB 50  //NUMBER OF WP PER AC (MAX)
#define MAXWPNAMELEN 50  //WP NAME LENGTH (MAX)

// Default TCP port (listen clients commands)
int tcp_port = 5010;
// Default UDP port (broadcast data to clients)
int udp_port = 5005;

#ifdef __APPLE__
char defaultIvyBus[] = "224.255.255.255:2010";
#else
char defaultIvyBus[] = "127.255.255.255:2010";
#endif
char* IvyBus;

char ivybuffer[BUFLEN];

// verbose flag
int verbose = 0;

//Block structure
typedef struct {
  //Will we need any more block data?
  char Bl_Name[MAXWPNAMELEN];
} bl_data;

//Waypoint structure
typedef struct {
  //Will we need any more waypoint data?
  char Wp_Name[MAXWPNAMELEN];
} wp_data;

//Aircraft Data Structure
typedef struct {
  int device_id;
  char name[MAXNAMELENGTH];
  char type[MAXNAMELENGTH];
  char color[MAXNAMELENGTH];
  char flight_plan_path[MAXNAMELENGTH];
  char airframe_path[MAXNAMELENGTH];
  char settings_path[MAXNAMELENGTH];
  int dl_launch_ind;
  int kill_thr_ind;
  wp_data AcWp[MAXWPNUMB];
  bl_data AcBl[MAXWPNUMB];
} device_names;

device_names DevNames[MAXDEVICENUMB];

//Connected Clients Data Structure
typedef struct {
  int used ;
  int client_port; //do we need that??
  char client_ip[MAXIPLEN];
} client_data;

client_data ConnectedClients[MAXCLIENT];  //Holds all status of devices


//Remove Client from UdpBroadcast list
void remove_client(char* RemClientIpAd) {
  //remove client from client list
  int i=0;
  for (; i < MAXCLIENT; i++) {
    //Search thru the client list

    if ( (ConnectedClients[i].used > 0) ) {
      //Client list is being used
      if ( g_strcmp0(RemClientIpAd , ConnectedClients[i].client_ip) == 0  ) {
        //record found clean it!!
        ConnectedClients[i].client_ip[0]='\0';
        ConnectedClients[i].used=0;
        if (verbose) {
          printf("App Server: Client removed from client list %s\n", RemClientIpAd);
          fflush(stdout);
        }
        return;
      }
    }
  }
  //no record found!
  if (verbose) {
    printf("App Server: No record found to be removed from client list\n");
    fflush(stdout);
  }
}

//Record client (if new)
void add_client(char* ClientIpAd) {
  /* Check if client exists. If exists return else record it */
  int i;
  for (i = 0; i < MAXCLIENT; i++) {
    //Search thru the client list
    if ( (ConnectedClients[i].used > 0) ) {
      //Client list is being used
      if ( g_strcmp0(ClientIpAd , ConnectedClients[i].client_ip) == 0  ) {
        //Already in client list return
        return;
      }
      continue;
    }
    //no record found!

    //record new client ip
    g_stpcpy(ConnectedClients[i].client_ip,ClientIpAd);
    //
    ConnectedClients[i].used = 1;
    if (verbose) {
      printf("App Server: New client added to client list %s\n", ConnectedClients[i].client_ip);
      fflush(stdout);
    }
    break;
  }
}

//Get aircraft name and color
int get_ac_data(char* InStr, char* RetBuf) {
  const char delimiters[] = " ";
  int  AcID;
  char StrBuf[BUFLEN];

  //Make writable copy
  strcpy(StrBuf, InStr);
  strtok(StrBuf, delimiters);

  //Get id from command string
  AcID = atoi(strtok(NULL, delimiters));
  //Get & create return string
  if ( AcID > 0 ) {
    //Dont search it, it is thereeee :)
    sprintf(RetBuf, "AppServer ACd %d %s %s %s %d %d\n", AcID,
        DevNames[AcID].name,
        DevNames[AcID].type,
        DevNames[AcID].color,
        DevNames[AcID].dl_launch_ind,
        DevNames[AcID].kill_thr_ind);
  }
  return AcID;
}

//Get aircraft name waypoint names,ids
int get_wp_data(char* InStr, char* RetBuf) {
  //Input command
  const char delimiters[] = " ";
  int  AcID;
  char StrBuf[BUFLEN];

  //Make writable copy
  strcpy(StrBuf, InStr);
  strtok(StrBuf, delimiters);

  //Get id from command string
  AcID = atoi(strtok(NULL, delimiters));
  //Get & create return string
  if ( AcID > 0 ) {
    //create wp data string
    char WpStr[BUFLEN] = "";
    int i = 1;
    while ( strlen(DevNames[AcID].AcWp[i].Wp_Name)  > 0 ) {
      //Read & add wp data to return string
      sprintf(WpStr, "%s%d,%s ", WpStr, i, DevNames[AcID].AcWp[i].Wp_Name);
      i++;
    }
    sprintf(RetBuf, "AppServer WPs %d %s\n", AcID, WpStr);
  }
  return AcID;
}

//Get block names,ids
int get_bl_data(char* InStr, char* RetBuf) {
  //Input command
  const char delimiters[] = " ";
  int  AcID;
  char StrBuf[BUFLEN];

  //Make writable copy
  strcpy(StrBuf, InStr);
  strtok(StrBuf, delimiters);

  //Get id from command string
  AcID = atoi(strtok (NULL, delimiters));
  //Get & create return string
  if ( AcID > 0 ) {
    //create wp data string
    char BlStr[BUFLEN] ="";
    int i=1;
    while ( strlen(DevNames[AcID].AcBl[i].Bl_Name)  > 0 ) {
      sprintf(BlStr, "%s</n>%s", BlStr, DevNames[AcID].AcBl[i].Bl_Name);
      i++;
    }
    sprintf(RetBuf, "AppServer BLs %d %s\n", AcID, BlStr);
  }
  return AcID;
}

//Bfoadcast ivy msgs to clients
void broadcast_to_clients () {

  int i;
  for (i = 0; i < MAXCLIENT; i++) {
    if (ConnectedClients[i].used > 0) {

      //Send data
      GInetAddress *udpAddress;
      GSocketAddress *udpSocketAddress;
      GSocket *udpSocket;

      udpAddress = g_inet_address_new_from_string(ConnectedClients[i].client_ip);

      udpSocketAddress = g_inet_socket_address_new(udpAddress, udp_port);

      udpSocket = g_socket_new(G_SOCKET_FAMILY_IPV4, G_SOCKET_TYPE_DATAGRAM, 17, NULL);

      if (g_socket_connect(udpSocket, udpSocketAddress, NULL, NULL) == FALSE) {
        printf("App Server: stg wrong with socket connect\n");
        fflush(stdout);
      }
      if (g_socket_send(udpSocket, ivybuffer, strlen(ivybuffer) , NULL, NULL) < 0) {
        printf("App Server: stg wrong with send func\n");
        fflush(stdout);
      }
      if (g_socket_close(udpSocket, NULL) == FALSE) {
        printf("App Server: stg wrong with socket close\n");
        fflush(stdout);
      }
      //Unref objects
      g_object_unref(udpAddress);
      g_object_unref(udpSocketAddress);
      g_object_unref(udpSocket);
    }
  }
}

//Read tcp requests of connected clients
gboolean network_read(GIOChannel *source, GIOCondition cond, gpointer data) {

  GString *s = g_string_new(NULL);
  GError *error = NULL;
  GIOStatus ret = g_io_channel_read_line_string(source, s, NULL, &error);

  if (ret == G_IO_STATUS_ERROR) {
    //unref connection
    g_object_unref (data);
    g_string_free (s,TRUE);
    return FALSE;
  }
  else{
    //Read request command
    gchar  *RecString;
    RecString=s->str;
    //check client password
    if ((strncmp(RecString, AppPass, strlen(AppPass))) == 0) {
      //Password ok can send command
      GString *incs = g_string_new(s->str);
      incs = g_string_erase(s,0,(strlen(AppPass)+1));
      IvySendMsg("%s",incs->str);
      if (verbose) {
        printf("App Server: Command passed to ivy.. %s\n",incs->str);
        fflush(stdout);
      }
    }
    //AC data request. (Ignore client password)
    else if ((strncmp(RecString, "getac ", strlen("getac "))) == 0) {
      //AC data request
      char AcData[BUFLEN];
      //Read ac data
      if (get_ac_data(RecString, AcData)) {
        //Send requested data to client
        GOutputStream * ostream = g_io_stream_get_output_stream (data);
        g_output_stream_write(ostream, AcData, strlen(AcData), NULL, &error);
      }
    }
    //Waypoint data request (Ignore client password)
    else if ((strncmp(RecString, "getwp ", strlen("getwp "))) == 0) {
      char AcData[BUFLEN];
      //Read wp data of ac
      if (get_wp_data(RecString, AcData)) {
        //Send requested data to client
        GOutputStream * ostream = g_io_stream_get_output_stream (data);
        g_output_stream_write(ostream, AcData, strlen(AcData), NULL, &error);
      }
    }
    //Waypoint data request (Ignore client password)
    else if ((strncmp(RecString, "getbl ", strlen("getbl "))) == 0) {
      char AcData[BUFLEN];
      //Read block data of AC
      if (get_bl_data(RecString, AcData)) {
        //Send requested data to client
        GOutputStream * ostream = g_io_stream_get_output_stream (data);
        g_output_stream_write(ostream, AcData, strlen(AcData), NULL, &error);
      }
    }

    //If client sends removeme command, remove it from broadcast list
    else if ((strncmp( RecString, "removeme", strlen("removeme"))) == 0) {
      GSocketAddress *sockaddr = g_socket_connection_get_remote_address(data, NULL);
      GInetAddress *addr = g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(sockaddr));
      //Read sender ip
      if (verbose) {
        printf("App Server: need to remove %s\n", g_inet_address_to_string(addr));
        fflush(stdout);
      }
      //Remove client
      remove_client(g_inet_address_to_string(addr));
      //Free objects
      g_string_free(s, TRUE);
      g_object_unref(sockaddr);
      g_object_unref(addr);
      g_object_unref(data);
      //Return false to stop listening this socket
      return FALSE;
    }
    else {
      //Unknown command
      if (verbose) {
        printf("App Server: Client send an unknown command: %s\n",RecString);
        fflush(stdout);
      }
    }
  }

  if (ret == G_IO_STATUS_EOF) {
    //Client disconnected
    if (verbose) {
      printf("App Server: Client disconnected without saying 'bye':(\n");
      fflush(stdout);
    }
    g_string_free(s, TRUE);
    //Unref the socket and return false to allow the client to reconnect
    g_object_unref(data);
    return FALSE;
  }
  //None above.. Keep listening the socket
  g_string_free(s, TRUE);
  return TRUE;
}

//New tcp conection
gboolean new_connection(GSocketService *service, GSocketConnection *connection, GObject *source_object, gpointer user_data) {

  //New client connected
  GSocketAddress *sockaddr = g_socket_connection_get_remote_address(connection, NULL);
  GInetAddress *addr = g_inet_socket_address_get_address(G_INET_SOCKET_ADDRESS(sockaddr));
  guint16 port = g_inet_socket_address_get_port(G_INET_SOCKET_ADDRESS(sockaddr));

  if (verbose) {
    printf("App Server: New Connection from %s:%d\n", g_inet_address_to_string(addr), port);
    fflush(stdout);
  }

  //Record client (if new)
  add_client(g_inet_address_to_string(addr));

  g_object_ref (connection);
  GSocket *socket = g_socket_connection_get_socket(connection);

  gint fd = g_socket_get_fd(socket);
  GIOChannel *channel = g_io_channel_unix_new(fd);
  g_io_add_watch(channel, G_IO_IN, (GIOFunc) network_read, connection);
  return TRUE;
}

//Ivy msg function
void Ivy_All_Msgs(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //Ivy msg received broadcast to clients..
  sprintf(ivybuffer, "%s", argv[0]);
  broadcast_to_clients();
}

//Parse AC flight plan xml (block & waypoint names
void parse_ac_fp(int DevNameIndex, char *filename) {
  xmlTextReaderPtr reader;
  int ret;

  reader = xmlReaderForFile(filename, NULL, XML_PARSE_NOWARNING | XML_PARSE_NOERROR); /* Dont validate with the DTD */

  xmlChar *name, *value;
  if (reader != NULL) {
    ret = xmlTextReaderRead(reader);

    int wp_queue=1;
    int bl_queue=1;
    while (ret == 1) {
      name = xmlTextReaderName(reader);
      if (name == NULL) {
        name = xmlStrdup(BAD_CAST "--");
      }

      //read waypoint names
      if (xmlStrEqual(name, (const xmlChar *)"waypoint")) {
        //waypoint node read name attr.
        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"name");
        value = xmlTextReaderValue(reader);
        //copy it to DevNames[] structure
        strcpy(DevNames[DevNameIndex].AcWp[wp_queue].Wp_Name, (char *) value);
        wp_queue++;
      }

      if (xmlStrEqual(name, (const xmlChar *)"block")) {
        //Read block names
        if ( xmlTextReaderAttributeCount(reader) >= 1 ) {
          xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"name");
          value = xmlTextReaderValue(reader);
          strcpy(DevNames[DevNameIndex].AcBl[bl_queue].Bl_Name, (char *) value);
          bl_queue++;
        }
      }

      ret = xmlTextReaderRead(reader);
    }

    xmlFreeTextReader(reader);
    if (ret != 0) {
      if (verbose) {
        printf("App Server: failed to parse %s\n", filename);
        fflush(stdout);
      }
    }
  }
  else
  {
    if (verbose) {
      printf("App Server: Unable to open %s\n", filename);
      fflush(stdout);
    }
  }

}

//check firmware type
//return true if "acceptable" type (e.g. fixedwing or rotorcraft)
int check_firmware_type (xmlChar* firmware) {
  return (xmlStrEqual(firmware, (const xmlChar *)"fixedwing")
      || xmlStrEqual(firmware,  (const xmlChar *)"rotorcraft"));
}

void parse_ac_af(int DevNameIndex, char *filename) {
  xmlTextReaderPtr reader;
  int ret;

  reader = xmlReaderForFile(filename, NULL, XML_PARSE_NOWARNING | XML_PARSE_NOERROR); /* Dont validate with the DTD */

  xmlChar *name, *value;
  if (reader != NULL) {
    ret = xmlTextReaderRead(reader);

    while (ret == 1) {
      name = xmlTextReaderName(reader);

      if (name == NULL) {
        name = xmlStrdup(BAD_CAST "--");
      }

      if (xmlStrEqual(name, (const xmlChar *)"firmware")) {
        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"name");
        value = xmlTextReaderValue(reader);
        //check if firmware name is accaptable
        if (check_firmware_type(value)>0) {
          strcpy(DevNames[DevNameIndex].type, (char *) value);
        }

      }

      if (xmlStrEqual(name, (const xmlChar *)"define")) {
        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"name");
        value = xmlTextReaderValue(reader);

        if (xmlStrEqual(value, (const xmlChar *)"AC_ICON")) {
          xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"value");
          value = xmlTextReaderValue(reader);
          strcpy(DevNames[DevNameIndex].type, (char *) value);
          return;
        }
      }

      ret = xmlTextReaderRead(reader);
    }

    xmlFreeTextReader(reader);
    if (ret != 0) {
      if (verbose) {
        printf("App Server: failed to parse %s\n", filename);
        fflush(stdout);
      }
    }
  } else {
    if (verbose) {
      printf("App Server: Unable to open %s\n", filename);
      fflush(stdout);
    }
  }

}

//Parse dl values
void parse_dl_settings(int DevNameIndex, char *filename) {

  xmlTextReaderPtr reader;
  int ret;

  reader = xmlReaderForFile(filename, NULL, XML_PARSE_NOWARNING | XML_PARSE_NOERROR); /* Dont validate with the DTD */

  xmlChar *name, *value;
  if (reader != NULL) {
    ret = xmlTextReaderRead(reader);

    int valind = 0;
    while (ret == 1) {
      name = xmlTextReaderName(reader);
      if (name == NULL) {
        name = xmlStrdup(BAD_CAST "--");
      }

      if (xmlStrEqual(name, (const xmlChar *)"dl_setting")) {
        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"var");

        value = xmlTextReaderValue(reader);
        if (xmlStrEqual(value, (const xmlChar *)"launch")) {
          DevNames[DevNameIndex].dl_launch_ind=valind;
        }

        if (xmlStrEqual(value, (const xmlChar *)"kill_throttle")) {
          DevNames[DevNameIndex].kill_thr_ind=valind;
        }
        xmlTextReaderNext(reader);
        valind+=1;
      }
      ret = xmlTextReaderRead(reader);
    }
    xmlFreeTextReader(reader);
    if (ret != 0) {
      if (verbose) {
        printf("App Server: %s : failed to parse\n", filename);
        fflush(stdout);
      }
    }
  }
  else {
    if (verbose) {
      printf("App Server: Unable to open %s\n", filename);
      fflush(stdout);
    }
  }
}

//Parse ac data from conf.xml
void parse_ac_data (char *PprzFolder) {
  xmlTextReaderPtr reader;
  int ret;

  //Create full file path
  char xmlFileName[BUFLEN];
  strcpy(xmlFileName, PprzFolder);
  strcat(xmlFileName, "/conf/conf.xml");

  reader = xmlReaderForFile(xmlFileName, NULL, XML_PARSE_NOWARNING | XML_PARSE_NOERROR); /* Dont validate with the DTD */

  xmlChar *AcName, *AcInd, *FpPath, *AcColor, *name, *AfPath;
  if (reader != NULL) {
    ret = xmlTextReaderRead(reader);
    int AcId;

    while (ret == 1) {
      name = xmlTextReaderName(reader);
      if (name == NULL) {
        name = xmlStrdup(BAD_CAST "--");
      }
      //read waypoint names

      if (xmlStrEqual(name, (const xmlChar *)"aircraft")) {

        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"ac_id");
        AcInd = xmlTextReaderValue(reader);

        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"name");
        AcName = xmlTextReaderValue(reader);

        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"airframe");
        AfPath = xmlTextReaderValue(reader);

        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"flight_plan");
        FpPath = xmlTextReaderValue(reader);

        xmlTextReaderMoveToAttribute(reader,(const xmlChar *)"gui_color");
        AcColor = xmlTextReaderValue(reader);

        //Get Device Id
        AcId = atoi(((char *) AcInd));

        //Save Device Name
        strcpy(DevNames[AcId].name, ((char *) AcName));

        //Save color
        strcpy(DevNames[AcId].color, (char *) AcColor);

        //Save Flight Plan Path
        strcpy(DevNames[AcId].flight_plan_path, "/conf/");
        strcat(DevNames[AcId].flight_plan_path , (char *) FpPath);

        //Save airframe  Path
        strcpy(DevNames[AcId].airframe_path, "/conf/");
        strcat(DevNames[AcId].airframe_path , (char *) AfPath);

        //Save Settings Path
        sprintf(DevNames[AcId].settings_path, "/var/%s/settings.xml", (char *) AcName);

        //parse flight plan file for waypoint and block names
        char FlightPlanPath[BUFLEN];
        strcpy(FlightPlanPath, PprzFolder);
        strcat(FlightPlanPath, DevNames[AcId].flight_plan_path);
        parse_ac_fp(AcId, FlightPlanPath);

        //parse airframe file
        char AirframePath[BUFLEN];
        strcpy(AirframePath, PprzFolder);
        strcat(AirframePath, DevNames[AcId].airframe_path);
        parse_ac_af(AcId, AirframePath);

        //parse dl_settings for launch & kill throttle
        char SettingsPath[BUFLEN];
        strcpy(SettingsPath, PprzFolder);
        strcat(SettingsPath, DevNames[AcId].settings_path);
        parse_dl_settings(AcId, SettingsPath);
      }

      ret = xmlTextReaderRead(reader);
    }

    xmlFreeTextReader(reader);
    if (ret != 0) {
      if (verbose) {
        printf("App Server: failed to parse %s\n", xmlFileName);
        fflush(stdout);
      }
    }
  }
  else{
    if (verbose) {
      printf("App Server: Unable to open %s\n", xmlFileName);
      fflush(stdout);
    }
  }

  return;
} // end of XMLParseDoc function

// Print help message
void print_help() {
  printf("Usage: app_server [options]\n");
  printf(" Options :\n");
  printf("   -t <TCP port>\tfor receiving devices commands (default: %d)\n", tcp_port);
  printf("   -u <UDP port>\tfor sending AC data (default: %d)\n", udp_port);
  printf("   -b <Ivy bus>\tdefault is %s\n", defaultIvyBus);
  printf("   -p <password>\tpassword for connection with control capabilities (default is %s)\n", defaultAppPass);
  printf("   -v\tverbose\n");
  printf("   -h --help show this help\n");
}

int main(int argc, char **argv) {

  // default password
  AppPass = defaultAppPass;

  // try environment variable first, set to default if failed
  IvyBus = getenv("IVYBUS");
  if (IvyBus == NULL) IvyBus = defaultIvyBus;

  // Look for paparazzi folder (PAPARAZZI_HOME or assume local path by default)
  char* PprzFolder = getenv("PAPARAZZI_HOME");
  if (PprzFolder == NULL) PprzFolder = defaultPprzFolder;

  // Parse options
  int i;
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help();
      exit(0);
    }
    else if (strcmp(argv[i], "-t") == 0) {
      tcp_port = atoi(argv[++i]);
    }
    else if (strcmp(argv[i], "-u") == 0) {
      udp_port = atoi(argv[++i]);
    }
    else if (strcmp(argv[i], "-b") == 0) {
      IvyBus = argv[++i];
    }
    else if (strcmp(argv[i], "-p") == 0) {
      AppPass = argv[++i];
    }
    else if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    }
    else {
      printf("App Server: Unknown option\n");
      print_help();
      exit(0);
    }
  }

  if (verbose) {
    printf("### Paparazzi App Server ###\n");
    printf("Using Paparazzi Folder      : %s\n", PprzFolder);
    printf("Server listen port (TCP)    : %d\n", tcp_port);
    printf("Server broadcast port (UDP) : %d\n", udp_port);
    printf("Control Pass                : %s\n", AppPass);
    printf("Ivy Bus                     : %s\n", IvyBus);
    fflush(stdout);
  }

  //Parse conf.xml
  parse_ac_data(PprzFolder);

  //Create tcp listener
#if !GLIB_CHECK_VERSION (2, 35, 1)
  // init GLib type system (only for older version)
  g_type_init();
#endif
  GSocketService *service = g_socket_service_new();

  GInetAddress *address = g_inet_address_new_any(G_SOCKET_FAMILY_IPV6); //G_SOCKET_FAMILY_IPV4 could be used
  GSocketAddress *socket_address = g_inet_socket_address_new(address, tcp_port);
  //Add listener
  g_socket_listener_add_address(G_SOCKET_LISTENER(service), socket_address, G_SOCKET_TYPE_STREAM,
      G_SOCKET_PROTOCOL_TCP, NULL, NULL, NULL);

  g_object_unref(socket_address);
  g_object_unref(address);
  g_socket_service_start(service);

  //Connect listening signal
  g_signal_connect(service, "incoming", G_CALLBACK(new_connection), NULL);

  //Here comes the ivy bindings
  IvyInit ("PPRZ_App_Server", "Papparazzi App Server Ready", NULL, NULL, NULL, NULL);

  IvyBindMsg(Ivy_All_Msgs, NULL, "(^ground .*)");
  IvyBindMsg(Ivy_All_Msgs, NULL, "(^\\S* AIRSPEED (\\S*) (\\S*) (\\S*) (\\S*))");
  IvyStart(IvyBus);

  GMainLoop *loop = g_main_loop_new(NULL, FALSE);

  if (verbose) {
    printf("Starting App Server\n");
    fflush(stdout);
  }

  g_main_loop_run(loop);

  if (verbose) {
    printf("Stoping App Server\n");
    fflush(stdout);
  }
  return 0;
}

