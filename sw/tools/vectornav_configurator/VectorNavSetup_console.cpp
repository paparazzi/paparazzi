/**
 * VectorNav configurator
 * Author: Michal Podhradsky, 2014, michal.podhradsky@aggiemail.usu.edu
 *
 * Note: it is a poorly written code and I am embarassed for it, but it was created in rush
 * so feel free to improve it.
 *
 * The configuration is recorded in this spreadsheet:
 * https://docs.google.com/spreadsheets/d/1JJIN4E16M2ii-TDnkjxinNSCmxCxwGGYOY_cep56z00/edit#gid=0
 */
#include <stdio.h>     // Standard input/output definitions
#include <string.h>    // String function definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>     // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <iostream>

using namespace std;

#define VN_BAUD1 B115200
#define VN_BAUD2 B230400
#define VN_BAUD3 B921600
#define VN_PORT "/dev/ttyUSB1"

#define DEBUG 0

// Serial port identifier
int baudrate = 0;
int new_baudrate = 0;
int msg_format = 0;
int port;
FILE * file;
//termios - structure contains options for port manipulation
struct termios specs; // for setting baud rate

int main ( int argc, char *argv[] )
{
  /**
   * Introduction
   */
  if (DEBUG) {
    cout << "WARNING! Running in DEBUG mode, no data will be written! "  << endl;
  }


  cout << "Starting configuration for VectorNav VN-200." << endl;

  cout << "Connect VN-200 to /dev/ttyUSB0 and pres enter..." << endl;
  cin.ignore();

  /**
   * Open serial port
   */
  cout << "Opening serial port..." << endl;
  //open the port and store the file descriptor in 'port'
  port = open(VN_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
  if (port == -1){
    // Could not open the port
    char p_err[64];
    sprintf(p_err, "open_port: Unable to open - %s", VN_PORT);
    perror(p_err);
    close(port);
    return -1;
  }else{
    fcntl(port, F_SETFL, 0); //leave this
    cout << "Port opened" << endl;
  }

  int set = 0;
  set = tcgetattr(port, &specs);

  //now the specs points to the opened port's specifications
  specs.c_cflag = (CLOCAL | CREAD ); //control flags

  /**
   * Set baudrate
   */
  cout << "What is current VN-200 baudrate?" << endl;
  cout << "1 - B115200" << endl;
  cout << "2 - B230400" << endl;
  cout << "3 - B921600" << endl;

  cin >> baudrate;
  cout << "The value you entered is ";

  switch (baudrate) {
    case 1:
      cout << "B115200" << endl;
      //set Baud Rate
      set = cfsetospeed(&specs,VN_BAUD1);
      break;
    case 2:
      cout << "B230400" << endl;
      //set Baud Rate
      set = cfsetospeed(&specs,VN_BAUD2);
      break;
    case 3:
      cout << "B921600" << endl;
      //set Baud Rate
      set = cfsetospeed(&specs,VN_BAUD3);
      break;
    default:
      cout << "Unknown baudrate!" << endl;
      close(port);
      return -1;
  }

  //our custom specifications set to the port
  //TCSANOW - constant that prompts the system to set
  //specifications immediately.
  set = tcsetattr(port,TCSANOW,&specs);

  /**
   * stops sending asynchronous messages on active port
   */
  char buf[] = "$VNASY,0*XX\r\n";
  cout << "Writing " << buf << " (stop async output)" << endl;
  int stat = write(port, buf, sizeof(buf));

  char res[64];
  if (!DEBUG){
    read(port, res, sizeof(res));
  }
  cout << "Response:" << res << endl;

  sleep(1);

  /**
   * Async Data Output Type Register - turn it off, port 1
   */
  char buf_asy1[] = "$VNWRG,06,0,1*XX\r\n";
  cout << "Writing " << buf_asy1 << " (stop async output on port 1)" << endl;
  stat = write(port, buf_asy1, sizeof(buf_asy1));

  char res_asy1[64];
  if (!DEBUG){
    read(port, res_asy1, sizeof(res_asy1));
  }
  cout << "Response:" << res_asy1 << endl;

  sleep(1);

  /**
   * Async Data Output Type Register - turn it off, port 2
   */
  char buf_asy2[] = "$VNWRG,06,0,2*XX\r\n";
  cout << "Writing " << buf_asy2 << " (stop async output on port 2)" << endl;
  stat = write(port, buf_asy2, sizeof(buf_asy2));

  char res_asy2[64];
  if (!DEBUG){
    read(port, res_asy2, sizeof(res_asy2));
  }
  cout << "Response:" << res_asy2 << endl;

  sleep(1);

  /**
   * Save to non-volatile memory
   */
  char cmd1[] = "$VNWNV*57\r\n";
  cout << "Writing " << cmd1 << " (save to non-volatile memory)" << endl;
  stat = write(port, cmd1, sizeof(cmd1));

  char res1[32];
  if (!DEBUG){
    stat = read(port, res1, sizeof(res1));
  }
  cout << "Response:" << res1 << endl;

  sleep(1);

  /**
   * Change baudrate if necessary
   */
  cout << "Do you want to change the baudrate settings? [y/n]";
  char bdchg;
  bool change_baudrate = 0;
  cin >> bdchg;

  if (bdchg == 'y') {
    change_baudrate = 1;
    cout << "The value you entered is YES";
  }
  else {
    cout << "The value you entered is NO";
  }

  /**
   * If we want to change baudrate, ask for the value
   */
  if (change_baudrate) {
    cout << "What is the desired VN-200 baudrate?" << endl;
    cout << "1 - B115200" << endl;
    cout << "2 - B230400" << endl;
    cout << "3 - B921600" << endl;

    cin >> new_baudrate;
    cout << "The value you entered is ";

    // command defines
    char cmd2a[] = "$VNWRG,05,115200,2*XX\r\n"; // set serial port 2 (TTL to new baud)
    char cmd3a[] = "$VNWRG,05,115200,1*XX\r\n"; // set serial port 1 (RS232 to new baud)
    char cmd2b[] = "$VNWRG,05,230400,2*XX\r\n"; // set serial port 2 (TTL to new baud)
    char cmd3b[] = "$VNWRG,05,230400,1*XX\r\n"; // set serial port 1 (RS232 to new baud)
    char cmd2c[] = "$VNWRG,05,921600,2*XX\r\n"; // set serial port 2 (TTL to new baud)
    char cmd3c[] = "$VNWRG,05,921600,1*XX\r\n"; // set serial port 1 (RS232 to new baud)

    switch (new_baudrate) {
      case 1:
        cout << "B115200" << endl;
        //set Baud Rate to B115200

        cout << "Writing " << cmd2a << " (set serial port 2 (TTL) to 115200)" << endl;
        stat = write(port, cmd2a, sizeof(cmd2a));
        char res2a[64];
        if (!DEBUG){
          stat = read(port, res2a, sizeof(res2a));
        }
        cout << "Response:" << res2a << endl;
        sleep(1);


        cout << "Writing " << cmd3a << " (set serial port 1 (RS232) to 115200)" << endl;
        stat = write(port, cmd3a, sizeof(cmd3a));

        char res3a[64];
        if (!DEBUG){
          stat = read(port, res3a, sizeof(res3a));
        }
        cout << "Response:" << res3a << endl;

        sleep(1);

        cout << "Changing port speed to B115200." << endl;

        break;
      case 2:
        cout << "B230400" << endl;
        //set Baud Rate to B230400

        cout << "Writing " << cmd2b << " (set serial port 2 (TTL) to 230400)" << endl;
        stat = write(port, cmd2b, sizeof(cmd2b));
        char res2b[64];
        if (!DEBUG){
          stat = read(port, res2b, sizeof(res2b));
        }
        cout << "Response:" << res2b << endl;
        sleep(1);


        cout << "Writing " << cmd3b << " (set serial port 1 (RS232) to 230400)" << endl;
        stat = write(port, cmd3b, sizeof(cmd3b));

        char res3b[64];
        if (!DEBUG){
          stat = read(port, res3b, sizeof(res3b));
        }
        cout << "Response:" << res3b << endl;

        sleep(1);

        cout << "Changing port speed to B230400." << endl;

        break;
      case 3:
        cout << "B921600" << endl;
        //set Baud Rate to B921600

        cout << "Writing " << cmd2c << " (set serial port 2 (TTL) to 921600)" << endl;
        stat = write(port, cmd2c, sizeof(cmd2c));
        char res2c[64];
        if (!DEBUG){
          stat = read(port, res2c, sizeof(res2c));
        }
        cout << "Response:" << res2c << endl;
        sleep(1);


        cout << "Writing " << cmd3c << " (set serial port 1 (RS232) to 921600)" << endl;
        stat = write(port, cmd3c, sizeof(cmd3c));

        char res3c[64];
        if (!DEBUG){
          stat = read(port, res3c, sizeof(res3c));
        }
        cout << "Response:" << res3c << endl;

        sleep(1);

        cout << "Changing port speed to B921600." << endl;

        break;
      default:
        cout << "Unknown baudrate!" << endl;
        close(port);
        return -1;
    }


    /**
     * Now we have to change the baudrate of our serial port
     * we rather close the port and open it all again, just to be sure
     */
    close(port);
    //open the port and store the file descriptor in 'port'
    port = open(VN_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (port == -1){
      // Could not open the port
      char p_err[64];
      sprintf(p_err, "open_port: Unable to open - %s", VN_PORT);
      perror(p_err);
      close(port);
      return -1;
    }else{
      fcntl(port, F_SETFL, 0); //leave this
      cout << "Port opened" << endl;
    }

    int set = 0;
    set = tcgetattr(port, &specs);
    //now the specs points to the opened port's specifications
    specs.c_cflag = (CLOCAL | CREAD ); //control flags

    switch (new_baudrate){
      case 1:
        set = cfsetospeed(&specs,VN_BAUD1);
        break;
      case 2:
        set = cfsetospeed(&specs,VN_BAUD2);
        break;
      case 3:
        set = cfsetospeed(&specs,VN_BAUD3);
        break;
      default:
        cout << "Unknown baudrate!" << endl;
        break;
    }

    //our custom specifications set to the port
    //TCSANOW - constant that prompts the system to set
    //specifications immediately.
    set = tcsetattr(port,TCSANOW,&specs);
  }

  /**
   * Save settings to non-volatile mememory
   */
  cout << "Writing " << cmd1 << " (save to non-volatile memory)" << endl;
  stat = write(port, cmd1, sizeof(cmd1));

  char res4[32];
  if (!DEBUG){
    stat = read(port, res4, sizeof(res4));
  }
  cout << "Response:" << res4 << endl;

  sleep(1);

  /**
   * Reseting VectorNav (to make sure stuff is saved)
   */
  char cmd4[] = "$VNRST*4D\r\n";
  cout << "Writing " << cmd4 << " (reset vector nav)" << endl;
  stat = write(port, cmd4, sizeof(cmd4));

  char res5[32];
  if (!DEBUG){
    stat = read(port, res5, sizeof(res5));
  }
  cout << "Response:" << res5 << endl;

  sleep(1);

  /**
   * Select message settings
   */
  cout << "Select message settings:" << endl;
  cout << "0 - Default [recommended]" << endl;
  cout << "1 - Flight" << endl;
  cout << "2 - SysId" << endl;
  cout << "3 - SysId Minimal" << endl;
  cout << "4 - SysId Current" << endl;


  cin >> msg_format;
  cout << "The value you entered is: ";

  std::string msg_f1;
  std::string msg_f2;

  switch (msg_format) {
    case 0:
      cout << "Default" << endl;
      msg_f1 = "$VNWRG,75,2,8,39,01EA,061A,0140,0009*XX\r\n"; // sets sending the messages at serial port 1 (RS232) @100Hz
      break;
    case 1:
      cout << "Flight" << endl;
      msg_f1 = "$VNWRG,75,1,16,9,01EA,061A*XX\r\n"; // sets sending the messages at serial port 1 (RS232) @100Hz
      break;
    case 2:
      cout << "SysId" << endl;
      msg_f1 = "$VNWRG,75,1,8,39,01EA,061A,0040,0008*XX\r\n"; // sets sending the messages at serial port 1 (RS232) @100Hz
      break;
    case 3:
      cout << "SysId Minimal" << endl;
      msg_f1 = "$VNWRG,75,1,8,31,01EA,0040,0008*XX\r\n"; // sets sending the messages at serial port 1 (RS232) @100Hz
      break;
    case 4:
      cout << "SysId Current" << endl;
      msg_f1 = "$VNWRG,75,3,2,35,0001,0600,0042,001A*XX\r\n"; // sets sending the messages at serial port 1 (RS232) @100Hz
      break;
    default:
      cout << "Unknown message format!" << endl;
      close(port);
      return -1;
  }

  const char *msg1 = msg_f1.c_str();
  int msg1_size = msg_f1.length() + 1;
  const char *msg2 = msg_f2.c_str();
  int msg2_size = msg_f2.length() + 1;

  /**
   * Settings for both ports
   */
  cout << "Writing " << msg1 << " (set msg format on port 1 and 2)" << endl;
  stat = write(port, msg1, (size_t)msg1_size);

  char res6[64];
  if (!DEBUG){
    stat = read(port, res6, sizeof(res6));
  }
  cout << "Response:" << res6 << endl;

  sleep(1);

  /**
   * Save settings to non-volatile mememory
   */
  cout << "Writing " << cmd1 << " (save to non-volatile memory)" << endl;
  stat = write(port, cmd1, sizeof(cmd1));

  char res8[32];
  if (!DEBUG){
    stat = read(port, res8, sizeof(res8));
  }
  cout << "Response:" << res8 << endl;

  /**
   * Reseting VectorNav (to make sure stuff is saved)
   */
  cout << "Writing " << cmd4 << " (reset vector nav)" << endl;
  stat = write(port, cmd4, sizeof(cmd4));

  char res9[32];
  if (!DEBUG){
    stat = read(port, res9, sizeof(res9));
  }
  cout << "Response:" << res9 << endl;

  cout << "Now it should be all set. To see if you are getting binary data, type either \"cat /dev/ttyUSB0\" or \"screen /dev/ttyUSB0 921600\" and you should see some bytes coming in." << endl;

  close(port);
  return 0;
}
