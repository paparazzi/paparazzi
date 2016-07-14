#include <iostream>
#include <csignal>
#include <unistd.h>
#include <stdlib.h>     /* exit, EXIT_FAILURE */
#include <chrono>
#include <thread>
#include <iostream>

extern "C" {
#include "nps_ivy.h"
}

#include <Ivy/ivy.h>
//#include <Ivy/ivyloop.h>

using namespace std;

#define HOST_TIMEOUT_MS 1000

std::chrono::milliseconds ms(HOST_TIMEOUT_MS);

int pauseSignal = 0;

void tstp_hdl(int n __attribute__((unused)))
{
  std::cout << "Signal called.\n";
  if (pauseSignal) {
    pauseSignal = 0;
    signal(SIGTSTP, SIG_DFL);
    raise(SIGTSTP);
  } else {
    pauseSignal = 1;
  }
}

void cont_hdl(int n __attribute__((unused)))
{
  signal(SIGCONT, cont_hdl);
  signal(SIGTSTP, tstp_hdl);
  std::cout << "Press <enter> to continue.\n";
}

void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

   exit(signum);

}

void nps_main_periodic(void) {
  cout << "Worker thread" << endl;

    if (pauseSignal) {

    cout <<  "Press <enter> to continue (or CTRL-Z to suspend)" << endl;

  string age;
  cin.clear();
  cin >> age;

    pauseSignal = 0;
  }
}


void run_behavior(void) {
  std::chrono::high_resolution_clock::time_point start;
  std::chrono::high_resolution_clock::time_point stop;
  std::chrono::duration<int32_t, std::nano> sleep_time;
  while(true)
  {
    start = std::chrono::high_resolution_clock::now();
    nps_main_periodic();
    stop = std::chrono::high_resolution_clock::now();
    sleep_time = ms - (stop - start);
    if(sleep_time > std::chrono::duration<int32_t,std::nano>(0))
    {
      std::this_thread::sleep_for(sleep_time);
    }
    else
    {
      std::cout << "We took too long\n";
    }
  }
}

void run_fg_thread(void){
  std::chrono::high_resolution_clock::time_point start;
  std::chrono::high_resolution_clock::time_point stop;
  std::chrono::duration<int32_t, std::nano> sleep_time;

  cout << "FG Thread init" << endl;

  while(true)
  {
    start = std::chrono::high_resolution_clock::now();
    cout << "FG Thread running" << endl;

    float x = 1.23;
    IvySendMsg("%d NPS_RATE_ATTITUDE %f %f %f %f %f %f",
               42,
               x,
               x,
               x,
               x,
               x,
               x);


    stop = std::chrono::high_resolution_clock::now();
    sleep_time = ms - (stop - start);
    if(sleep_time > std::chrono::duration<int32_t,std::nano>(0))
    {
      std::this_thread::sleep_for(sleep_time);
    }
    else
    {
      std::cout << "FG: We took too long\n";
    }
  }
}


void run_ivy_thread(void) {
  std::chrono::high_resolution_clock::time_point start;
  std::chrono::high_resolution_clock::time_point stop;
  std::chrono::duration<int32_t, std::nano> sleep_time;

  cout << "IVY Thread init" << endl;

  char x[] = "127.255.255.255";
  nps_ivy_init(x);

/*
  IvyInit ("IvyTranslater", "Hello le monde", 0, 0, 0, 0);
  IvyStart (x);
  IvyBindMsg(HelloCallback, 0, "^Hello(.*)");
  IvyBindMsg(ByeCallback, 0, "^Bye$");
*/
  //IvyMainLoop();
  nps_ivy_main_loop();
  /*
  while(true)
  {
    start = std::chrono::high_resolution_clock::now();
    cout << "IVY Thread running" << endl;

    float x = 1.23;
    IvySendMsg("%d NPS_RATE_ATTITUDE %f %f %f %f %f %f",
               42,
               x,
               x,
               x,
               x,
               x,
               x);


    stop = std::chrono::high_resolution_clock::now();
    sleep_time = ms - (stop - start);
    if(sleep_time > std::chrono::duration<int32_t,std::nano>(0))
    {
      std::this_thread::sleep_for(sleep_time);
    }
    else
    {
      std::cout << "IVY: We took too long\n";
    }
  }
  */
}


int main ()
{
    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTSTP, tstp_hdl);

    std::thread t_fg(run_fg_thread);
    t_fg.detach();

    std::thread t_ivy(run_ivy_thread);
    t_ivy.join();

    //std::thread t_nps(run_behavior);
    //t_nps.detach();


    /*
    while(1){
       cout << "Going to sleep...." << endl;
       sleep(1);
    }
    */

    return 0;
}
