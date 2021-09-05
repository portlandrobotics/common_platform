
#include <iostream>
#include <cstring>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "powerswitch.h"

using namespace std;

// program for systemd-halt to call to schedule poweroff in 10 seconds

// should be placed in /usr/lib/systemd/system-shutdown/

// will be called with halt, poweroff, reboot, or kexec

int main(int argc, char ** argv) {

  // only act if argv[1] is halt or poweroff
  //   it could be argued that we should only do poweroff
  
  int proceed=false;

  if(argc > 1) {
    cout << "got arg " << argc << " " << argv[1] << endl;
    if(!strcmp("halt",argv[1]))
      proceed=true;
    if(!strcmp("poweroff", argv[1]))
      proceed=true;
  }
  if(!proceed) {
    cout << "not a shutdown" << endl;
    return 0;
  }

  cout << "powering off" << endl;
  
  const int file = open_i2c(1, POWERSWITCH_I2C_ADDRESS);

  if(file < 0) {
    cerr << "i2c open failed" << endl;
    return 1;
  }

  int returncode=0;
  uint8_t buf[2];
  
  // write 10 seconds to the shutdown timer register (0x01) 
  buf[0]=0x01;
  buf[1]=10; // seconds
  if ( write(file,buf,2) != 2) {
    cerr << "short i2c write " << errno << endl;
    returncode=1;
  }

  close(file);
  return returncode;
}
