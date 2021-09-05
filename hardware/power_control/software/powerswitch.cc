
#include <iostream>
#include <cstring>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <ctime>
#include <cassert>
#include <sstream>
#include <iomanip>

#include "powerswitch.h"

using namespace std;

double adc2v(unsigned char c) {
  return c /255.*2.5/0.25;
}

volatile int lastsignal;

void sig_term_handler(int signum, siginfo_t *info, void *ptr)
{
  lastsignal=signum;
}

static void throwonreadwritefailed(const char * op, ssize_t rv, size_t sz) {
  if(rv < 0) {
    ostringstream ess;
    ess << op << " error: ";
    const char * msg = std::strerror(errno);
    if(msg)
        ess << msg;
    else
      ess << "errno=" << errno;
    throw std::runtime_error(ess.str());
  }
  else if(rv != static_cast<ssize_t>(sz)) {
    ostringstream ess;
    ess << op << " error: short " << op;
    throw std::runtime_error(ess.str());
  }
}


static void safewrite(int fileno, const void * data, size_t sz) {
  ssize_t bw = write(fileno, data, sz);
  throwonreadwritefailed("write",bw,sz);
}
static void saferead(int fileno, void * data, size_t sz) {
  ssize_t bw = read(fileno, data, sz);
  throwonreadwritefailed("read",bw,sz);
}

int main(int argc, char ** argv) {


  enum {NORMAL,ABORT,TEST} mode= NORMAL;
  for(int i=1;i<argc;i++) {
    if(!strcmp(argv[i],"--test"))
      mode = TEST;
    else if(!strcmp(argv[i],"--abort"))
      mode = ABORT;
    else {
      cerr << "unknown command, --test or --abort" << endl;
      return 1;
    }
  }
  
  int file = open_i2c(1, POWERSWITCH_I2C_ADDRESS);
  if(file < 0) {
    cerr << "open failed" << endl;
    return 1;
  }

  if(mode == ABORT || mode == TEST) {
    uint8_t cbuf[] = {0x01, 0};

    if(mode==TEST)
      cbuf[1]=60;

    try {
      safewrite(file,cbuf,sizeof(cbuf));
    }
    catch (runtime_error const& ex) {
      cerr << ex.what() << endl;
      close(file);
      return 1;
    }
    close(file);
    return 0;
  }

  // register signal handlers
  struct sigaction sa;
  memset(&sa,0,sizeof(sa));
  sa.sa_sigaction=sig_term_handler;
  sigaction(SIGUSR1, &sa, NULL);
  sigaction(SIGUSR2, &sa, NULL);

#if 0
  {
    // set a specific LVC
    const char txbuf[] = {0x03, 0x70};
    if(sizeof(txbuf)!=write(file,txbuf,sizeof(txbuf)))
      cerr << "failed to set LVC, errno=" << errno << endl;
  }
#endif
  
  struct timespec lastprint;
  memset(&lastprint,0,sizeof(lastprint));
  char previous[6];
  memset(previous,0,sizeof(previous));
  int shutdown_ctr=0;

  while(1) {

    try {
      // write address to 0
      const uint8_t txbuf=0;
      safewrite(file,&txbuf,sizeof(txbuf));
      
      // read registers 0-5
      uint8_t rxbuf[6];
      saferead(file,rxbuf,sizeof(rxbuf));
      const uint8_t * buf = rxbuf;

      // read was successful

      // decide whether to print a status line
      //   we want every 5 minutes unless something
      //   has changed
      
      bool diff = false;

      // check for registers changed (with some fuzz for ADC)
      static_assert(sizeof(previous)==sizeof(rxbuf));
      for(size_t i=0;i<sizeof(previous);i++) {
	if(i==0x02) { //adc +/- 2 counts
	  if(abs(buf[i]-previous[i]) > 2)
	    diff=true;
	}
	else if(buf[i]!= previous[i]) {
	  diff=true;
	}
      }
	
      if (!diff) {
	// print anyway if it's been 5 minutes with no change
	struct timespec now;
	assert(!clock_gettime(CLOCK_MONOTONIC, &now));
	if(now.tv_sec > lastprint.tv_sec + 300)
	  diff = true;
      }
      if(diff) {
	for(size_t i=0;i<sizeof(rxbuf);i++)
	  cout << hex << setfill('0') << setw(2) << (int)buf[i] << " ";
	cout << "SYS=" << (buf[0]!=0 ? "ON ":"OFF");
	cout << " MOTOR=" << (buf[0]==2 ? "ON ":"OFF");
	cout << dec << " countdown=" << (int)buf[1];
	cout << fixed << setprecision(1) << " V=" << adc2v(buf[2]) << " LVC=" << adc2v(buf[3]) << endl;

	memcpy(previous,buf,sizeof(previous));
	assert(!clock_gettime(CLOCK_MONOTONIC, &lastprint));
      }


      // shutdown the system if we get 3 consecutive reads where
      //   the timer is under 45 seconds
      
      if(buf[1] && buf[1] < 45 ) {
	shutdown_ctr++;
	
	// make sure a random i2c error doesn't shut us down, so only poweroff if we
	//    get 3 in a row
	if(shutdown_ctr > 3) {
	  
	  // execute systemctl poweroff
	  const char * newargv[] = { "/usr/bin/systemctl", "poweroff", (char *)NULL };
	  execv(newargv[0],const_cast<char*const*>(newargv));
	  
	  // execv doesn't return unless there's an error
	  cerr << "execv failed" << endl;
	  return 1;
	}
      }
      else
	  shutdown_ctr=0;
      
    }
    catch(std::runtime_error const& ex) {
      cerr << ex.what() << endl;
    }

    sleep(1);

    // the user can toggle the motors on or off
    //    with SIGUSR1 and SIGUSR2 respectively
    if(lastsignal) {
      int s = lastsignal;
      lastsignal=0;
      
      cout << "caught signal " << s << endl;
      try {
	if(s == SIGUSR1) {
	  const uint8_t txdata[] = {0x00, 0x02};
	  safewrite(file,txdata,sizeof(txdata));
	}
	else if(s == SIGUSR2) {
	  const uint8_t txdata[] = {0x00, 0x01};
	  safewrite(file,txdata,sizeof(txdata));
	}
      }
      catch (runtime_error const& ex) {
	cerr << ex.what() << endl;
      }
    }
  }
  return 0;
}
