
#pragma once

#include <iostream>
#include <cstring>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

const static int POWERSWITCH_I2C_ADDRESS = 0x6e;

static int open_i2c(int adapter_nr, int address) {
  
  int file;
  char filename[32];

  auto rv = snprintf(filename,sizeof(filename),"/dev/i2c-%d",adapter_nr);
  if (rv >= static_cast<decltype(rv)>(sizeof(filename)) || rv < 0) {
    std::cerr << "snprintf truncated" << std::endl;
    return -1;
  }
  
  if ((file = open(filename,O_RDWR)) < 0) {
    std::cerr << "file open failed " << filename << " errno=" << errno << std::endl;
    return -1;
  }

  int tmpaddr = address; /* The I2C address */
  if (ioctl(file,I2C_SLAVE,tmpaddr) < 0) {
    std::cerr << "set address failed, errno=" << errno << std::endl;
    close(file);
    return -1;
  }

  return file;
}
