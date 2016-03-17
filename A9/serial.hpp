#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>

class SerialPort {

private:

  //int test1[10];
  int fd;
  //int test2[10]; //keeps fd from changing...

  int set_interface_attribs (int fd, int speed, int parity);
  void set_blocking (int fd, int should_block);

public:

  SerialPort();

  void debug();

  int send(char* buf, int l);

  int receive(char* buf, int l);

};

#endif //SERIAL_H
