#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string>

class SerialPort {

private:

  int fd;

  int set_interface_attribs (int fd, int speed, int parity);
  void set_blocking (int fd, int should_block);

public:

  SerialPort();
  
  void setup(std::string portName, int speed);

  void debug();

  int send(const char* buf, int l);

  int receive(char* buf, int l);

};

#endif //SERIAL_H
