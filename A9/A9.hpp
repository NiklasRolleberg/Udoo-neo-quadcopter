#ifndef A9_H
#define A9_H

#include "serialmanager.hpp"
#include "UDP_server.hpp"

class A9 : public Observer {

private:
  SerialPortManager serialport;
  UDP_server network;
public:
  A9();
  ~A9();
  void update();
};

#endif //A9
