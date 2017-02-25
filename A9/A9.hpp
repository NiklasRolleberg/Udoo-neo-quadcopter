#ifndef A9_H
#define A9_H

#include "serialmanager.hpp"
#include "UDP_server.hpp"

#include "gps.hpp"

class A9 : public Observer 
{
private:
  SerialPortManager serialport_M4;
  SerialPortManager serialport_GPS;
  UDP_server network;
  
  GPS gps;
public:
  A9();
  ~A9();
  void update();
};

#endif //A9
