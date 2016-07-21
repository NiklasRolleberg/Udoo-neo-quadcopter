#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

#include "serial.hpp"
#include <cstring>
#include "observer.hpp"
#include <thread>

class SerialPortManager : public Observable
{

private:
  bool run;
  SerialPort port;

  static const int bufferSize = 40;
  char buffer[bufferSize];
  int stopIndex;

  std::string message;

  std::thread* listenThread;

  void listen();

public:
  SerialPortManager();
  ~SerialPortManager();

  void start();
  void stop();

  void send(std::string s);
  std::string getLine();


};

#endif //SERIALMANAGER
