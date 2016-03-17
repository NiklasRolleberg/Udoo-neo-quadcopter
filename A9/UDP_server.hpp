#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include "observer.hpp"


class UDP_server : public Observable{

private:

  int sock, length, n;
  socklen_t fromlen;
  struct sockaddr_in server;
  struct sockaddr_in from;
  char buf[1024];
  bool run;
  std::string message;

  void listen();
  void error(const char *msg);

public:
  UDP_server();
  ~UDP_server();

  void start(int port);
  void stop();

  void send(char* buf, int length);

  std::string getMessage();
};

#endif //UDP_SERVER
