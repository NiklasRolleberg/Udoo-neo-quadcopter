#ifndef UDP_SERVER_H
#define UDP_SERVER_H

class UDP_server {

private:

  int sock, length, n;
  socklen_t fromlen;
  struct sockaddr_in server;
  struct sockaddr_in from;
  char buf[1024];

  void listen();
  void error(const char *msg);

public:
  UDP_server();
  ~UDP_server();

  void start(int port);

  void send(char* buf, int length);


};

#endif //UDP_SERVER
