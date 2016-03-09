#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>

#include "UDP_Server.hpp"

UDP_server::UDP_server()
{
  printf("udp-server contructor\n");
}

UDP_server::~UDP_server()
{
  printf("udp-server destructor\n");
}

void UDP_server::start(int port)
{
  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) error("Opening socket");
  length = sizeof(server);
  bzero(&server,length);
  server.sin_family=AF_INET;
  server.sin_addr.s_addr=INADDR_ANY;
  server.sin_port=htons(port);
  if (bind(sock,(struct sockaddr *)&server,length)<0)
      error("binding");
  fromlen = sizeof(struct sockaddr_in);

  listen();
}

void UDP_server::listen()
{
  while (1)
  {
      n = recvfrom(sock,buf,1024,0,(struct sockaddr *)&from,&fromlen);
      if (n < 0) error("recvfrom");
      //write(1,"\nReceived a datagram: ",21);
      //write(1,buf,n);

      buf[n]=0;
      printf("%s\n", buf); /* or, slightly more efficient: fputs(buffer, stdout); */
  }
}


void UDP_server::send(char* buf, int length)
{
  printf("send something back\n");

  n = sendto(sock,"Got your message\n",17,
             0,(struct sockaddr *)&from,fromlen);
  if (n  < 0) error("sendto");

}

void UDP_server::error(const char *msg)
{
  perror(msg);
  exit(0);
}
