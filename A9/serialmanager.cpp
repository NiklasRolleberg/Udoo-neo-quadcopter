#include "serialmanager.hpp"
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <thread>

SerialPortManager::SerialPortManager()
{
  run = true;
  stopIndex = 0;
  for(int i=0;i<bufferSize;i++)
    buffer[i] = 0;

}

SerialPortManager::~SerialPortManager()
{
  std::cout << "SerialPortManager destructor" << std::endl;
  stop();
  delete listenThread;
}

void SerialPortManager::start()
{
  //TODO start in a new thread
  listenThread = new std::thread(&SerialPortManager::listen, this);
  //listen();
}

void SerialPortManager::listen()
{
  int test = 0;
  while(run)
  {
    usleep(100000);

    //clear buffer
    if(stopIndex == bufferSize-1)
    {
      std::cout << "Clear buffer" << std::endl;
      for(int i=0;i<bufferSize;i++)
        buffer[i] = 0;
      stopIndex = 0;
    }

    //read from serial port
    int r = port.receive(buffer+stopIndex,bufferSize-stopIndex);

    if(r<0)
    {
      perror ("Read error occurred ");
      port.debug();
      std::cout << "Read: " << r << " : " << stopIndex << std::endl;
      std::cout << "----- Buffer -------" << std::endl << buffer << std::endl
                << "--------------------" << std::endl;
      exit(-1);
    }
    else
      stopIndex += r;

    int start = 0;
    for(int i=0;i<=stopIndex;i++)
    {
      if(buffer[i] == '\n')
      {
        if((i-start)>0)
        {
          message = std::string(buffer + start,i-start);
          setChanged();
          notifyObservers();
          //std::cout << "Message: " << (i-start) << " : " << message << std::endl;
        }
        start = i+1;
      }
    }
    if(start <= stopIndex && start != 0)
    {
      for(int i=0;i<stopIndex-start;i++)
        buffer[i] = buffer[start+i];
      for(int i=stopIndex-start;i<bufferSize;i++)
        buffer[i] = 0;
      stopIndex = stopIndex-start;
    }
  }
}

void SerialPortManager::stop()
{
  run = false;
}

void SerialPortManager::send(std::string s)
{
  std::cout << "send: " << s << std::endl;
  int w = port.send(s.c_str(),s.size());

}

std::string SerialPortManager::getLine()
{
  return message;
}
