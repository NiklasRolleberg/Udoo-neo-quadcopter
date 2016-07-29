#include "A9.hpp"
#include <iostream>

A9::A9()
{
  std::cout << "A9 constructor" << std::endl;

  serialport_M4.setSerialPort("/dev/ttyMCC",115200);
  serialport_M4.AddObserver(*this);
  serialport_M4.start();

  serialport_GPS.setSerialPort("/dev/ttyUSB0",9600);
  serialport_GPS.AddObserver(*this);
  serialport_GPS.start();
  serialport_GPS.send("$PMTK220,5000*1B\r\n"); // position data every 5s
  serialport_GPS.send("$PMTK300,10000,0,0,0,0*2C\r\n"); //fix data every 10s
  serialport_GPS.send("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); //GPRMC and GGA
  serialport_GPS.send("$PGCMD,33,0*6D\r\n"); //don't send antenna message

  network.AddObserver(*this);
  network.start(5555);
}
A9::~A9()
{
  std::cout << "A9 destructor" << std::endl;
}

void A9::update()
{
  if(serialport_M4.hasChanged())
  {
    std::cout << "From M4: " << serialport_M4.getLine() << std::endl;
  }
  if(serialport_GPS.hasChanged())
  {
    //std::cout << "From gps:" << serialport_GPS.getLine() << std::endl;
    gps.decodeMessage(serialport_GPS.getLine());
  }
  if(network.hasChanged())
  {
    std::cout << "From Android: " << network.getMessage() << std::endl;
    //serialport.send("$QCPUL,1501,1502,1503,1504,*00\n");
    serialport_M4.send("$QCSTA,0.111,0.222,0.333,*00\n");
  }
}
