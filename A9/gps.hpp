#ifndef GPS_H
#define GPS_H

#include <cstring>

class GPS 
{
private:
  //private
public:
  volatile double latitude = 0;
  volatile double longitude = 0;
 
  //TODO: time, speed, cog etc

  GPS(){};
  ~GPS(){};

  void decodeMessage(std::string message);
};

#endif //A9
