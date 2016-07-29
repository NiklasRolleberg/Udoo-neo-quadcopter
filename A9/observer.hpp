#ifndef OBSERVERPATTERN_H
#define OBSERVERPATTERN_H

#include <iostream>
#include <vector>

class Observer
{
public:
  Observer(){};
  virtual void update() = 0;
};


class Observable
{
private:
  std::vector<Observer*> observers;
  bool ch;

public:
  Observable(){ch = false;};

  void notifyObservers()
  {
    for (std::vector<Observer*>::iterator it = observers.begin(); it != observers.end(); ++it)
      (*it)->update();
 
    ch = false;
  };

  void setChanged()
  {
    ch = true;
  };

  bool hasChanged()
  {
    return ch;
  };

  void AddObserver(Observer &o)
  {
    observers.push_back(&o);
  };

  void ClearObservers()
  {
        observers.clear();
  };
};

#endif //OBSERVERPATTERN_H
