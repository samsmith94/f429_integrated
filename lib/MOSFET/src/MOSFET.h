// MOSFET.h

#ifndef _MOSFET_h
#define _MOSFET_h

#include "Arduino.h"
  
class MOSFET
{
 protected:
	 int pin;
	 bool state;
	 bool normallyOpen;

 public:
	 MOSFET();
	 MOSFET(int p, bool isNormallyOpen);
	 void begin();
	 bool getState();
	 void turnOn();
	 void turnOff();
};

#endif
