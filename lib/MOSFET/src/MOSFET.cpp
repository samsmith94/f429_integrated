#include "MOSFET.h"

MOSFET::MOSFET(){}

MOSFET::MOSFET(int p, bool isNormallyOpen){
	pin = p;
	normallyOpen = isNormallyOpen;
}

void MOSFET::begin(){
	pinMode(pin, OUTPUT);
}

bool MOSFET::getState(){
	if (normallyOpen){
		return !state;
	}
	else {
		return state;
	}
}
void MOSFET::turnOn(){
	if (normallyOpen){
		if (state == !true) return;
		state = !true;
	} else {
		if (state == true) return;
		state = true;
	}
	digitalWrite(pin, state);
}
void MOSFET::turnOff(){
	if (normallyOpen){
		if (state == !false) return;
		state = !false;
	} else {
		if (state == false) return;
		state = false;
	}
	digitalWrite(pin, state);
}
