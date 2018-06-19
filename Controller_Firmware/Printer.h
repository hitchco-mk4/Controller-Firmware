// printer allows developer to keep debugging print statements around without having to comment/uncomment. 

#ifndef Printer_h
#define Printer_h  

#include "Arduino.h"

class Printer {
	public:
		Printer();
		void debug_print(const String &s, bool newline);
		void debug_print(int s, bool newline);
		void debug_print(float s, bool newline);
		void debug_print(byte s, bool newline);
		void debug_print(unsigned long s, bool newline);
    void disable();
    void enable();
	private:
		bool _enabled;
};

#endif
