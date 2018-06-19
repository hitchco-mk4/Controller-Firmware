#include "Arduino.h"
#include "Printer.h"

Printer::Printer() {
	_enabled = true;
  enable();
}

void Printer::disable(void) {
  _enabled = false;
}

void Printer::enable(void) {
  _enabled = true;
}

void Printer::debug_print(const String &s, bool newline) {
	if (_enabled) {
		if (newline) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}


void Printer::debug_print(int s, bool newline) {
	if (_enabled) {
		if (newline) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}

void Printer::debug_print(float s, bool newline) {
	if (_enabled) {
		if (newline) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}

void Printer::debug_print(byte s, bool newline) {
	if (_enabled) {
		if (newline) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}


void Printer::debug_print(unsigned long s, bool newline) {
	if (_enabled) {
		if (newline) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}
