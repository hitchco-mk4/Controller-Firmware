#ifndef DEBUG_PRINT_H_   /* Include guard */
#define DEBUG_PRINT_H_  

void debug_print(const String &s, bool newline) {
#if DEBUGMODE
	if (newline) {
		Serial.println(s);
	}
	else {
		Serial.print(s);
	}
#endif // DEBUGMODE
}

void debug_print(int s, bool newline) {
#if DEBUGMODE
	if (newline) {
		Serial.println(s);
	}
	else {
		Serial.print(s);
	}
#endif // DEBUGMODE
}

void debug_print(float s, bool newline) {
#if DEBUGMODE
	if (newline) {
		Serial.println(s);
	}
	else {
		Serial.print(s);
	}
#endif // DEBUGMODE
}

void debug_print(byte s, bool newline) {
#if DEBUGMODE
	if (newline) {
		Serial.println(s);
	}
	else {
		Serial.print(s);
	}
#endif // DEBUGMODE
}

void debug_print(unsigned long s, bool newline) {
#if DEBUGMODE
	if (newline) {
		Serial.println(s);
	}
	else {
		Serial.print(s);
	}
#endif // DEBUGMODE
}

#endif