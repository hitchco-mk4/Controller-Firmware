/*
Test Functions
Run these at your own peril, some of them use state and could mess things up
*/

void test_keypad_pushbuttons(void) {

	for (int i = 0; i < NUMKEYPADLEDS; i++) {

		setAndWrite(keypadLEDs[i], HIGH);
		bool waiting_for_push = true;

		while (waiting_for_push) {

			int reading = read_mux_button(keypadbuttons[i]);

			if (reading) {
				waiting_for_push = false;
			}
		}
		setAndWrite(keypadLEDs[i], LOW);
	}

}

void test_system_pushbuttons(void) {

	for (int i = 0; i < 4; i++) {

		setAndWrite(system_button_LEDs[i], HIGH);
		bool waiting_for_push = true;

		while (waiting_for_push) {

			int reading = read_mux_button(system_buttons[i]);

			if (reading) {
				waiting_for_push = false;
			}
		}
		setAndWrite(system_button_LEDs[i], LOW);
	}

}

void test_start_button(void) {

	for (int i = 0; i < NUMPB8COLORS; i++) {

		set_pb8_color(pb8_colors[i]);

		debug_print(pb8_colors[i], true);

		bool waiting_for_push = true;

		while (waiting_for_push) {

			int reading = digitalRead(PB8_SIG_PIN);

			if (reading) {
				waiting_for_push = false;
			}
		}

		delay(1000);

	}
}

void test_LEDs(void) {

	debug_print("testing leds", true);

	int LEDs[2] = { LED1_EN_PIN, LED2_EN_PIN };

	for (int i = 0; i < NUMPB8COLORS; i++) {

		set_pb8_color(pb8_colors[i]);

		bool waiting_for_push = true;

		while (waiting_for_push) {

			int reading = digitalRead(PB8_SIG_PIN);

			if (reading) {
				waiting_for_push = false;
			}
		}

		delay(1000);
	}

	debug_print("leds done", true);
}

void test_output_fets(void) {

	int output_fets[NUMOUTFETS] = { SPED_SIG_PIN, ECF_EN_PIN, STEN_EN_PIN, RUEN_EN_PIN, DIRL_EN_PIN, DIRR_EN_PIN, WIPEO_EN_PIN, WIPEL_EN_PIN, WIPEH_EN_PIN, PSU1_EN_PIN, PSU2_EN_PIN, PSU3_EN_PIN };
	for (int i = 0; i < NUMOUTFETS; i++) {

		digitalWrite(output_fets[i], HIGH);

		bool waiting_for_push = true;

		while (waiting_for_push) {

			int reading = digitalRead(PB8_SIG_PIN);

			if (reading) {
				waiting_for_push = false;
			}

			delay(100);
		}

		waiting_for_push = true;

		while (waiting_for_push) {

			int reading = digitalRead(PB8_SIG_PIN);

			if (reading == false) {
				waiting_for_push = false;
			}

			delay(100);
		}

		digitalWrite(output_fets[i], LOW);
	}
}

void test_latch(void) {

	bool waiting_for_push = true;

	while (waiting_for_push) {

		int reading = digitalRead(PB8_SIG_PIN);

		if (reading) {
			waiting_for_push = false;
		}
	}
	pinMode(PSU1_EN_PIN, OUTPUT);
	digitalWrite(PSU1_EN_PIN, LOW);
}