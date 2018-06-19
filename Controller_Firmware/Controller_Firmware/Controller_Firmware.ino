#include <limits.h>

#define DEBUGMODE 1

#include "debug_print.h"
#include "data_block.h"


/*
  State Machine Setup
*/

// state machine states
#define MODECOMBO 0
#define MODEAUTOSTART 1
#define MODERUNNING 2
#define MODESHUTOFF 3
#define MODELIMBO 4

int state; // the global state variable
int starting = false;
int transitiontimeoutEnabled = true;
int killSwitchEnabled = true; // if set to true, the main switch can disable the starter

unsigned long lastTransitionTime = 0;
const long transitionTimeoutInterval = 20 * 1000; // 20 seconds


/* 
  Auto Start Variables
*/

unsigned long startTime = 0;
const long startDuration = 5000; // in milliseconds, how long a start attempt can take
bool auto_start_fail = true;


bool shutting_off = false; 
unsigned long startShutoffTime = 0;
const long shutoffTimeInterval = 5000;

unsigned long lastStartButtonPressTime = 0; // for debouncing the start button
const long startButtonDuration = 200; // how long the start button must be held
bool startButtonPressed = false;

#define COMBOLENGTH 4
int comboPos = 0;
int comboBuffer[COMBOLENGTH];
static int goodCombo[COMBOLENGTH] = { 2, 5, 2, 0 };


/*
  analog pin declarations
*/

int VBAT_SIG_PIN =	0; // on the arduino analog pins
int ECT_SIG_PIN =	1; // on the arduino analog pins
int ACT_SIG_PIN =	2; // on the arduino analog pins
int OILP_SIG_PIN =	3; // on the arduino analog pins
int FUEL_SIG_PIN =	4; // on the arduino analog pins
int OILL_SIG_PIN =	A5; // on the arduino analog pins - used as digital input
int BATC_SIG_PIN =	6; // on the arduino analog pins
int EGOL_SIG_PIN =	7; // on the arduino analog pins
int EGOR_SIG_PIN =	8; // on the arduino analog pins
int EBRK_SIG_PIN =	9; // on the arduino analog pins
int RVRS_SIG_PIN =	10; // on the arduino analog pins
int PSU1_SIG_PIN =	A11; // on the arduino analog pins - used as digital input
int MUX_SIG_PIN =	12; // on the arduino analog pins
int CSS_SIG_PIN =	13; // on the arduino analog pins
int LDR_SIG_PIN =	14; // on the arduino analog pins
int LITE_SIG_PIN =	15; // on the arduino analog pins

/*
  digital pin declarations
*/

#define NUMOUTFETS 12
#define NUMDIGPINS 33

int TACK_SIG_PIN =	2; // on the arduino digital pins - an interrupt pin
int SPED_SIG_PIN =	3;  // on the arduino digital pins - an interrupt pin
int LED2_EN_PIN =	4; // on the arduino digital pins - green LED
int STEN_EN_PIN =	5; // on the arduino digital pins
int RUEN_EN_PIN =	6; // on the arduino digital pins
int DIRL_EN_PIN =	7; // on the arduino digital pins
int DIRR_EN_PIN =	8; // on the arduino digital pins
int WIPEO_EN_PIN =	9; // on the arduino digital pins
int WIPEL_EN_PIN =	10; // on the arduino digital pins
int WIPEH_EN_PIN =	11; // on the arduino digital pins
int LED1_EN_PIN =	12; // on the arduino digital pins - red LED
int debug_LED_en =	13; // on the arduino digital pins - onboard LED, not visible

/* pins 14, 15 are used by the hardware Serial3 16, 17 are not used */

int HALL_SIG_PIN =	18; // on the arduino digital pins - an interrupt pin
int HALR_SIG_PIN =	19; // on the arduino digital pins - an interrupt pin

/* pins 20, 21 are not used */

int ECF_EN_PIN = ; // on the arduino digital pins

// Power MGMT pins
int PSU1_EN_PIN =	22; // on the arduino digital pins - this is the onboard switching converter
int PSU2_EN_PIN =	23; // on the arduino digital pins - offboard
int PSU3_EN_PIN =	24; // on the arduino digital pins - offboard
int PSU4_EN_PIN =	25; // on the arduino digital pins - offboard

// Shift Register Control Pins
int SREG_SER_PIN =		26; // on the arduino digital pins
int SREG_RCLK_PIN =		27; // on the arduino digital pins
int SREG_SRCLK_PIN =	28; // on the arduino digital pins
int SREG_SRCLR_PIN =	29; // on the arduino digital pins
int SREG_OE_PIN =		30; // on the arduino digital pins

// Mux control pins
int MUX_S3_PIN =	31; // on the arduino digital pins
int MUX_S2_PIN =	32; // on the arduino digital pins
int MUX_S1_PIN =	33; // on the arduino digital pins
int MUX_S0_PIN =	34; // on the arduino digital pins
int MUX_EN_PIN =	35; // on the arduino digital pins

// general purpose mosfets
int GPN0_EN_PIN =	36; // on the arduino digital pins
int GPN1_EN_PIN =	37; // on the arduino digital pins
int GPP0_EN_PIN =	38; // on the arduino digital pins
int GPP1_EN_PIN =	39; // on the arduino digital pins

// Cruise control pins
int CRZ_RESUME_PIN =	40; // on the arduino digital pins
int CRZ_MAINTAIN_PIN =	41; // on the arduino digital pins
int CRZ_DISABLE_PIN =	42; // on the arduino digital pins

// Starter button pins
int PB8_SIG_PIN =	43; // on the arduino digital pins
int PB8_BLED_PIN =	44; // on the arduino digital pins
int PB8_GLED_PIN =	45; // on the arduino digital pins
int PB8_RLED_PIN =	46; // on the arduino digital pins

// Debug LEDs
int LED3_EN_PIN =	47; // on the arduino digital pins - onboard debug pin
int LED4_EN_PIN =	48; // on the arduino digital pins - onboard debug pin

int output_fets[NUMOUTFETS] = { SPED_SIG_PIN, ECF_EN_PIN, STEN_EN_PIN, RUEN_EN_PIN, DIRL_EN_PIN, DIRR_EN_PIN, WIPEO_EN_PIN, WIPEL_EN_PIN, WIPEH_EN_PIN, PSU1_EN_PIN, PSU2_EN_PIN, PSU3_EN_PIN };


/*
  Shift Register setup and pin mapping
*/

#define number_of_74hc595s 2 // Number of Shift Registers
#define numOfRegisterPins number_of_74hc595s * 8
boolean registers[numOfRegisterPins];

unsigned long lastBounceTime = 0;
const long debounceInterval = 10;

unsigned long lastPressTime = 0;
const long pressTimeoutInterval = 3000;

#define NUMPUSHBUTTONS 15

int PB1_LED_EN = 0; // on the shift registers
int PB2_LED_EN = 1; // on the shift registers
int PB3_LED_EN = 2; // on the shift registers
int PB4_LED_EN = 3; // on the shift registers
int PB6_LED_EN = 4; // on the shift registers
int PB7_LED_EN = 5; // on the shift registers
int PB9_LED_EN = 6; // on the shift registers
int PB10_LED_EN = 7; // on the shift registers
int PB11_LED_EN = 8; // on the shift registers
int PB12_LED_EN = 9; // on the shift registers
int PB13_LED_EN = 10; // on the shift registers
int PB14_LED_EN = 11; // on the shift registers
int PB15_LED_EN = 12; // on the shift registers
int PB16_LED_EN = 13; // on the shift registers
int PB17_LED_EN = 14; // on the shift registers

int pushbutton_LEDs[NUMPUSHBUTTONS] = { PB1_LED_EN, PB2_LED_EN, PB3_LED_EN, PB4_LED_EN, PB6_LED_EN, PB7_LED_EN, PB9_LED_EN, PB10_LED_EN, PB11_LED_EN, PB12_LED_EN, PB13_LED_EN, PB14_LED_EN, PB15_LED_EN, PB16_LED_EN, PB17_LED_EN };


/*
  Multiplexer Mapping
*/
int PB1_SIG_PIN = 0; // on the mux - left blinker
int PB2_SIG_PIN = 1; // on the mux - right blinker
int PB3_SIG_PIN = 2; // on the mux - hazards
int PB4_SIG_PIN = 3; // on the mux
int PB6_SIG_PIN = 4; // on the mux
int PB7_SIG_PIN = 5; // on the mux
int PB9_SIG_PIN = 6; // on the mux
int PB10_SIG_PIN = 7; // on the mux
int PB11_SIG_PIN = 8; // on the mux
int PB12_SIG_PIN = 9; // on the mux
int PB13_SIG_PIN = 10; // on the mux
int PB14_SIG_PIN = 11; // on the mux
int PB15_SIG_PIN = 12; // on the mux
int PB16_SIG_PIN = 13; // on the mux
int PB17_SIG_PIN = 14; // on the mux

#define NUMKEYPADLEDS 9
int keypadLEDs[NUMKEYPADLEDS] = { PB9_LED_EN, PB10_LED_EN, PB11_LED_EN, PB12_LED_EN, PB13_LED_EN, PB14_LED_EN, PB15_LED_EN, PB16_LED_EN, PB17_LED_EN }; // these are on the shift registers

#define NUMKEYPADBUTTONS NUMKEYPADLEDS
int keypadbuttons[NUMKEYPADBUTTONS] = { PB9_SIG_PIN, PB10_SIG_PIN, PB11_SIG_PIN, PB12_SIG_PIN, PB13_SIG_PIN, PB14_SIG_PIN, PB15_SIG_PIN, PB16_SIG_PIN, PB17_SIG_PIN }; // these are on the multiplexer
int lastStates[NUMKEYPADBUTTONS] = { 0 }; // holds the previous states of the buttons (not-pressed = 0, pressed = 1)

#define NUMSYSTEMBUTTONLEDS 6
int system_button_LEDs[NUMSYSTEMBUTTONLEDS] = { PB1_LED_EN, PB2_LED_EN, PB3_LED_EN, PB4_LED_EN, PB6_LED_EN, PB7_LED_EN };

#define NUMSYSTEMBUTTONS NUMSYSTEMBUTTONLEDS
int system_buttons[NUMSYSTEMBUTTONS] = { PB1_SIG_PIN, PB2_SIG_PIN, PB3_SIG_PIN, PB4_SIG_PIN, PB6_SIG_PIN, PB7_SIG_PIN };


/*
  Set up Start Button (PB8)
*/

#define PB8BLACK 0
#define PB8RED 1
#define PB8GREEN 2
#define PB8BLUE 3
#define PB8YELLOW 4

#define NUMPB8COLORS 5

int pb8_colors[NUMPB8COLORS] = {PB8BLACK, PB8RED, PB8GREEN, PB8BLUE, PB8YELLOW };


/*
  Car starting variables setup
*/

#define OILPHIGHTIMECARON 100 // the amount of time oil p is high for the car to be considered on
unsigned long oilp_high_time = 0;
bool oilp_timer_started = 0;

/*
  General pushbutton setup
*/


#define PRESSTIME 100


/*
  Blinking variables setup
*/

#define NUMBLINKVARS 4

#define BLINKSLOW 0
#define BLINKMEDIUM 1
#define BLINKFAST 2
#define BLINKFASTEST 3

int blink_increments[NUMBLINKVARS] = { 800, 700, 200, 50 }; 
unsigned long blink_times[NUMBLINKVARS] = { 0 };
bool blink_states[NUMBLINKVARS] = { false };

// for wheel canceling
bool waiting_for_left = true;
bool waiting_for_right = true;
unsigned long left_time = 0;
unsigned long right_time = 0;


/*
  Turn signal variables setup
*/

#define NUMTOGGLEBUTTONSLEDS 3 

#define LEFTBLINKSTATE 0 // the location of the left blinker's state in toggle_states
#define RIGHTBLINKSTATE 1
#define HAZARDBLINKSTATE 2

int toggle_buttons[NUMTOGGLEBUTTONSLEDS] = { PB1_SIG_PIN, PB2_SIG_PIN, PB3_SIG_PIN };
int toggle_button_LEDs[NUMTOGGLEBUTTONSLEDS] = { PB1_LED_EN, PB2_LED_EN, PB3_LED_EN }; 

bool toggle_states[NUMTOGGLEBUTTONSLEDS] = { false };
bool toggle_newpresses[NUMTOGGLEBUTTONSLEDS] = { true };
unsigned long toggle_downtimes[NUMTOGGLEBUTTONSLEDS] = { 0 };
unsigned long toggle_pressstarted[NUMTOGGLEBUTTONSLEDS] = { false };

bool left_turn_signal_state = 0;
bool right_turn_signal_state = 0;


/*
  Wiper button variables setup
*/

#define WIPEROFF 0
#define WIPERLOW 1
#define WIPERHIGH 2

int wiperstate = WIPEROFF; 
unsigned long previous_wiperbutton_time = 0;
bool looking_for_wiper_button = true;


/*
  Engine fan (ECT) Table Setup
*/

#define NUMECTTABLEVALUES 12

int ECT_table_temps[NUMECTTABLEVALUES] =  { 248,    230,    212,    194,    176,    158,    140,    122,    104,    86,     68,     50 }; // in degrees
float ECT_table_volts[NUMECTTABLEVALUES] =  { 0.28,   0.36,   0.47,   0.61,   0.80,   1.04,   1.35,   1.72,   2.16,   2.62,   3.06,   3.52 }; // in volts

int engine_temp_low = 122;    // in deg f
int engine_temp_high = 140;   // in deg f


/*
  SPED and TACK variables
*/

#define COMPUTECOUNTSPERIOD 100 // how often in ms `compute_counts()` gets called

#define BUFFSIZE 100 // a rolling average of the frequency/period is computed, and this is the size of that buffer

#define NUMSIGS 2
#define TACKSIGINDEX 0
#define SPEDSIGINDEX 1

#define SPEDPULSESPERMILE 7000
#define MILESPERPULSE (float)1/(float)SPEDPULSESPERMILE

volatile int period_buffer_indices[NUMSIGS] = { 0 }; // the location of the index for adding to the rolling buffer average
volatile unsigned long period_buffers[NUMSIGS][BUFFSIZE] = { 0 }; // the buffers
volatile unsigned long previous_edge_times_us[NUMSIGS] = { 0 }; // the time that the previous edge came in in microseconds
volatile float period_averages_ms[NUMSIGS] = { 0 }; // the period time of a given signal in milliseconds
volatile float frequency_averages_hz[NUMSIGS] = { 0 }; // the frequency of a given signal in hertz
volatile bool period_buffer_locked[NUMSIGS] = { false }; // spin locks for the different buffers
unsigned long previous_compute_time_ms = 0;


/*
Odometer Variables Defenition
*/

float miles_this_trip = 0; 

/*
  Cruise Variables Setup
*/

#define SETCRUISEDELAY 1000
#define MAINTAINPULSEPERIOD 100

bool PB6_toggle_pressstarted = false;
unsigned long PB6_toggle_downtime = 0;
bool PB6_toggle_newpress = true;
bool PB6_toggle_state = false;
bool PB6_previous_toggle_state = true;

bool set_cruise_delay_started = false;
unsigned long set_cruise_start_time = 0;

bool maintain_pulse_started = 0;
unsigned long maintain_pulse_start_time = 0;

/*
  Define Display Variables, these are the things that get passed to the raspberry pi running `Display Software`
*/

float display_rpm = 0;
float display_mph = 0; 
float display_vbat = 0;
float display_batc = 0;
int display_ect = 0;
float display_oilp = 0;
float display_fuel = 0;
float display_egol = 0; 
float display_egor = 0;
byte display_ebrk = 0;
byte display_rvrs = 0;
int display_act = 0;
byte display_left = 0;
byte display_right = 0;
byte display_shutoff = false;
float display_miles_this_trip = 0; 
int display_state = 0;

void setup() {
  
  /*
    Set up serial ports
  */

  Serial.begin(115200);
  Serial3.begin(9600);

  /*
    Set Up Pins
  */

  // set initial pinModes
  int digital_pins[37]  = { ECF_EN_PIN,		STEN_EN_PIN,	RUEN_EN_PIN,	DIRL_EN_PIN,	DIRR_EN_PIN,	WIPEO_EN_PIN,	WIPEL_EN_PIN,	WIPEH_EN_PIN,	PSU1_EN_PIN,	PSU2_EN_PIN,	PSU3_EN_PIN,	SREG_SER_PIN,	SREG_RCLK_PIN,		SREG_SRCLK_PIN,		SREG_SRCLR_PIN,		SREG_OE_PIN,	MUX_S3_PIN,		MUX_S2_PIN,		MUX_S1_PIN,		MUX_S0_PIN,		MUX_EN_PIN,		CRZ_RESUME_PIN,		CRZ_DISABLE_PIN,	PB8_SIG_PIN,	PB8_BLED_PIN,		PB8_GLED_PIN,		PB8_RLED_PIN,		LED2_EN_PIN,		LED1_EN_PIN,		PSU1_SIG_PIN,		OILL_SIG_PIN,		CRZ_MAINTAIN_PIN,	SPED_SIG_PIN,	TACK_SIG_PIN,	HALL_SIG_PIN,	HALR_SIG_PIN,	debug_LED_en };
  int pin_modes[37]		= { OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			INPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,				OUTPUT,				OUTPUT,				OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,			OUTPUT,				OUTPUT,				INPUT,			OUTPUT,				OUTPUT,				OUTPUT,				OUTPUT,				OUTPUT,				INPUT,				INPUT,				OUTPUT,				INPUT_PULLUP,	INPUT_PULLUP,	INPUT_PULLUP,	INPUT_PULLUP,	OUTPUT };
  for (int i = 0; i < 37; i++) {
    pinMode(digital_pins[i], pin_modes[i]);
  }

  // set initial state
  int init_pins[12]       = { SREG_OE_PIN,  SREG_SRCLR_PIN,   WIPEO_EN_PIN, WIPEH_EN_PIN, WIPEL_EN_PIN, STEN_EN_PIN,  RUEN_EN_PIN,  ECF_EN_PIN,   WIPEO_EN_PIN, WIPEL_EN_PIN, WIPEH_EN_PIN, WIPEO_EN_PIN };
  int init_pin_states[12]     = { LOW,      HIGH,       LOW,      LOW,      LOW,      LOW,      LOW,      LOW,      HIGH,     LOW,      LOW,      LOW };
  for (int i = 0; i < 12; i++) {
    digitalWrite(init_pins[i], init_pin_states[i]);
  }

  // set up interrupts
  attachInterrupt(digitalPinToInterrupt(SPED_SIG_PIN), new_SPED_edge, RISING);
  attachInterrupt(digitalPinToInterrupt(TACK_SIG_PIN), new_TACK_edge, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_SIG_PIN), new_HALL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(HALR_SIG_PIN), new_HALR_isr, RISING);

  /*
    Run initilization functions for other devices
  */

  set_pb8_color(PB8BLACK);

  //reset all register pins
  clearRegisters();
  writeRegisters();

  disable_wipers();

  /*
    Run final setup tasks
  */

  debug_print("Setup Complete", true);

  transitionIntoState(MODECOMBO); // prep state machine functions
}

void loop() {

  /*
    Operations that happen every loop
  */
  
  // set the state of the global blink variables
  for (int i = 0; i < NUMBLINKVARS; i++) {
    if (millis() - blink_times[i] > blink_increments[i]) {
      blink_states[i] = !blink_states[i];
      blink_times[i] = millis();
    }
  }

  digitalWrite(debug_LED_en, blink_states[BLINKFASTEST]);

  // handle a shutoff event
  if (switchSetToOff() && killSwitchEnabled) {
    transitionIntoState(MODESHUTOFF);
  }


  // handle a state machine timeout
  if ((millis() - lastTransitionTime >= transitionTimeoutInterval) && (transitiontimeoutEnabled == true)) { // if the timeout 
    debug_print("Timeout in state: ", false);
    debug_print(state, true);
    transitionIntoState(MODECOMBO);
  }

  if ((state == MODEAUTOSTART) || (state == MODERUNNING) || (state == MODESHUTOFF)) {
    /*
    Blinkers
    */
    {
      for (int i = 0; i < NUMTOGGLEBUTTONSLEDS; i++) {
        if (read_mux_button(toggle_buttons[i])) {
          if (toggle_pressstarted[i] == false) {
            toggle_pressstarted[i] = true;
            toggle_downtimes[i] = millis();
          }
          if (millis() - toggle_downtimes[i] > PRESSTIME) {
            if (toggle_newpresses[i]) {
              toggle_states[i] = !toggle_states[i];
              toggle_newpresses[i] = false;
            }
          }
        }
        else {
          toggle_pressstarted[i] = false;
          toggle_newpresses[i] = true;
        }
      }

      left_turn_signal_state = (toggle_states[LEFTBLINKSTATE] || toggle_states[HAZARDBLINKSTATE]) && blink_states[BLINKSLOW];
      right_turn_signal_state = (toggle_states[RIGHTBLINKSTATE] || toggle_states[HAZARDBLINKSTATE]) && blink_states[BLINKSLOW];

      setAndWrite(toggle_button_LEDs[LEFTBLINKSTATE], left_turn_signal_state);
      setAndWrite(toggle_button_LEDs[RIGHTBLINKSTATE], right_turn_signal_state);
      setAndWrite(toggle_button_LEDs[HAZARDBLINKSTATE], toggle_states[HAZARDBLINKSTATE] && blink_states[BLINKSLOW]);

      digitalWrite(DIRL_EN_PIN, left_turn_signal_state);
      digitalWrite(DIRR_EN_PIN, right_turn_signal_state);
    }

    /*
    Wipers
    */
    {
      if (read_mux_button(PB4_SIG_PIN)) {
        debug_print("new press", true);
        if (looking_for_wiper_button) {
          looking_for_wiper_button = false;
          if (millis() - previous_wiperbutton_time > PRESSTIME) {
            debug_print("incrementing", true);
            previous_wiperbutton_time = millis();

            wiperstate++;
            if (wiperstate > WIPERHIGH)
            {
              wiperstate = WIPEROFF;
            }
          }
        }
      }
      else {
        looking_for_wiper_button = true;
      }

      switch (wiperstate) {

        case WIPEROFF:
        {
          disable_wipers();
        }
        break;

        case WIPERLOW:
        {
          digitalWrite(WIPEO_EN_PIN, LOW);
          digitalWrite(WIPEH_EN_PIN, LOW);
          digitalWrite(WIPEL_EN_PIN, HIGH);

          setAndWrite(PB4_LED_EN, blink_states[BLINKFAST]);
        }
        break;

        case WIPERHIGH:
        {
          digitalWrite(WIPEO_EN_PIN, LOW);
          digitalWrite(WIPEL_EN_PIN, LOW);
          digitalWrite(WIPEH_EN_PIN, HIGH);

          setAndWrite(PB4_LED_EN, blink_states[BLINKFASTEST]);
        }
        break;
      }
    }

    /*
    Engine Fan
    */
    {
      int engine_temp = get_temp_from_count(analogRead(ECT_SIG_PIN));

      if (engine_temp >= engine_temp_high) {
        digitalWrite(ECF_EN_PIN, HIGH);
      }

      if (engine_temp <= engine_temp_low) {
        digitalWrite(ECF_EN_PIN, LOW);
      }
    }
  }
  
  set_display_variables(false);

  // data block mapping
  db_f0 = display_rpm;    // works
  db_f1 = display_mph;    // works
  db_f2 = display_vbat;   // works 
  db_f3 = display_oilp;   // works
  db_f4 = display_fuel;   // works
  db_f5 = display_egol;   // works
  db_f6 = display_egor;   // works
  db_f7 = display_miles_this_trip; // untested
  db_f8 = display_batc;   // untested
  // db_f9 = ;
  // db_f10 = ;
  // db_f11 = ;
  
  db_i0 = display_ect;    // works
  db_i1 = display_act;    // works
  db_i2 = display_state;    // untested
  // db_i3 = ;

  db_b0 = display_ebrk;   // works
  db_b1 = display_rvrs;   // works
  db_b2 = display_shutoff;  // works
  db_b3 = display_left;   // works
  db_b4 = display_right;    // works
  // db_b5 = ;
  // db_b6 = ;  


  process_message(Serial3);

  /*
    The state machine
  */
  switch (state) {

    case MODECOMBO:
    {

      /*
        if user takes to long to enter a combo, reset the system
      */

      if ((millis() - lastPressTime >= pressTimeoutInterval) && (comboPos > 0)) {
        debug_print("Combo input timed out", true);
        restartCombo();
      }

      if (millis() - lastBounceTime >= debounceInterval) { // make sure enough time has passed between button presses

        lastBounceTime = millis();

        /*
          Process new button presses from the keypad
        */
        for (int buttonNumber = 0; buttonNumber < NUMKEYPADBUTTONS; buttonNumber++) {

          int buttonState = read_mux_button(keypadbuttons[buttonNumber]);

          // this ensures that the button is released before registering new presses
          if ((buttonState != lastStates[buttonNumber]) && (buttonState == 1)) {

            lastPressTime = millis();

            debug_print("Button press: ", false);
            debug_print(buttonNumber, false);
            debug_print(" Combo Position: ", false);
            debug_print(comboPos, true);

            if (addPress(buttonNumber)) {
              debug_print("good combo!", true);
              transitionIntoState(MODEAUTOSTART);
              break; // break out of the for loop if the right entry is made

            }
          }
          lastStates[buttonNumber] = buttonState;
        }
      }
    }
    break;

    case MODEAUTOSTART:
    {
      if (starting == false) {

        debug_print("Waiting for user to press start button", true);

        if (digitalRead(PB8_SIG_PIN) == HIGH) {
          debug_print("User has pressed start button!", true);
          starting = true;
          startTime = millis();
        }
      }

      if (starting == true) {

        debug_print("In starter mode", true);

        transitiontimeoutEnabled = false;
        killSwitchEnabled = false;

        digitalWrite(LED1_EN_PIN, LOW);

        debug_print("STEN, RUEN are both enabled", true);

        digitalWrite(RUEN_EN_PIN, HIGH);

        if (millis() - startTime > 2000) {
          digitalWrite(STEN_EN_PIN, HIGH);
        }

        set_pb8_color(pb8_colors[PB8BLUE]);

        if (is_car_running()) {
          
          debug_print("Car is running! that start took ", false);
          debug_print(millis() - startTime, false);
          debug_print("ms", true);
          digitalWrite(STEN_EN_PIN, LOW);
          transitionIntoState(MODERUNNING);
          auto_start_fail = false;
        }
        else {
          debug_print("Car is not running, OILP:", false);
          debug_print(analogRead(OILP_SIG_PIN), true);
        }

        if (millis() - startTime > startDuration) {
          debug_print("Automatic start attempt has timed out", true);
          transitionIntoState(MODERUNNING);
          auto_start_fail = true;
        }
      }
    }
    break;

    case MODERUNNING:
    {

      if (is_car_running()) {
        set_pb8_color(PB8GREEN);
      }
      else {
        set_pb8_color(PB8YELLOW);
      }

      /*
        Manual Start
      */
      {
        if (digitalRead(PB8_SIG_PIN) == HIGH) {
          digitalWrite(STEN_EN_PIN, HIGH);
          set_pb8_color(pb8_colors[PB8BLUE]);
        }

        else {
          digitalWrite(STEN_EN_PIN, LOW);

          if (auto_start_fail) {
            set_pb8_color(pb8_colors[PB8YELLOW]);
          }

          else {
            set_pb8_color(pb8_colors[PB8GREEN]);
          }
        }
      }

      /*
      Cruise Control
      */
      {
        // handle the toggle button
        if (read_mux_button(PB6_SIG_PIN)) {
          if (PB6_toggle_pressstarted == false) {
            PB6_toggle_pressstarted = true;
            PB6_toggle_downtime = millis();
          }
          if (millis() - PB6_toggle_downtime > PRESSTIME) {
            if (PB6_toggle_newpress) {
              PB6_toggle_state = !PB6_toggle_state;
              PB6_toggle_newpress = false;
            }
          }
        }
        else {
          PB6_toggle_pressstarted = false;
          PB6_toggle_newpress = true;
        }

        if ((PB6_toggle_state != PB6_previous_toggle_state)) { // make sure that this is only on toggle changes
          if (PB6_toggle_state) {
            set_cruise_delay_started = true;
            set_cruise_start_time = millis();

            digitalWrite(CRZ_DISABLE_PIN, HIGH);
            setAndWrite(PB6_LED_EN, HIGH);
          }
          else {
            set_cruise_delay_started = false;

            digitalWrite(CRZ_DISABLE_PIN, LOW);
            setAndWrite(PB6_LED_EN, LOW);
          }

          PB6_previous_toggle_state = PB6_toggle_state;
        }

        if (set_cruise_delay_started) {
          if (millis() - set_cruise_start_time > SETCRUISEDELAY) {

            if (maintain_pulse_started == false) {
              maintain_pulse_start_time = millis();
              digitalWrite(CRZ_MAINTAIN_PIN, HIGH);
              maintain_pulse_started = true;
            }
            else {
              if (millis() - maintain_pulse_start_time > MAINTAINPULSEPERIOD) {
                maintain_pulse_started = false;
                digitalWrite(CRZ_MAINTAIN_PIN, LOW);
                setAndWrite(PB6_LED_EN, HIGH);
                set_cruise_delay_started = false;
              }
            }
          }
        }

      }

    }
    break;

    case MODESHUTOFF:
    {

      digitalWrite(RUEN_EN_PIN, LOW);

      debug_print("Waiting for Pi to shut off", true);

      if (ready_for_shutoff()) {

        // make sure all UI LEDs are disabled
        for (int i = 0; i < NUMKEYPADBUTTONS; i++) {
          setRegisterPin(keypadLEDs[i], LOW);
        }

        writeRegisters();

		for (int i = 0; i < 5; i++) {
			digitalWrite(LED1_EN_PIN, HIGH);
			delay(50);
			digitalWrite(LED1_EN_PIN, LOW);
			delay(50);
		}

        digitalWrite(LED1_EN_PIN, LOW);
        digitalWrite(LED2_EN_PIN, LOW);

        debug_print("Raspberry Pi shutoff signal received or timeout occured", true);

        digitalWrite(PSU2_EN_PIN, LOW);

        turnoffstarter(); // kill self

        delay(1000);

        // should never get here, but for debugging it is fine
        transitionIntoState(MODECOMBO);
      }
    }
    break;

    case MODELIMBO:
    {
      debug_print("In Limbo, shouldn't be here for very long", true);
    }
    break;

    default:
    {
      debug_print("bad state", true);
    }
    break;

  }
}

/*
  State Machine Functions
*/

void transitionIntoState(int toState) {

  lastTransitionTime = millis();

  debug_print("Going to state: ", false);
  debug_print(toState, false);
  debug_print(" From: ", false);
  debug_print(state, true);

  state = toState; // set the global state varaible

  switch (toState) {

    case MODECOMBO: 
    {

      transitiontimeoutEnabled = false;
      killSwitchEnabled = true;
      starting = false;
      shutting_off = false;

      set_pb8_color(pb8_colors[PB8BLACK]);

      for (int i = 0; i < NUMKEYPADLEDS; i++) {
        setRegisterPin(keypadLEDs[i], LOW);
      }

      writeRegisters();

      digitalWrite(ECF_EN_PIN, LOW);

      digitalWrite(LED1_EN_PIN, LOW);
      digitalWrite(LED2_EN_PIN, HIGH);
    
      digitalWrite(RUEN_EN_PIN, LOW);
      digitalWrite(STEN_EN_PIN, LOW);

      digitalWrite(PSU2_EN_PIN, HIGH);

      disable_wipers();

      restartCombo();
    }
    break; 

    case MODEAUTOSTART:
    {
      transitiontimeoutEnabled = false;
      killSwitchEnabled = true;
      starting = false;
      shutting_off = false;

      set_pb8_color(pb8_colors[PB8YELLOW]);

      digitalWrite(ECF_EN_PIN, LOW);

	  digitalWrite(LED1_EN_PIN, LOW);
	  digitalWrite(LED2_EN_PIN, HIGH);
    
      digitalWrite(RUEN_EN_PIN, LOW);
      digitalWrite(STEN_EN_PIN, LOW);

      digitalWrite(PSU2_EN_PIN, HIGH);
    }
    break;

    case MODERUNNING:
    {
      transitiontimeoutEnabled = false;
      killSwitchEnabled = true;
      starting = false;
      shutting_off = false;

      set_pb8_color(pb8_colors[PB8GREEN]);

      digitalWrite(ECF_EN_PIN, LOW);

      digitalWrite(LED1_EN_PIN, HIGH);
      digitalWrite(LED2_EN_PIN, LOW);
    
      digitalWrite(RUEN_EN_PIN, HIGH);
      digitalWrite(STEN_EN_PIN, LOW);

      digitalWrite(PSU2_EN_PIN, HIGH);
    }
    break;

    case MODESHUTOFF:
    {
      transitiontimeoutEnabled = false;
      killSwitchEnabled = false;
      starting = false;
      shutting_off = true;

      set_pb8_color(pb8_colors[PB8BLACK]);

      digitalWrite(ECF_EN_PIN, LOW);

      digitalWrite(LED1_EN_PIN, HIGH);
	  digitalWrite(LED2_EN_PIN, LOW);
    
      digitalWrite(RUEN_EN_PIN, LOW);
      digitalWrite(STEN_EN_PIN, LOW);
    
      digitalWrite(PSU2_EN_PIN, HIGH);

      startShutoffTime = millis(); // start the shutoff timer
    }
    break;

    case MODELIMBO:
    {
      debug_print("Waiting for reset", true);

      transitiontimeoutEnabled = true;

      set_pb8_color(pb8_colors[PB8BLACK]);
      
      /*
        Do nothing, the actual program should never do this, only when connected to the debugger code
      */
    }
    break;

    default:
    {
      debug_print("Transitioning into bad state", true);
    }
    break;
  }

  debug_print("Global State Variables", true);

  debug_print("Transition Timeout Enabled: ", false);
  debug_print(transitiontimeoutEnabled, true);

  debug_print("Kill Switch Enabled: ", false);
  debug_print(killSwitchEnabled, true);

  debug_print("Starting: ", false);
  debug_print(starting, true);

  debug_print("Shutting off: ", false);
  debug_print(shutting_off, true);

}

/*
  Combo Pad Functions
*/

void restartCombo(void) {
  comboPos = 0;
  for (int i = 0; i < NUMKEYPADLEDS; i++) {
    setRegisterPin(keypadLEDs[i], HIGH);
  }
  writeRegisters();
}

bool addPress(int buttonNumber) {

  setAndWrite(keypadLEDs[buttonNumber], LOW);

  comboBuffer[comboPos] = buttonNumber; // global array
  comboPos++; // global int

  if (comboPos == COMBOLENGTH) {

    for (int i = 0; i < COMBOLENGTH; i++) {

      bool correct = comboBuffer[i] == goodCombo[i];

      if (!correct) {
        restartCombo();
        return false;
      }
    }

    for (int i = 0; i < NUMKEYPADLEDS; i++) {
      setRegisterPin(keypadLEDs[i], LOW);
    }
    writeRegisters();

    return true;
  }
  return false;
}

/*
  Serial Communication Functions
*/

void send_data_block(Stream &port) {
  pack_data_block();
  for (int i = 0; i < BLOCKSIZE; i++) {
    port.write(data_block[i]);
  }
}

void process_message(Stream &port) {
  if (port.available() > 0) {

	debug_print("Message Requested", true);

    char b = port.read();

    switch (b) {
    case '0':
      break;

    case '1':
      break;
    }

    send_data_block(port);
    port.flush();

	debug_print("Data Block Sent", true);
  }
}

void set_display_variables(bool print_debug) {

  compute_rpm_mph(); // this will set `display_rpm` and `display_mph`

  float vbat_tweak = 0;
  float batc_tweak = 0; 

  float vbat_in_12v = adc_counts_to_volts(analogRead(VBAT_SIG_PIN)) * ((float)(51 + 100)) / ((float)51) + vbat_tweak; // magic number comes from the ratio of the divider
  float batc_in_12v = adc_counts_to_volts(analogRead(BATC_SIG_PIN)) * ((float)(100 + 105))/((float)105) + batc_tweak; // magic number comes from the ratio if this devices divider
  
  float batc = (float)vbat_in_12v - (float)batc_in_12v;

  display_vbat = vbat_in_12v;
  display_ect = get_temp_from_count(analogRead(ECT_SIG_PIN));
  display_act = get_temp_from_count(analogRead(ACT_SIG_PIN));
  display_oilp = map(analogRead(OILP_SIG_PIN), 1023, 200, 0, 60);
  display_fuel = map(analogRead(FUEL_SIG_PIN), 353, 799, 0, 100);
  display_egol = adc_counts_to_volts(analogRead(EGOL_SIG_PIN)) / 3.32558139535; // magic number is from gain of amplfier
  display_egor = adc_counts_to_volts(analogRead(EGOR_SIG_PIN)) / 3.32558139535; // magic number is from gain of amplfier
  display_ebrk = (analogRead(EBRK_SIG_PIN) > 10);
  display_rvrs = !ar_to_dr(analogRead(RVRS_SIG_PIN));
  display_left = left_turn_signal_state;
  display_right = right_turn_signal_state; 
  display_shutoff = shutting_off; 
  display_miles_this_trip = miles_this_trip;
  display_batc = batc;
  display_state = state; 

  if (print_debug) {
	
    debug_print("VBAT: ", false);
    debug_print(display_vbat, false);
    debug_print(" volts", true);

	debug_print("BATC ", false);
	debug_print(display_batc, false);
	debug_print(" volts ", false);
	debug_print(batc_in_12v, false);
	debug_print(" in 12v ", false);
	debug_print(analogRead(BATC_SIG_PIN), false);
	debug_print(" counts", true);

    debug_print("E Break: ", false);
    debug_print(display_ebrk, false);
    debug_print(" bool counts", false);
    debug_print(analogRead(EBRK_SIG_PIN), true);

    debug_print("Trip miles: ", false);
    debug_print(miles_this_trip, true);

    debug_print("OILP: ", false);
    debug_print(display_oilp, false);
    debug_print(" pounds", true);

    debug_print("FUEL: ", false);
    debug_print(display_fuel, false);
    debug_print(" % ", false);
    debug_print(analogRead(FUEL_SIG_PIN), false);
    debug_print(" counts", true);

    debug_print("EGOL: ", false);
    debug_print(display_egol, false);
    debug_print(" counts", true);

    debug_print("EGOR: ", false);
    debug_print(display_egor, false);
    debug_print(" counts", true);

    debug_print("Reverse: ", false);
    debug_print(display_rvrs, false);
    debug_print(" bool", true);

  }

}


/*
  Interrupt Service Routines and ISR helpers
*/

void new_edge(int period_index) {

  unsigned long current = micros();

  if (period_buffer_locked[period_index] == false) { // if compute_counts is using the buffer, skip adding to it because that process isn't atomic

    period_buffer_locked[period_index] = true;

    period_buffers[period_index][period_buffer_indices[period_index]] = current - previous_edge_times_us[period_index];

    period_buffer_locked[period_index] = false;

    period_buffer_indices[period_index]++;
    if (period_buffer_indices[period_index] >= BUFFSIZE) {
      period_buffer_indices[period_index] = 0;
    }
  }

  previous_edge_times_us[period_index] = current; // but make sure the new time is set because this operation is atomic

}

void new_TACK_edge() {
  new_edge(TACKSIGINDEX);
}

void new_SPED_edge() {
  miles_this_trip += MILESPERPULSE; // increment the odometer count
  new_edge(SPEDSIGINDEX);
}

void new_hal_edge() {

  if ((waiting_for_left == false) && (waiting_for_right == false)) {
    if (right_time < left_time) { // A right turn
      if (toggle_states[LEFTBLINKSTATE]) {
        toggle_states[LEFTBLINKSTATE] = false;
      }
    }
    else if (right_time > left_time) { // A left turn
      if (toggle_states[RIGHTBLINKSTATE]) {
        toggle_states[RIGHTBLINKSTATE] = false;
      }
    }

    waiting_for_left = true;
    waiting_for_right = true;

  }
}

void new_HALL_isr() {
  left_time = millis();
  waiting_for_left = false;
  new_hal_edge();
}

void new_HALR_isr() {
  right_time = millis();
  waiting_for_right = false;
  new_hal_edge();
}

/*
  Helper Functions
*/

void disable_wipers(void) {
  digitalWrite(WIPEL_EN_PIN, LOW);
  digitalWrite(WIPEH_EN_PIN, LOW);
  digitalWrite(WIPEO_EN_PIN, HIGH);
  setAndWrite(PB4_LED_EN, LOW);
}

bool ready_for_shutoff(void) {
  bool timeout = ( (shutting_off) && (millis() - startShutoffTime >= shutoffTimeInterval));
  return (isPiOff() || timeout);
}

float adc_counts_to_volts(int counts) {
  return (((float)counts * (float)5) / (float)1024);
}

void print_engine_speed(void) {
  debug_print("TACK: ", false);
  debug_print(period_averages_ms[TACKSIGINDEX], false);
  debug_print("ms, ", false);
  debug_print(frequency_averages_hz[TACKSIGINDEX], false);
  debug_print("hz", false);
  debug_print(" - SPED: ", false);
  debug_print(period_averages_ms[SPEDSIGINDEX], false);
  debug_print("ms, ", false);
  debug_print(frequency_averages_hz[SPEDSIGINDEX], false);
  debug_print("hz - RPM: ", false);
  debug_print(display_rpm, false);
  debug_print(" - MPH: ", false);
  debug_print(display_mph, true);
}

float period_ms_to_rpm(float ms) {
  return (float)15 / (ms / (float)1000); 
}

float period_ms_to_mph(float ms) {
  return (((float)1 / (float)SPEDPULSESPERMILE)*((float)1 / (float)ms*(float)1000)*((float)3600));
}

void compute_rpm_mph(void) {
  // computes the average of the buffer for a given signal. Must be called before using the period_averages_ms or frequency_averages_hz buffers.

  for (int p_index = 0; p_index < NUMSIGS; p_index++) {

    float buffer_sum = 0;

    while (period_buffer_locked[p_index]) {}; // wait around for the ISR to finish

    period_buffer_locked[p_index] = true; // ISR won't add new data to `period_buffers`
    if ((micros() - previous_edge_times_us[p_index]) < 1000000) {
      for (int j = 0; j < BUFFSIZE; j++) {
        buffer_sum += period_buffers[p_index][j];
      }
    }
    period_buffer_locked[p_index] = false; // ISR will now add new data to `period_buffers`

    if (buffer_sum > 0) {
      period_averages_ms[p_index] = ((buffer_sum / (float)BUFFSIZE)) / 1000;
      frequency_averages_hz[p_index] = (1 / period_averages_ms[p_index]) * 1000;
    }
    else {
      period_averages_ms[p_index] = 0;
      frequency_averages_hz[p_index] = 0;
    }
  }

  display_rpm = period_ms_to_rpm(period_averages_ms[TACKSIGINDEX]);
  display_mph = period_ms_to_mph(period_averages_ms[SPEDSIGINDEX]);

}

void set_pb8_color(int color) {
  
  switch (color) {
    case PB8BLACK:
    {
      digitalWrite(PB8_RLED_PIN, HIGH);
      digitalWrite(PB8_GLED_PIN, HIGH);
      digitalWrite(PB8_BLED_PIN, HIGH);
    }
    break;

    case PB8RED:
    {
      digitalWrite(PB8_RLED_PIN, LOW);
      digitalWrite(PB8_GLED_PIN, HIGH);
      digitalWrite(PB8_BLED_PIN, HIGH);
    }
    break;
  
    case PB8GREEN:
    {
      digitalWrite(PB8_RLED_PIN, HIGH);
      digitalWrite(PB8_GLED_PIN, LOW);
      digitalWrite(PB8_BLED_PIN, HIGH);
    }
    break;

    case PB8BLUE:
    {
      digitalWrite(PB8_RLED_PIN, HIGH);
      digitalWrite(PB8_GLED_PIN, HIGH);
      digitalWrite(PB8_BLED_PIN, LOW);
    }
    break;

    case PB8YELLOW:
    {
      analogWrite(PB8_RLED_PIN, 100);
      analogWrite(PB8_GLED_PIN, 0);
      analogWrite(PB8_BLED_PIN, 0);
    }
    break;
  }
}

bool is_car_running(void) {

  debug_print("Checking if car is running using Oil Pressure", true);

  bool output = false;

  int oilp_reading = analogRead(OILP_SIG_PIN);
  bool good_oilp = (oilp_reading < 500);
  
  debug_print("OILP Reading: ", false);
  debug_print(oilp_reading, true);
  debug_print("OILP Good Reading: ", false);
  debug_print(good_oilp, true);
  debug_print("OILP Timer Started? ", false);
  debug_print(oilp_timer_started, true);

  if (good_oilp) {
    if (oilp_timer_started == false) {
      oilp_high_time = millis();
      oilp_timer_started = true;
    }
  }
  else {
    if (oilp_timer_started) {
      oilp_timer_started = false;
    }
  }

  if ( (oilp_timer_started) && (millis() - oilp_high_time > OILPHIGHTIMECARON ) ) {
    debug_print("OILP has been high enough for long enough, car is started!", true);
    oilp_timer_started = false;
    output = true;
  }

  debug_print("Is Car Running: ", false);
  debug_print(output, true);

  return output;

}

void turnoffstarter(void) {
  debug_print("Turning off starter", true);
  pinMode(PSU1_EN_PIN, OUTPUT);
  digitalWrite(PSU1_EN_PIN, LOW);
}

boolean isPiOff(void) {
  // TODO
  return false;
}

bool switchSetToOff() {
  return !(bool)digitalRead(PSU1_SIG_PIN);
}

int get_temp_from_count(int count) {
  // return the engine temp in degrees F, this is read out from a table

  float vref = (float)5 / ((float)4.97 / (float)4.90); // measured this ratio
  float engine_temp_voltage = ((count * vref) / (float)1024);

  float biggest_difference = INT_MAX;
  int biggest_difference_index;

  for (int i = 0; i < NUMECTTABLEVALUES; i++) {
    float lookup_voltage = ECT_table_volts[i];

    float difference = abs((float)engine_temp_voltage - (float)(lookup_voltage));

    if (difference < biggest_difference) {
      biggest_difference = difference;
      biggest_difference_index = i;
    }
  }

  float closest_engine_volts = ECT_table_volts[biggest_difference_index];
  int closest_engine_temp = ECT_table_temps[biggest_difference_index];

  /*
  debug_print("ECT count: ", false);
  debug_print(engine_temp_count, false);
  debug_print(" volts: ", false);
  debug_print(engine_temp_voltage, false);
  debug_print(" closest to: ", false);
  debug_print(closest_engine_volts, false);
  debug_print(" volts -> ", false);
  debug_print(closest_engine_temp, false);
  debug_print(" degrees F", true);
  */

  return closest_engine_temp;
}


/*
  Shift Register / Multiplexer Functions
*/

void setAndWrite(int index, int value) {
  setRegisterPin(index, value);
  writeRegisters();
}

void clearRegisters() {
  for (int i = numOfRegisterPins - 1; i >= 0; i--) {
    registers[i] = LOW;
  }
}

void writeRegisters() {

  digitalWrite(SREG_RCLK_PIN, LOW);

  for (int i = numOfRegisterPins - 1; i >= 0; i--) {

    digitalWrite(SREG_SRCLK_PIN, LOW);

    int val = registers[i];

    digitalWrite(SREG_SER_PIN, val);
    digitalWrite(SREG_SRCLK_PIN, HIGH);
  }
  digitalWrite(SREG_RCLK_PIN, HIGH);
}

void setRegisterPin(int index, int value) {
  //set an individual pin HIGH or LOW

  registers[index] = value;
}

bool ar_to_dr(int ar_value) {
  // turns an analog read to a binary digital value

  if (ar_value > 512) {
    return true;
  }
  else {
    return false;
  }
}

int read_mux(int channel) {

  int controlPin[] = { MUX_S0_PIN, MUX_S1_PIN, MUX_S2_PIN, MUX_S3_PIN };

  int muxChannel[16][4] = {
    { 0,0,0,0 }, //channel 0
    { 1,0,0,0 }, //channel 1
    { 0,1,0,0 }, //channel 2
    { 1,1,0,0 }, //channel 3
    { 0,0,1,0 }, //channel 4
    { 1,0,1,0 }, //channel 5
    { 0,1,1,0 }, //channel 6
    { 1,1,1,0 }, //channel 7
    { 0,0,0,1 }, //channel 8
    { 1,0,0,1 }, //channel 9
    { 0,1,0,1 }, //channel 10
    { 1,1,0,1 }, //channel 11
    { 0,0,1,1 }, //channel 12
    { 1,0,1,1 }, //channel 13
    { 0,1,1,1 }, //channel 14
    { 1,1,1,1 }  //channel 15
  };

  //loop through the 4 sig
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  int val = analogRead(MUX_SIG_PIN);

  return val;
}

int read_mux_button(int channel) {
  return ar_to_dr(read_mux(channel));
}

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
