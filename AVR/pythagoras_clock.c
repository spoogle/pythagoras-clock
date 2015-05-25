/*
  Interrupt-driven clock with output to two pythagorean triangles.

  Copyright (c) 2010, 2014, 2015 Julian Richardson.

  Licensed under the MIT license. See LICENSE.TXT 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define TIMER_CONTROL_REG TCCR0B

#if __AVR_ATtiny2313__
#define TIMER_MASK_REF TIMSK
#define TIMER_VECT TIMER0_OVF_vect
#elif __AVR_ATtiny13__
#define TIMER_MASK_REF TIMSK0
#define TIMER_VECT TIM0_OVF_vect
#elif __AVR_ATmega644__ || __AVR_ATmega168__
#define TIMER_MASK_REF TIMSK0
#define TIMER_VECT TIMER0_OVF_vect
#endif

/*
  B = minutes % 5, PD0..4
  G = minutes / 5 % 3, PD5..7 
  R = minutes /15 % 4, PB0..3

  Y = hours % 3, PC0..2
  W = hours / 3 % 4, PC3..4, PB4..5

  Button = PC5, connected to ground through 1K resistor

       4W
    -- -- -- -- 
  | \           |
       \          
3G|   5B \      |3Y
           \
  |           \ |
    -- -- -- -- 
       4R
*/

// count interrupts
unsigned int t, input, state, nticks_passed, toincr, interrupts_per_tick, 
  time_since_last_button_press;

// disregard button value for k/3906 seconds after press
// to debounce
#define DEBOUNCE_TICKS 100
// 1000000/256 = 3906.25
#define TICKS_NORMAL 3906
#define TICKS_FAST 10
// BUTTON_LONG = 1-2 seconds
#define BUTTON_LONG 3906
// BUTTON_V_LONG = 2+ seconds
#define BUTTON_V_LONG 7812
#define INCR_MINUTES 0
#define INCR_HOURS 1

// maximum time to wait for a button press before going back
// to initial state
#define MAX_BUTTON_WAIT 65000

// States for button press state machine.
#define STATE_START 0
#define STATE_DEBOUNCE 1
#define STATE_COUNT_DEPRESS_TICKS 2
#define STATE_SHORT_PRESS 3
#define STATE_LONG_PRESS 4
#define STATE_VLONG_PRESS 5

// seconds, hours - separate to avoid integer overflow if using
// only seconds
int s, h;

// minutes, time in units if 5, 15, 60, 180 minutes
short int min, min5, min15, hour, hour3;

// since there are 3906.25 interrupts per second, we add a leap tick
// to the number of interrupts in a second once every four
// seconds
short int leaptick;


int main (void)
{
  DDRB = 0b00111111; // PB0..5 outputs
  DDRD = 0b11111111; // PD0..7 outputs
  DDRC = 0b00011111; // PC0..4 outputs

  // PortB, C, D outputs all low - LEDs off
  // internal pull-ups enabled on all input pins,
  // in particular PC5, which is connected to ground through
  // button
  PORTB = 0b11000000;
  PORTD = 0b00000000;
  PORTC = 0b11100000;

  // Timer clock is I/O clock/8
  TIMER_CONTROL_REG |= _BV(CS01);
  
  // generate interrupt on timer overflow
  TIMER_MASK_REF |= _BV(TOIE0);

  // reset timer
  TCNT0 = 0;

  input = state = nticks_passed = 0;
  interrupts_per_tick = TICKS_NORMAL;
  toincr  = INCR_MINUTES;

  // trigger calculation of digits on first interrupt
  t = interrupts_per_tick - 1;

  // current time: 9:00
  s = 0;
  h = 9;

  // enable interrupts
  SREG |= _BV(SREG_I);

  time_since_last_button_press = 1;

  // nothing to do but service interrupts
  while(1) 
    sleep_mode();

  return 0;
}

ISR(TIMER_VECT) {
  // start with timer=6, will overflow after 250 cycles
  // TCNT0 = 6;

  // every fourth second a second requires an extra tick, so that
  // the number of ticks in a second on average is 3906.25
  if ((s % 4) == 0) leaptick = 1;
  else leaptick = 0;

  // increment t, will hit TICKS_NORMAL once every second
  // (1MHz I/O clock / 256 timer steps to interrupt / 3906.25 
  // interrupts = 1 sec)
  if (t++ == interrupts_per_tick + leaptick) {
    t = 0;
    s = (s + 1) % (60*60);
    if (s == 0) h = ((h + 1) % 12);
    min = (s / 60) % 5;
    min5 = (s / (5*60)) % 3;
    min15 = (s / (15*60)) % 4;
    hour = h  % 3;
    hour3 = (h / 3) % 4;
    
    PORTD = (1 << min) | (1 << (min5+5));
    if (hour3 < 2) {
      PORTC = 0b11100000 | (1 << hour) | (1 << (3 + hour3));
      PORTB = 0b11000000 | (1 << min15);
    }
    else {
      PORTC = 0b11100000 | (1 << hour);
      PORTB = 0b11000000 | (1 << min15) | (1 << (2 + hour3));
    }
  }

  // get button value: depressed = 0, non-depressed > 0
  input = PINC & 0b00100000;

  // reinitialize counter whenever button pressed
  if (input == 0) 
    time_since_last_button_press = 0;

  time_since_last_button_press++;

  // reinitialize state if time_since_last_button_press has
  // reached maximum
  if (time_since_last_button_press == MAX_BUTTON_WAIT) {
    state = 0;
    toincr = INCR_MINUTES;
    time_since_last_button_press = 0;
  }

  // state machine to debounce and process button presses:
  // short press = incremement hour or minute
  // long press = switch between hours or minutes for incrementing
  // very long press = demo mode fast clock
  switch (state) {
  case STATE_START:
    if (input == 0) { 
      // button depressed
      nticks_passed = 0; 
      state = STATE_DEBOUNCE; 
    }
    break;
  case STATE_DEBOUNCE:
    // wait for k ticks
    if (nticks_passed++ == DEBOUNCE_TICKS) {
      state = STATE_COUNT_DEPRESS_TICKS;
      nticks_passed = 0;
    }
    break;
  case STATE_COUNT_DEPRESS_TICKS:
    // count ticks until input > 0 i.e. button no longer depressed
    nticks_passed++;
    if (input > 0 && nticks_passed < BUTTON_LONG) {
        state = STATE_SHORT_PRESS;
    } else if (input > 0 && nticks_passed < BUTTON_V_LONG) {
        state = STATE_LONG_PRESS;
    } else if (input > 0) {
        state = STATE_VLONG_PRESS;
    }
    break;
  case STATE_SHORT_PRESS:
    // short press - increment hour or minute
    // ensure LED state will be updated on next interrupt
    // so that feedback to button release is immediate
    t = interrupts_per_tick - 1;

    if (toincr == INCR_MINUTES) {
	// add 59 seconds - the 60th will be added on the next tick
	s = s + 59; 
	if (s >= 3600) {
            s = s - 3600;
            h = (h + 1) % 12;
	}
    }
    else h = (h + 1) % 12;
    state = STATE_START;
    break;
  case STATE_LONG_PRESS:
    // long press - toggle between increment hour/minute
    toincr = (toincr == INCR_MINUTES) ? INCR_HOURS : INCR_MINUTES;
    state = STATE_START;
    break;
  case STATE_VLONG_PRESS:
    // very long press - switch between normal and fast clock
    interrupts_per_tick = 
        (interrupts_per_tick == TICKS_NORMAL) ? TICKS_FAST : TICKS_NORMAL;
    state = STATE_START;
    break;
  }
}


