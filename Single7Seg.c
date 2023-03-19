// IDEE zur Verallgmeinerung: C++ Klassen verwenden

/*
 * Dieses Modul dient der Anstreuerung einer einzigen 7-Segment-Anzeige ohne Punkt (DP).
 * Es ist spezifisch für das Projekt "Brutgeraet_V2"
 * Aktuell wird eine Anzeige mit gemeinsamer Kathode (Minuspol) verwendet.
 * 7-Segment-Anzeigen sind folgendermaßen aufgebaut:
		   __A__
		 /	   /
		F	  B
	   /__G__/
	  /		/
	 E	   C
	/__D__/	   _DP
	
 */



#include <avr/io.h>
#include <stdint.h>

#include "Single7Seg.h"

/*
#define A_PIN	PIND7
#define B_PIN	PIND6
#define C_PIN	PINB3
#define D_PIN	PINB2
#define E_PIN	PINB4
#define F_PIN	PINB1
#define G_PIN	PINB0
//#define DP_PIN

#define A_PORT	PORTD
#define B_PORT	PORTD
#define C_PORT	PORTB
#define D_PORT	PORTB
#define E_PORT	PORTB
#define F_PORT	PORTB
#define G_PORT	PORTB
//#define DP_PORT
*/

typedef enum {	// insbesondere fuer gemeinsame Kathode (Minuspol) bzw. gemeinsame Anode (Pluspol)
	KATHODE,
	ANODE
} beschaltung_t;

/*
 * Kathode -> Active-High -> bei High-Pegel leuchted die LED
 * Anode -> Active-Low -> bei Low-Pegel leuchtet die LED 
 */

static const beschaltung_t beschaltung = KATHODE;

static uint8_t setDP = 0;

static volatile uint8_t * seg_ddrx[] = {
	&DDRD,	// A_SEG
	&DDRD,	// B_SEG
	&DDRB,	// C_SEG
	&DDRB,	// D_SEG
	&DDRB,	// E_SEG
	&DDRB,	// F_SEG
	&DDRB,	// G_SEG
	&DDRC	// DP_SEG
};

static volatile uint8_t * seg_ports[] = {
	&PORTD,	// A_SEG
	&PORTD,	// B_SEG
	&PORTB,	// C_SEG
	&PORTB,	// D_SEG
	&PORTB,	// E_SEG
	&PORTB,	// F_SEG
	&PORTB,	// G_SEG
	&PORTC	// DP_SEG
};

static uint8_t seg_pins[] = {
	PIND7,	// A_SEG
	PIND6,	// B_SEG	
	PINB3,	// C_SEG
	PINB2,	// D_SEG
	PINB4,	// E_SEG
	PINB1,	// F_SEG
	PINB0,	// G_SEG
	PINC0	// DP_SEG
};

static void init(void) {
	static uint8_t checkInit = 0;
	
	if (checkInit == 0) {
		checkInit = 1;
		if (beschaltung == KATHODE) {
			for (uint8_t i = 0; i < 8; ++i) {
				* (seg_ddrx[i]) |= (1 << seg_pins[i]);
				* (seg_ports[i]) &= ~(1 << seg_pins[i]);
			}
		} else {
			for (uint8_t i = 0; i < 8; ++i) {
				* (seg_ddrx[i]) |= (1 << seg_pins[i]);
				* (seg_ports[i]) |= (1 << seg_pins[i]);
			}
		}
	}
}

void single_7seg_off(void) {
	init();
	
	if (beschaltung == KATHODE) {
		for (uint8_t i = 0; i < 8; ++i) {
			* (seg_ddrx[i]) |= (1 << seg_pins[i]);
			* (seg_ports[i]) &= ~(1 << seg_pins[i]);
		}
	} else {
		for (uint8_t i = 0; i < 8; ++i) {
			* (seg_ddrx[i]) |= (1 << seg_pins[i]);
			* (seg_ports[i]) |= (1 << seg_pins[i]);
		}
	}
}

void setDP_general(void) {
	setDP = 1;
}

void unsetDP_general(void) {
	setDP = 0;
}

uint8_t single_7seg_number(uint8_t digit) {
	init();
	
	// Coding for kathode
	uint8_t mask;
	if (digit == 0) {
		// A, B, C, D, E, F
		mask = 0b11111100;
	} else if (digit == 1) {
		// B, C
		mask = 0b01100000;
	} else if (digit == 2) {
		// A, B, D, E, G
		mask = 0b11011010;
	} else if (digit == 3) {
		// A, B, C, D, G
		mask = 0b11110010;
	} else if (digit == 4) {
		// B, C, F, G
		mask = 0b01100110;
	} else if (digit == 5) {
		// A, C, D, F, G
		mask = 0b10110110;
	} else if (digit == 6) {
		// A, C, D, E, F, G
		mask = 0b10111110;
	} else if (digit == 7) {
		// A, B, C
		mask = 0b11100000;
	} else if (digit == 8) {
		// A, B, C, D, E, F, G
		mask = 0b11111110;
	} else if (digit == 9) {
		// A, B, C, D, F, G
		mask = 0b11110110;
	} else {
		// invalid number bigger than 9
		return -1;
	}
	
	if (setDP) {
		mask |= 0b00000001;
	}
	
	if (beschaltung == KATHODE) {
		for (uint8_t i = 0; i < 8; ++i) {
			if ( (mask & (1 << (7-i))) == 0b0000000 ) {
				* (seg_ports[i]) &= ~(1 << seg_pins[i]);	// LOW -> off
			} else {
				* (seg_ports[i]) |= (1 << seg_pins[i]);		// HIGH -> on
			}
		}
		
	} else {
		for (uint8_t i = 0; i < 8; ++i) {
			if ( (mask & (1 << (7-i))) == 0b0000000 ) {
				* (seg_ports[i]) |= (1 << seg_pins[i]);		// HIGH -> off
				} else {
				* (seg_ports[i]) &= ~(1 << seg_pins[i]);	// LOW -> on
			}
		}
	}
	// success
	return 0;
}