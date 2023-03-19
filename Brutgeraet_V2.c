/*
 * Das ist Version 2 des Brutgeräts.
 * Der Motor läuft pausiert stehts je im 15-Minuten-Takt, unabhängig vom der gesamten Schrittzahl pro Zyklus.
 * Nach einmaligem Vorwärts-laufen und 15-minütiger Pause läuft er zur Startposition zurück.
 * Mit zwei Tastern kann die Schrittweite pro Zylkus erhöhrt oder erniedrigt werden.
 * Diese wird per einstelliger 7-Segment-Anzeige als Stufen dargestellt.
 * 7-Seg. schaltet sich nach vorgegebener Zeit ab und wird eingeschaltet durch beliebigen Tastendruck, 
 * welcher zu diesem Zeitpunkt keine Änderung bewirkt.
 * Wahlweise ist möglich die voreingestellten Taktdauer des aktiven bzw. passiven Zustandes zu ändern.
 * Schließlich sollte die Schrittzahl eines ganzen Umlaufs des Motors vorgegeben werden. Beachte dafür die 'Microsteps'.
 */


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "Single7Seg.h"

/* --- Setzte hier gewünschte Parameter:		--- */
	/* Aktiver/passiver Zyklus in Millisekunden -- 1 min = 60 s = 60 000 ms	=> 15 min = 900 000 ms
	 * Eine Umdrehung
		 * 1 ganze Umdrehung:	Microstep * 200 (Standard-Schrittweite)
		 * Microstep 2  ->  400 Schritte
		 * Microstep 4  ->  800 Schritte
		 * Microstep 8  -> 1600 Schritte
		 * Microstep 16 -> 3200 Schritte
	 * Anzahl der Unterteilungen einer ganzen Umdrehung (min. 1, max. 9)
	 * Zeit in *Millisekunden*, bis 7-Segment-Anzeige ausgeschaltet wird (maximale Anzeigedauer betraegt 1 074 s = 1 074 000 ms)
	 */
// Der Motor dreht sich anfangs um 180°.
	
#define ZYKLUS_MILL_AKTIV 900000
#define ZYKLUS_MILL_PAUSE 900000
#define EINE_UMDREHUNG 1600
#define STUFENANZAHL 8
	// Bestenfalls soll bei halber STUFENANZAHL ein 180°-Winkel bei ganzzahliger Schrittzahl erreicht werden
#define ANZEIGE_DAUER_MILL 5000
/* --- Dankeschön --- */

#define TESTMODUS_ZYK_MILL 4000

#define TESTPIN PINB5
#define TESTDDR DDRB
#define TESTPORT PORTB

#define TASTER_BLOCK_MILL 200

// Setzen der Zustaende
typedef enum {
	ZUSTAND_AKTIV,
	ZUSTAND_PAUSE
} zustand_t;

typedef enum {
	ANZEIGE_AN,
	ANZEIGE_AUS
} anzeige_zustand;

typedef enum {
	TESTMODUS_AN,
	TESTMODUS_AUS
} testmodus_zustand;

static zustand_t aktuellerZustand = ZUSTAND_AKTIV;
static volatile anzeige_zustand anzeigeZustand = ANZEIGE_AN;
static testmodus_zustand testModusZustand = TESTMODUS_AUS; // Starte im Testmodus

static uint64_t zyklus_mill_aktiv = TESTMODUS_ZYK_MILL;
static uint64_t zyklus_mill_pause = TESTMODUS_ZYK_MILL;

// Initiieren mit halber Motorumdrehung
static uint16_t zeit_zwischen_schritten = TESTMODUS_ZYK_MILL / (EINE_UMDREHUNG / 2); ///*3000/1600; // */ zyklus_mill_aktiv / EINE_UMDREHUNG;

// Interrupt-Ereignisse
	// Taster 1 und 2
static volatile uint8_t taster0_ereignis = 0;
static volatile uint8_t taster1_ereignis = 0;
	// Timer für Laufzyklus
static volatile uint64_t ueberlauf_zaehler0 = 0;	// weil zyklus_mill_pause groß ist
static volatile uint8_t alarm0 = 0;
	// Timer für Anzeigensteuerung
static volatile uint64_t ueberlauf_zaehler_7seg = 0;
static volatile uint8_t zwischenZaehler = 0;

// Aktivitaet der Tester stuern
static volatile uint8_t taster0_aktiv = 1;
static volatile uint8_t taster1_aktiv = 1;
static volatile uint8_t tasterTest_aktiv = 1;

static volatile uint64_t zaehler_taster0 = 0;
static volatile uint64_t zaehler_taster1 = 0;
static volatile uint64_t zaehler_tasterTest = 0;

static void init(void) {

// Motortreiber
	// Direction_Pin an PD4 als Ausgang
	DDRD |= (1 << PIND4);
	PORTD &= ~(1 << PIND4);

	// Step_Pin an PD5 als Ausgang
	DDRD |= (1 << PIND5);
	PORTD &= ~(1 << PIND5);

// Taster
	// Taster0 an PD2 als Eingang mit Pull-up-Widerstand (extern) (active-Low)
	DDRD &= ~(1 << PIND2);
	PORTD &= ~(1 << PIND2);

	// Taster1 an PD3 als Eingang mit Pull-up-Widerstand (extern) (active-Low)
	DDRD &= ~(1 << PIND3);
	PORTD &= ~(1 << PIND3);

// Interrupts für Taster 
	// Tester0 an PD2 -> INT0 -> Interrupt bei fallender Flanke
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);

	EIMSK |= (1 << INT0);
	
	// Taster1 an PD3 -> INT1 -> Interrupt bei fallender Flanke
	EICRA |= (1 << ISC11);
	EICRA &= ~(1 << ISC10);
	
	EIMSK |= (1 << INT1);
	
// Taster 3 an PB5 als Eingang mit Pull-up-Widerstand (extern) (active-Low)
	// ohne Interrupt
	TESTDDR &= ~(1 << TESTPIN);
	TESTPORT &= ~(1 << TESTPIN);
	
// Zeitgeber0 für Zyklentiming
	// Vorteiler: 8
	TCCR0B &= ~(1 << CS00);
	TCCR0B |= (1 << CS01);
	TCCR0B &= ~(1 << CS02);
	
	// TIMER0 Overflow aktivieren
	TIMSK0 |= (1 << TOIE0);
}

ISR(INT0_vect) {
	//Hauptprogramm benachrichtigen
	if (taster0_aktiv) {
		taster0_ereignis = 1;	
	}
}

ISR(INT1_vect) {
	//Hauptprogramm benachrichtigen
	if (taster1_aktiv) {
		taster1_ereignis = 1;
	}
}

ISR(TIMER0_OVF_vect) {
	++ueberlauf_zaehler0;

	// etwa 1 Millisekunde ~~ 8 * TIMER0-Ereignis
	if (aktuellerZustand == ZUSTAND_PAUSE) {
		if (ueberlauf_zaehler0 == (8 * zyklus_mill_pause)) {
			ueberlauf_zaehler0 = 0;
			alarm0 = 1;
		}
	} else {
		if (ueberlauf_zaehler0 == (8 * zeit_zwischen_schritten)) {
			ueberlauf_zaehler0 = 0;
			alarm0 = 1;
		}
	}
	
	// Zeitereignis fuer die 7-Segment-Anzeige
	if (anzeigeZustand == ANZEIGE_AN) {
		++ueberlauf_zaehler_7seg;
		
		if (ueberlauf_zaehler_7seg == ANZEIGE_DAUER_MILL) {
			ueberlauf_zaehler_7seg = 0;
			++zwischenZaehler;
			if (zwischenZaehler == 8) {
				zwischenZaehler = 0;
				anzeigeZustand = ANZEIGE_AUS; // Signal, um 7-Segment auszuschalten
			}
		}
	}
	
	// Taster nach TASTER_BLOCK_MILL Millisekundes freigeben
	if (!taster0_aktiv) {
		++zaehler_taster0;
		
		if (zaehler_taster0 == (8 * TASTER_BLOCK_MILL)) {
			taster0_aktiv = 1;
			zaehler_taster0 = 0;
		}
	}
	
	if (!taster1_aktiv) {
		++zaehler_taster1;
		
		if(zaehler_taster1 == (8 * TASTER_BLOCK_MILL)) {
			taster1_aktiv = 1;
			zaehler_taster1 = 0;
		}
	}
	
	if (!tasterTest_aktiv) {
		++zaehler_tasterTest;
		
		if (zaehler_tasterTest == (8 * TASTER_BLOCK_MILL)) {
			tasterTest_aktiv = 1;
			zaehler_tasterTest = 0;
		}
	}
}


static uint8_t getTestPinState(void) {
	if ((PINB & (1 << TESTPIN)) == 0) {
		return 0;
	} else {
		return 1;
	}
}

static void reCalc_timeBetweenSteps(uint8_t schrittStufe) {
	zeit_zwischen_schritten = zyklus_mill_aktiv / (schrittStufe * EINE_UMDREHUNG / STUFENANZAHL);
}

static void setTestMode(uint8_t schrittStufe) {
	// Setze die Zyklus- und Pausedauer auf je 4 Sekunden
	zyklus_mill_aktiv = TESTMODUS_ZYK_MILL;
	zyklus_mill_pause = TESTMODUS_ZYK_MILL;
	
	// Gleich mit geaenderter Geschwindigkeit im Testmodus drehen
	reCalc_timeBetweenSteps(schrittStufe);
}

static void unsetTestMode(uint8_t schrittStufe, uint8_t reCalcTime) {
	zyklus_mill_aktiv = ZYKLUS_MILL_AKTIV;
	zyklus_mill_pause = ZYKLUS_MILL_PAUSE;
	
	if (reCalcTime) {
		reCalc_timeBetweenSteps(schrittStufe);
	}
}

static uint8_t setMode(uint8_t schrittStufe, uint8_t reCalcTime) {
	uint8_t zyklusReset = 0; // keine Aufforderung, den Zyklus neu zu starten
	if (testModusZustand == TESTMODUS_AN) {
		setTestMode(schrittStufe);
	} else {
		unsetTestMode(schrittStufe, reCalcTime);
		zyklusReset = 1;
	}
	
	return zyklusReset;
}


static uint8_t switchTestMode(uint8_t schrittStufe) {
	// Schalte den Testmodus ein / aus
	++testModusZustand;
	testModusZustand %= 2;
	
	cli();
	anzeigeZustand = ANZEIGE_AN;
	sei();
	
	return setMode(schrittStufe, 0);
}


int main(void) {
	
	// Idee für Testmodus / Einstellmodus
	/*
	 * 1. gleichzeitiges gedrückthalten beider Taster schaltet zwischen Zyklus- und Testmodus
	 * 2. gleichzeitiges gedrückthalten eines beliebigen Taster schaltet zwischen Z und T
	 * 3. Nutze dritten Taster
	 */
	
	/* zu 2.
	 speichere Tastendruck als true
	 > wenn Taste losgelassen -> false -> normale Funktion
	 > wenn Taste weiter gedrückt -> so 1sec -> Schalte zwischen Z und T
	 */
	
	// Initiieren der Hardware
	init();

	sei();

	sleep_enable();

	uint8_t stufe = STUFENANZAHL / 2;

	uint16_t schritte_aktuell = EINE_UMDREHUNG / 2;	// Ständig aktualisierbar über Taster
	uint16_t schritte_zyklus = schritte_aktuell;	// Während laufendem Zyklus konstant
	uint16_t schritte_uebrig = schritte_zyklus;
	
	uint8_t testPinState_old = getTestPinState();
	// Initiiere den ein- oder ausgeschaltenen Testmodus
	uint8_t zyklusReset = setMode(stufe, 1);
	
	
	// Hauptprogramm
	while (1) {
		// Aktualisiere die Anzeige, wenn sie eingeschaltet ist
		cli();
 		if (anzeigeZustand == ANZEIGE_AN) {
 			sei();
 			single_7seg_number(stufe);
 		} else {
 			sei();
 			single_7seg_off();
 		}
		
		// Detektiere einen Tastendruck fuer den Testmodus //// nicht in Pausen setzbar ...
		cli();
		if (tasterTest_aktiv) {
			sei();
			uint8_t neu = getTestPinState();
			
			if ((neu == 0) && (testPinState_old == 1) ) {
				// Fallende Flanke detektiert
				testPinState_old = 0;
				cli();
				tasterTest_aktiv = 0;
				sei();
				zyklusReset = switchTestMode(stufe);
			}// else if ( (getTestPinState() == 1) && (testPinState_old == 0) ) {
			// 			testPinState_old = 1;
			// 		}
				
			testPinState_old = neu;		
		}
		sei();
		
		// Warte und schlafe solange nichts passiert.
		cli();
		while ( (taster0_ereignis == 0) && (taster1_ereignis == 0) && (alarm0 == 0) ) {
			sei();
			sleep_cpu();
			cli();
		}

		// Prüfe, welches Ereignis eingetreten ist, und behandle dieses.
		if (alarm0 != 0) {
			alarm0 = 0;
			sei();

			if (aktuellerZustand == ZUSTAND_AKTIV) {
				// Schalte DP-LED ein
				setDP_general();
				if (schritte_uebrig != 0) {
					// Mache einen Schritt.
					PORTD |= (1 << PIND5);
					PORTD &= ~(1 << PIND5);
					--schritte_uebrig;
				} else {
					aktuellerZustand = ZUSTAND_PAUSE;
					// Schalte DP-LED aus
					unsetDP_general();
					if (zyklusReset == 1) {
						zyklusReset = 0;
						cli();
						alarm0 = 1;
						sei();
					}
					
				}
			} else {
				// zyklus_mill_pause ist abgelaufen.
				aktuellerZustand = ZUSTAND_AKTIV;
				schritte_zyklus = stufe * EINE_UMDREHUNG / STUFENANZAHL;
				schritte_uebrig = schritte_zyklus;
				zeit_zwischen_schritten = zyklus_mill_aktiv / schritte_zyklus;
				
				// Richtungsänderung.
				PORTD ^= (1 << PIND4);
			}
		}

		cli();
		if (taster0_ereignis != 0) {
			taster0_ereignis = 0;
			taster0_aktiv = 0;
			
			// Bei jedem Tastendruck wird der Timer bis zum Abschalten der Anzeige zurückgesetzt.
			ueberlauf_zaehler_7seg = 0;
			zwischenZaehler = 0;

			if (anzeigeZustand == ANZEIGE_AUS) {
				anzeigeZustand = ANZEIGE_AN;
			} else if (stufe < STUFENANZAHL) {
				++stufe;
			}
			sei();
		}

		cli();
		if (taster1_ereignis != 0) {
			taster1_ereignis = 0;
			taster1_aktiv = 0;
			
			// Bei jedem Tastendruck wird der Timer bis zum Abschalten der Anzeige zurückgesetzt.
			ueberlauf_zaehler_7seg = 0;
			zwischenZaehler = 0;
			
			if (anzeigeZustand == ANZEIGE_AUS) {
				anzeigeZustand = ANZEIGE_AN;
			} else if (stufe > 1) {
				--stufe;
			}
			sei();
		}

	}

	return 0;
}