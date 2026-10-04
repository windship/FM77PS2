#ifndef FM77PS2_H
#define FM77PS2_H

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// -----------------------------------------------------------------------------
// Arduino Nano (ATmega328P @ 16 MHz) pin assignment
// -----------------------------------------------------------------------------
// PS/2 keyboard CLK  -> D2 / INT0
// PS/2 keyboard DATA -> D3
// FM77AV KSDATA      -> D4
//
// The original ATtiny85 project did not use FM77AV DETECT. Keep that behavior
// here. Do NOT connect D4 directly to a different FM77AV signal.
// -----------------------------------------------------------------------------
#define PS2_CLK_PIN   2
#define PS2_DATA_PIN  3
#define FM_KSDATA_PIN 4

// Direct PORTD masks used by the ISR/output routines.
#define PS2_CLK_MASK   _BV(PD2)
#define PS2_DATA_MASK  _BV(PD3)
#define FM_KSDATA_MASK _BV(PD4)

// PS/2 make-code values used by the original converter.
#define L_SHIFT 0x12
#define R_SHIFT 0x59
#define L_CTRL  0x14
#define R_CTRL  0x14

#define BUF_SIZE 64

// These are modified by the INT0 ISR.
extern volatile uint8_t bitcount;
extern volatile uint8_t edge;

void init_kb(void);
void ps2_isr(void);
int kb_getbuf(void);
void kb_putbuf(uint8_t c);
void outserialdata(uint8_t kcode);
int cnvcode(uint16_t codeno);
void serial2port(uint8_t kc);

#endif
