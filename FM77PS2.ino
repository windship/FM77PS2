/*
 * FM77PS2.ino
 *
 * PS/2 keyboard -> FM77AV keyboard converter
 * Arduino Nano / ATmega328P @ 16 MHz
 *
 * Pin assignment:
 *   D2 : PS/2 CLK  (INT0)
 *   D3 : PS/2 DATA
 *   D4 : FM77AV KSDATA
 *
 * FM77AV DETECT is handled externally by tying it to GND
 * for wired-keyboard mode.
 *
 * Based on the original ATtiny85 project, with the January 2020
 * FM77AV release-code/beep fix incorporated.
 *
 * v0.2r: v0.2p + corrected FM77AV shifted symbol mappings (~, _, quote). No queue, no held-key array,
 * no automatic fallback to a previously pressed key.
 */

#include "FM77PS2.h"
#include "scancodes.h"

volatile uint8_t bitcount = 11;
volatile uint8_t edge = 0;

static volatile uint8_t kb_buf[BUF_SIZE];
static volatile uint8_t *inptr;
static volatile uint8_t *outptr;
static volatile uint8_t bufcnt;


// -----------------------------------------------------------------------------
// v0.2c: minimal held-key handling
// -----------------------------------------------------------------------------
// Keep exactly one repeat key.  There is no held-key array, no event queue,
// and no automatic return to an older key.
#define FM_KEY_GAP_MS 3
#define FM_REPEAT_MS  20

static int16_t fmRepeatKey = -1;
static unsigned long fmLastRepeat = 0;
static unsigned long fmLastTx = 0;

static void fm_wait_gap(void)
{
  while ((unsigned long)(millis() - fmLastTx) < FM_KEY_GAP_MS) {
  }
}

static void fm_send_make(uint8_t index)
{
  fm_wait_gap();
  cnvcode(index);
  fmLastTx = millis();
}

static void fm_send_release(uint8_t index)
{
  fm_wait_gap();
  cnvreleasecode(index);
  fmLastTx = millis();
}

static void fm_start_key(uint8_t index)
{
  // A new physical make becomes the only repeat source.
  // PS/2 typematic duplicates of the same key do not restart transmission.
  if (fmRepeatKey == (int16_t)index) {
    return;
  }

  fm_send_make(index);
  fmRepeatKey = index;
  fmLastRepeat = millis();
}

static void fm_stop_key(uint8_t index)
{
  // Stop repetition before doing anything that can block.
  if (fmRepeatKey == (int16_t)index) {
    fmRepeatKey = -1;
  }

  fm_send_release(index);
}

static void fm_repeat_service(void)
{
  if (fmRepeatKey < 0) {
    return;
  }

  unsigned long now = millis();
  if ((unsigned long)(now - fmLastRepeat) >= FM_REPEAT_MS) {
    fm_send_make((uint8_t)fmRepeatKey);
    fmLastRepeat = millis();
  }
}

// -----------------------------------------------------------------------------
// v0.2n US-101 -> JIS PS/2 front-end translator
//
// IMPORTANT: outserialdata(), FM transmission and repeat logic below are the
// original v0.2c implementation.  This layer only changes the PS/2 byte stream
// that reaches it.
//
// Shift make is held here until the following key is known.  We can therefore
// emit exactly the JIS PS/2 sequence that would type the requested US symbol.
// -----------------------------------------------------------------------------
static uint8_t trLShift = 0;
static uint8_t trRShift = 0;
static uint8_t trF0 = 0;
static uint8_t trE0 = 0;

// Remember the translated JIS key for every physical PS/2 key until its break.
// This is required because PS/2 make events can overlap (key rollover).
static uint8_t trActiveJis[256];
static uint8_t trActiveFlags[32];      // bit=1: translated key is active
static uint8_t trActiveShift[32];      // bit=1: its JIS sequence used Shift
static uint8_t trForcedBreak[32];      // late physical break must be swallowed

// v0.2c itself has one deferred Shift/Ctrl state.  Therefore only a translated
// Shift character may own that state at once.  If the next make arrives before
// its break, finish this character in the correct order first.
static uint8_t trShiftOwner = 0;

static uint8_t tr_bit_get(uint8_t *a, uint8_t k)
{
  return (a[k >> 3] >> (k & 7)) & 1;
}
static void tr_bit_set(uint8_t *a, uint8_t k)
{
  a[k >> 3] |= (uint8_t)(1U << (k & 7));
}
static void tr_bit_clear(uint8_t *a, uint8_t k)
{
  a[k >> 3] &= (uint8_t)~(1U << (k & 7));
}

static void tr_emit(uint8_t b)
{
  outserialdata(b);
}

static void tr_emit_make(uint8_t jisPs, uint8_t needShift)
{
  if (needShift) tr_emit(L_SHIFT);
  tr_emit(jisPs);
}

static void tr_emit_break(uint8_t jisPs, uint8_t hadShift)
{
  tr_emit(0xF0);
  tr_emit(jisPs);
  if (hadShift) {
    tr_emit(0xF0);
    tr_emit(L_SHIFT);
  }
}

// Return JIS Set-2 key and JIS Shift state for US punctuation.
// Digits/letters with identical relationship return 0 and use normal path.
static uint8_t tr_us_symbol(uint8_t ps, uint8_t usShift,
                            uint8_t *jisPs, uint8_t *jisShift)
{
  switch (ps) {
    case 0x1E: if(usShift){*jisPs=0x54;*jisShift=0;return 1;} break; // @
    case 0x36: if(usShift){*jisPs=0x55;*jisShift=0;return 1;} break; // ^
    case 0x3D: if(usShift){*jisPs=0x36;*jisShift=1;return 1;} break; // &
    case 0x3E: if(usShift){*jisPs=0x52;*jisShift=1;return 1;} break; // *
    case 0x46: if(usShift){*jisPs=0x3E;*jisShift=1;return 1;} break; // (
    case 0x45: if(usShift){*jisPs=0x46;*jisShift=1;return 1;} break; // )

    case 0x4E: *jisPs=usShift?0x51:0x4E; *jisShift=usShift?1:0; return 1; // - _
    case 0x55: *jisPs=usShift?0x4C:0x4E; *jisShift=1; return 1;     // = +
    case 0x54: *jisPs=0x5B; *jisShift=usShift; return 1;             // [ {
    case 0x5B: *jisPs=0x5D; *jisShift=usShift; return 1;             // ] }
    case 0x5D: *jisPs=0x6A; *jisShift=usShift; return 1;             // \ |
    case 0x4C: *jisPs=usShift?0x52:0x4C; *jisShift=0; return 1;     // ; :
    case 0x52: *jisPs=usShift?0x1E:0x3D; *jisShift=1; return 1;     // ' "
    case 0x0E: *jisPs=usShift?0x55:0x54; *jisShift=1; return 1;     // ` ~
    case 0x41: *jisPs=0x41; *jisShift=usShift; return 1;             // , <
    case 0x49: *jisPs=0x49; *jisShift=usShift; return 1;             // . >
    case 0x4A: *jisPs=0x4A; *jisShift=usShift; return 1;             // / ?
  }
  return 0;
}

static void translate_us101(uint8_t b)
{
  if (b==0xE0) { trE0=1; return; }
  if (b==0xF0) { trF0=1; return; }

  // Physical Shift is only input state. The equivalent JIS Shift sequence is
  // synthesized together with the following character.
  if (!trE0 && (b==L_SHIFT || b==R_SHIFT)) {
    if (!trF0) {
      if (b==L_SHIFT) trLShift=1; else trRShift=1;
    } else {
      if (b==L_SHIFT) trLShift=0; else trRShift=0;
    }
    trF0=trE0=0;
    return;
  }

  uint8_t usShift=(trLShift||trRShift)?1:0;

  // v0.2r FM77AV editing-key cluster mapping.
  // Keep the proven US symbol translator untouched; only remap these physical keys.
  //
  // IBM PC Home       (E0 6C) -> FM HOME (original F12 / 07)
  // IBM PC Page Up    (E0 7D) -> FM DUP  (original F11 / 78)
  // IBM PC End        (E0 69) -> FM EL   (existing L-Win / E0 1F)
  // IBM PC Page Down  (E0 7A) -> FM CLS  (original ScrollLock / 7E)
  if (trE0 && (b==0x6C || b==0x7D || b==0x69 || b==0x7A)) {
    uint8_t target;
    uint8_t targetE0=0;

    if      (b==0x6C) target=0x07;       // HOME
    else if (b==0x7D) target=0x78;       // DUP
    else if (b==0x69) { target=0x1F; targetE0=1; } // EL
    else              target=0x7E;       // CLS

    if (targetE0) tr_emit(0xE0);
    if (trF0) tr_emit(0xF0);
    tr_emit(target);
    trF0=trE0=0;
    return;
  }

  // Physical Scroll Lock is the FM BREAK key in v0.2r.
  // The table entry for 7E itself remains CLS so Page Down can synthesize CLS.
  if (!trE0 && b==0x7E) {
    if (trF0) tr_emit(0xF0);
    tr_emit(0x0E);                       // existing FM BREAK entry
    trF0=trE0=0;
    return;
  }

  if (!trE0 && trF0) {
    // A key that was force-released because of rollover: consume its late
    // physical break and do not send a second FM77AV release.
    if (tr_bit_get(trForcedBreak,b)) {
      tr_bit_clear(trForcedBreak,b);
      trF0=trE0=0;
      return;
    }

    if (tr_bit_get(trActiveFlags,b)) {
      uint8_t jp=trActiveJis[b];
      uint8_t js=tr_bit_get(trActiveShift,b);
      tr_emit_break(jp,js);
      tr_bit_clear(trActiveFlags,b);
      tr_bit_clear(trActiveShift,b);
      if (trShiftOwner==b) trShiftOwner=0;
      trF0=trE0=0;
      return;
    }
  }

  if (!trE0 && !trF0) {
    // PS/2 rollover: a new make can arrive before the previous shifted key's
    // break. v0.2c has one deferred Shift state, so finish only that shifted
    // character now, preserving the proven order: key break -> Shift break.
    // Non-Shift keys (e.g. VALIS direction + Space) are NOT serialized here.
    if (trShiftOwner && trShiftOwner!=b &&
        tr_bit_get(trActiveFlags,trShiftOwner)) {
      uint8_t old=trShiftOwner;
      tr_emit_break(trActiveJis[old],1);
      tr_bit_clear(trActiveFlags,old);
      tr_bit_clear(trActiveShift,old);
      tr_bit_set(trForcedBreak,old);
      trShiftOwner=0;
    }

    uint8_t jp=b, js=0;
    if (tr_us_symbol(b,usShift,&jp,&js)) {
      tr_emit_make(jp,js);
    } else {
      js=usShift;
      if (js) tr_emit(L_SHIFT);
      tr_emit(b);
    }

    trActiveJis[b]=jp;
    tr_bit_set(trActiveFlags,b);
    if (js) {
      tr_bit_set(trActiveShift,b);
      trShiftOwner=b;
    } else {
      tr_bit_clear(trActiveShift,b);
    }

    trF0=trE0=0;
    return;
  }

  // Extended/special keys remain the original v0.2c stream.
  if (trE0) tr_emit(0xE0);
  if (trF0) tr_emit(0xF0);
  tr_emit(b);
  trF0=trE0=0;
}

void setup()
{
  init_kb();
}

void loop()
{
  // Consume pending PS/2 bytes before generating another FM77AV repeat.
  while (kb_getbuf()) {
  }
  fm_repeat_service();
}

void init_kb(void)
{
  // D2 = PS/2 CLK input, D3 = PS/2 DATA input, D4 = FM77 KSDATA output.
  DDRD &= ~(PS2_CLK_MASK | PS2_DATA_MASK);
  DDRD |= FM_KSDATA_MASK;

  // Original design did not enable internal pull-ups.
  PORTD &= ~(PS2_CLK_MASK | PS2_DATA_MASK);

  // FM77AV KSDATA idle state = High.
  PORTD |= FM_KSDATA_MASK;

  // INT0 on falling edge initially.
  EICRA &= ~(_BV(ISC00));
  EICRA |= _BV(ISC01);

  bitcount = 11;
  edge = 0;

  inptr = kb_buf;
  outptr = kb_buf;
  bufcnt = 0;

  fmRepeatKey = -1;
  fmLastRepeat = millis();
  fmLastTx = millis() - FM_KEY_GAP_MS;

  // Enable INT0.
  EIFR |= _BV(INTF0);
  EIMSK |= _BV(INT0);

  sei();
}

// -----------------------------------------------------------------------------
// PS/2 receiver
// -----------------------------------------------------------------------------
ISR(INT0_vect)
{
  static uint8_t data = 0;

  if (!edge) {
    // Falling edge: sample PS/2 data bits.
    if (bitcount < 11 && bitcount > 2) {
      data >>= 1;
      if (PIND & PS2_DATA_MASK) {
        data |= 0x80;
      }
    }

    // Next interrupt: rising edge.
    EICRA |= _BV(ISC00);
    EICRA |= _BV(ISC01);
    edge = 1;
  }
  else {
    // Rising edge: advance the frame counter.
    EICRA &= ~_BV(ISC00);
    EICRA |= _BV(ISC01);
    edge = 0;

    if (--bitcount == 0) {
      kb_putbuf(data);
      bitcount = 11;
    }
  }
}

void kb_putbuf(uint8_t c)
{
  if (bufcnt < BUF_SIZE) {
    *inptr = c;
    ++inptr;
    ++bufcnt;

    if (inptr >= kb_buf + BUF_SIZE) {
      inptr = kb_buf;
    }
  }
}

int kb_getbuf(void)
{
  uint8_t byte;

  noInterrupts();
  if (bufcnt == 0) {
    interrupts();
    return 0;
  }

  byte = *outptr;
  ++outptr;
  if (outptr >= kb_buf + BUF_SIZE) {
    outptr = kb_buf;
  }
  --bufcnt;
  interrupts();

  translate_us101(byte);
  return byte;
}

// -----------------------------------------------------------------------------
// PS/2 scan code -> FM77AV serial code
// -----------------------------------------------------------------------------
void outserialdata(uint8_t kcode)
{
  uint16_t i;
  uint16_t stti = 0;
  static uint8_t svcode = 0;
  static uint8_t svsftctrl = 0;
  static uint8_t skpcnt = 0;
  static uint8_t flgF0 = 0;
  static uint8_t flgE0 = 0;
  static uint8_t flgE1 = 0;
  static uint8_t flgSFTCTRL = 0;

  PORTD |= FM_KSDATA_MASK;

  // PAUSE sequence.
  if (skpcnt > 0) {
    --skpcnt;
    return;
  }

  switch (kcode) {
    case 0xF0:
      flgF0 = 1;
      break;

    case 0xE0:
      flgE0 = 1;
      break;

    case 0xE1:
      flgE1 = 1;
      skpcnt = 6;
      break;

    default:
      if (flgF0 == 0) {
        // Shift/Ctrl are emitted before the following key.
        if (kcode == L_SHIFT || kcode == R_SHIFT ||
            kcode == L_CTRL  || kcode == R_CTRL) {
          flgSFTCTRL = 1;
          svsftctrl = kcode;
        }
        else {
          if (flgSFTCTRL == 1) {
            svcode = kcode;
            kcode = svsftctrl;

            for (i = 0;
                 i < 107 && pgm_read_byte(&pscode0[i]) != kcode &&
                 pgm_read_byte(&pscode0[i]) != 0x00;
                 ++i) {
            }

            if (i < 107 && pgm_read_byte(&pscode0[i]) == kcode) {
              fm_start_key((uint8_t)i);
            }

            kcode = svcode;
          }

          if (flgE0 != 0) {
            stti = 91;
          }

          if (flgE1 != 0) {
            stti = 105;
            kcode = 0x77;
          }

          for (i = stti;
               i < 107 && pgm_read_byte(&pscode0[i]) != kcode &&
               pgm_read_byte(&pscode0[i]) != 0x00;
               ++i) {
          }

          flgE0 = 0;
          flgE1 = 0;

          if (i < 107 && pgm_read_byte(&pscode0[i]) == kcode) {
            fm_start_key((uint8_t)i);
          }
        }
      }
      else {
        // PS/2 break code.  IMPORTANT: preserve E0 until after the table
        // lookup, otherwise an extended-key release can be matched wrongly.
        flgF0 = 0;

        if (flgSFTCTRL == 1 && kcode == svsftctrl) {
          stti = 0;
          for (i = stti;
               i < 107 && pgm_read_byte(&pscode0[i]) != kcode &&
               pgm_read_byte(&pscode0[i]) != 0x00;
               ++i) {
          }

          flgSFTCTRL = 0;
          svsftctrl = 0;

          if (i < 107 && pgm_read_byte(&pscode0[i]) == kcode) {
            fm_stop_key((uint8_t)i);
          }
        }
        else {
          stti = (flgE0 != 0) ? 91 : 0;

          for (i = stti;
               i < 107 && pgm_read_byte(&pscode0[i]) != kcode &&
               pgm_read_byte(&pscode0[i]) != 0x00;
               ++i) {
          }

          if (i < 107 && pgm_read_byte(&pscode0[i]) == kcode) {
            fm_stop_key((uint8_t)i);
          }
          else {
            // Fail-safe: an unmatched break must never leave an old repeat
            // running forever.
            fmRepeatKey = -1;
          }
        }

        flgE0 = 0;
        flgE1 = 0;
      }
      break;
  }
}

// -----------------------------------------------------------------------------
// Send FM77AV make code.
// 40 bits = 0xB4 + four bytes from fmcode0[].
// -----------------------------------------------------------------------------
int cnvcode(uint16_t codeno)
{
  uint8_t kc;

  serial2port(0xB4);

  for (uint8_t i = 0; i < 4; ++i) {
    kc = pgm_read_byte(&fmcode0[codeno][i]);
    serial2port(kc);
  }

  PORTD |= FM_KSDATA_MASK;

  // The original 2019 code used 20 ms here.  The release-code fix makes
  // that unnecessary; 1 ms is sufficient in the corrected implementation.
  delay(1);
  return 0;
}

// -----------------------------------------------------------------------------
// Send FM77AV release code.
//
// Only the first two bytes change from the make code:
//   B4 CC -> B3 4C
//   B4 CB -> B3 4B
//   B4 B4 -> B3 34
//   B4 B3 -> B3 33
// The remaining three bytes are unchanged.
// -----------------------------------------------------------------------------
int cnvreleasecode(uint16_t codeno)
{
  uint8_t kc;

  serial2port(0xB3);

  kc = pgm_read_byte(&fmcode0[codeno][0]);
  switch (kc) {
    case 0xCC:
      serial2port(0x4C);
      break;
    case 0xCB:
      serial2port(0x4B);
      break;
    case 0xB4:
      serial2port(0x34);
      break;
    case 0xB3:
      serial2port(0x33);
      break;
    default:
      // The existing FM77AV key table is expected to contain only the
      // four patterns above.  Keep the stream aligned if an unexpected
      // value is encountered.
      serial2port(kc);
      break;
  }

  for (uint8_t i = 1; i < 4; ++i) {
    kc = pgm_read_byte(&fmcode0[codeno][i]);
    serial2port(kc);
  }

  PORTD |= FM_KSDATA_MASK;
  delay(1);
  return 0;
}

// -----------------------------------------------------------------------------
// Send one byte on FM77AV KSDATA.
//
// FM77AV serial timing used by the corrected implementation:
//   bit 0: 100 us
//   bit 1: 125 us
//   bit 2: 175 us
//   bit 3: no pulse (the 4th bit is skipped)
//   bit 4: 100 us
//   bit 5: 125 us
//   bit 6: 175 us
//   bit 7: no pulse
// -----------------------------------------------------------------------------
void serial2port(uint8_t kc)
{
  for (uint8_t i = 0; i < 8; ++i) {
    if ((i + 1) % 4 != 0) {
      if (kc & 0x80) {
        PORTD &= ~FM_KSDATA_MASK;
      }
      else {
        PORTD |= FM_KSDATA_MASK;
      }
    }

    switch (i) {
      case 0:
      case 4:
        delayMicroseconds(100);
        break;

      case 1:
      case 5:
        delayMicroseconds(125);
        break;

      case 2:
      case 6:
        delayMicroseconds(175);
        break;

      case 3:
      case 7:
        break;
    }

    kc <<= 1;
  }
}
