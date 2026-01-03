
// Thermoino_32_4 v 1.0 Port of Thermode_PWM to Atmel 32u4
// (C) Christian Büchel
// Faster USB connectivity
// Allows for simple custom PCBs that can include optocouplers for Thermode outputs
// Needed to reallocate the Timers as follows:

// Timer1: All Thermode stuff PWM up/down, CTC
// Timer3: Digitimer PWM shock
// There was no real reason why slow ramp had to use Timer4, could also do this with Timer1

// The only real difference is that we have less SRAM (2560 has 8kB, 32u4 has 2.5kB)
// This only gives about 500 CTC entries with 500ms bin width (vs 2500 on 2560)
// To mitigate this we use a variable bit coding scheme so we could fit e.g. ~800 (10bit) entries in 2.5kB
//
// clean up unused stuff
// now using fastio.h for fast port access

#include "Arduino.h"
#include "Thermoino_32u4.h"
#include "SerialCommand.h"
#include "FastIO.h"
#include <EEPROM.h>

// use C preproc to generate enums and char arrays with the same content
#define ERROR_CODES      \
  C(ERR_NULL)            \
  C(ERR_NO_PARAM)        \
  C(ERR_CMD_NOT_FOUND)   \
  C(ERR_CTC_BIN_WIDTH)   \
  C(ERR_CTC_PULSE_WIDTH) \
  C(ERR_CTC_NOT_INIT)    \
  C(ERR_CTC_FULL)        \
  C(ERR_CTC_EMPTY)       \
  C(ERR_SHOCK_RANGE)     \
  C(ERR_SHOCK_ISI)       \
  C(ERR_BUSY)            \
  C(ERR_DEBUG_RANGE)     \
  C(ERR_CHANNEL_RANGE)

#define C(x) x,
enum error_codes
{
  ERROR_CODES N_ERROR_CODES
}; // N_ERROR_CODES gets the number of elements in enum
#undef C
#define C(x) #x,
const char *error_str[] = {ERROR_CODES};
#undef C

// define error codes
#define OK_CODES  \
  C(OK_NULL)      \
  C(OK)           \
  C(OK_READY)     \
  C(OK_MOVE_SLOW) \
  C(OK_MOVE_PREC)

#define C(x) x,
enum ok_codes
{
  OK_CODES N_OK_CODES
}; // N_OK_CODES gets the number of elements in enum
#undef C
#define C(x) #x,
const char *ok_str[] = {OK_CODES};
#undef C

#define A 1 //
#define B 2 //

// TIMER1
//  OC1A = GPIO port PB5 = Arduino Digital Pin D9 32u4 --> DOWN blue
//  OC1B = GPIO port PB6 = Arduino Digital Pin D10 32u4 --> UP green

// TIMER3
// OC3A = GPIO port PC6 = Arduino	Digital pin D5 Leonardo  --> PWM shock
#define SHOCK_PIN 0xC6 // Pin 5 OC3A

#define DOWN_PIN 0xB5 // Pin 9 OC1A
#define UP_PIN 0xB6   // Pin 10 OC1B

#define START_PIN 0xD4 // Pin 4

#define PIN188_0 0xF4 // Pin A3/D17
#define PIN188_1 0xF5 // Pin A2/D16
#define PIN188_2 0xF6 // Pin A1/D15
#define PIN188_3 0xF7 // Pin A0/D14

#define CTC_MAX_N 1000 // on the 32u4 we can only use 1000 bytes of SRAM
// therefore we bit-pack the entries (see below)
// given a max ctc_bin_ms of 500ms (10bit) we can define ctc_data of ~400s = 6.5min
// using smaller ctc_bin_ms allows more entries

#define DIGIHI_US 100       // pulse dur in  µs
#define MIN_DIGI_ISI 1100   // in us
#define MAX_DIGI_STIM 30000 // why not?
#define MAX_DIGI_ISI 65000  // in us should allow freqeuncies down to 15 Hz

#define SCK 16E6        // clock at 16 MHz
#define PWMPRESCALER 64 // prescaler for PWM mode

#define MAX_OSP_TIME 4194240 // this is the longest in us the timer can do precisely. Then we switch to ms

// For prescaler = 8, 64, 256, 1024 use OSP_SET_AND_FIRE_LONG(cycles) instead. The "wait" time-waster makes it work!
// #define wait {delayMicroseconds(2);} // Un-comment this for prescaler = 8
// #define wait {delayMicroseconds(5);} // ...for prescaler = 64, make sure we get at least one clock
// #define wait {delayMicroseconds(17);} // ...for prescaler = 256

#define wait       \
  {                \
    _delay_us(65); \
  } // ...for prescaler = 1024

#define OSP_SET_AND_FIRE_LONG_A(cycles) \
  {                                     \
    uint16_t m = 0xffff - (cycles - 1); \
    OCR1A = m;                          \
    wait;                               \
    TCNT1 = m - 1;                      \
  } // for prescaler > 1
#define OSP_SET_AND_FIRE_LONG_B(cycles) \
  {                                     \
    uint16_t m = 0xffff - (cycles - 1); \
    OCR1B = m;                          \
    wait;                               \
    TCNT1 = m - 1;                      \
  }
#define OSP1_INPROGRESS() (TCNT1 > 0)
#define OSP3_INPROGRESS() (TCNT3 > 0)

//***********************************************************************************
//*********** initialize global variables
//***********************************************************************************

const float SWversion = 3.5;

int32_t cps, prescaler;
uint8_t debug_mode;

uint8_t bit_width;     // once we set ctc_bin_ms we can define how many bits we need
uint16_t bs_max_count; // keep track of how many entries are stored in buffer
uint16_t ctc_bin_ms;   // 0 to 500ms
uint16_t saved_pos;    // if we need to save current position in bitstream

volatile int32_t ctc_bin_ticks;
volatile uint8_t ctc_data[CTC_MAX_N];

typedef struct
{
  volatile uint8_t *buffer;
  uint16_t bitpos;
  uint16_t size_bytes;
  uint8_t bits;
  uint16_t mask;
  uint16_t count; // number of written entries
} BitStream;

BitStream bs;

volatile int32_t count_down_ms;

volatile bool busy_t, busy_d;

volatile uint16_t n_pulse, c_pulse;
int16_t pulse_ms0;
int32_t pulse_tick0;

// complex variables
SerialCommand s_cmd; // The demo SerialCommand object

//***********************************************************************************
//*********** Initialize
//***********************************************************************************

void setup()
{
  busy_d = false;
  busy_t = false;
  debug_mode = 0;
  // setup pins
  Fast_pinMode(SHOCK_PIN, OUTPUT);
  Fast_digitalWrite(SHOCK_PIN, LOW);

  Fast_pinMode(UP_PIN, OUTPUT);
  Fast_digitalWrite(UP_PIN, LOW);
  Fast_pinMode(DOWN_PIN, OUTPUT);
  Fast_digitalWrite(DOWN_PIN, LOW);
  Fast_pinMode(START_PIN, OUTPUT);
  Fast_digitalWrite(START_PIN, LOW);

  Fast_pinMode(PIN188_0, OUTPUT);
  Fast_digitalWrite(PIN188_0, LOW);
  Fast_pinMode(PIN188_1, OUTPUT);
  Fast_digitalWrite(PIN188_1, LOW);
  Fast_pinMode(PIN188_2, OUTPUT);
  Fast_digitalWrite(PIN188_2, LOW);
  Fast_pinMode(PIN188_3, OUTPUT);
  Fast_digitalWrite(PIN188_3, LOW);

  // kill Timer1
  TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  TCCR1A = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  TCNT1 = 0;  // set counter to zero

  // kill Timer3
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  s_cmd.addCommand("VER", processVER);
  s_cmd.addCommand("DIAG", processDIAG);
  s_cmd.addCommand("MOVE", processMOVE);
  s_cmd.addCommand("START", processSTART);
  s_cmd.addCommand("SHOCK", processSHOCK);
  s_cmd.addCommand("GETTIME", processGETTIME);
  s_cmd.addCommand("DEBUG", processDEBUG);
  s_cmd.addCommand("D188", processD188);
  s_cmd.addCommand("HELP", processHELP);
  s_cmd.addCommand("INITCTC", processINITCTC);
  s_cmd.addCommand("LOADCTC", processLOADCTC);
  s_cmd.addCommand("QUERYCTC", processQUERYCTC);
  s_cmd.addCommand("EXECCTC", processEXECCTC);
  s_cmd.addCommand("FLUSHCTC", processFLUSHCTC);
  s_cmd.addCommand("MAXCTC", processMAXCTC);
  s_cmd.addCommand("STATUS", processSTATUS);
  s_cmd.addCommand("STATUS_D", processSTATUS_D);
  s_cmd.addCommand("STATUS_T", processSTATUS_T);
  s_cmd.addCommand("KILL", processKILL);
  s_cmd.addCommand("KILL_D", processKILL_D);
  s_cmd.addCommand("KILL_T", processKILL_T);
  s_cmd.addCommand("SETID", processSETID);
  s_cmd.addCommand("GETID", processGETID);
  s_cmd.setDefaultHandler(unrecognized); // Handler for command that isn't matched  (says "What?")
}

//***********************************************************************************
//*********** Run
//***********************************************************************************

void loop()
{
  s_cmd.readSerial(); //  parse commands
}

//***********************************************************************************
//*********** Setup Parse handlers
//***********************************************************************************

void processSETID()
// allows to name Thermoino and store ID in EEPROM (set debug level to 2)
{
  char *arg;
  char Buffer[16] = {0};
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // if there is more, take it
  {
    if (debug_mode > 1)
    {
      Serial.println(debug_mode);
      strncpy(Buffer, arg, sizeof(Buffer) - 1);
      Serial.println(Buffer);
      EEPROM.put(0, Buffer);
    }
    else
    {
      Serial.println(F("Not authorized !")); // debug level needs to be 2
    }
  }
}

void processGETID()
// returns name of Thermoino from EEPROM
{
  char Buffer[16] = {0};
  EEPROM.get(0, Buffer);
  Serial.println(Buffer);
}

void processVER()
// returns the FW version of the Thermoino
{
  Serial.println(SWversion);
}

void processDIAG()
// returns all available commands
{
  displayStatusSerial();
  Serial.println(F("Available commands:"));
  s_cmd.printCommands();
}

void processDEBUG()
// sets the debug level i.e. verbosity level 2 allows to set a new name
{
  char *arg;
  uint8_t New;
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // if there is more, take it
  {
    New = atoi(arg);
    if (check_range(&debug_mode, New, (uint8_t)0, (uint8_t)2)) // setting DEBUG to 2 allows to change the ID in EEprom
      print_ok(OK);
    else
    {
      print_error(ERR_DEBUG_RANGE);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processD188()
// sets the Digitimer D188 channel outputs 1..8
{
  char *arg;
  uint8_t New, channel;
  if ((busy_d) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // if there is more, take it
  {
    New = atoi(arg);
    if (check_range(&channel, New, (uint8_t)0, (uint8_t)8)) // channel 1..8   0=off
    {
      print_ok(OK);
      // now set D188 channel outputs
      uint8_t out = channel; //
      busy_d = true;
      /*Fast_digitalWrite(PIN188_0, (out >> 0) & 1);
      Fast_digitalWrite(PIN188_1, (out >> 1) & 1);
      Fast_digitalWrite(PIN188_2, (out >> 2) & 1);
      Fast_digitalWrite(PIN188_3, (out >> 3) & 1);*/

      out &= 0x0F;                         // ensure 0..15
      PORTF = (PORTF & 0x0F) | (out << 4); // very fast port write

      delay(1); // let relais settle
      busy_d = false;
    }

    else
    {
      print_error(ERR_CHANNEL_RANGE);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processMOVE()
// main thermode command moving temperature up (pos) or down (neg); times are in µs
{
  char *arg;
  int32_t us;

  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it exists, take it
  {

    us = atol(arg);
    if (abs(us) < MAX_OSP_TIME)
    {
      ramp_temp_prec(us);
      print_ok(OK_MOVE_PREC);
    }
    else
    {
      if (debug_mode > 0)
      {
        Serial.print(F("Pulse time longer than "));
        Serial.println(MAX_OSP_TIME);
        Serial.println(F("using ramp_temp"));
      }
      ramp_temp(us / 1000);
      print_ok(OK_MOVE_SLOW);
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processSTART()
// simple command to create a pulse (40ms) to start thermode
{
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }
  Fast_digitalWrite(START_PIN, HIGH);
  delay(40); // wait 40ms
  Fast_digitalWrite(START_PIN, LOW);
  print_ok(OK);
}

void processSHOCK()
// create a train of shocks
{
  char *arg;
  uint32_t New, n_stim, isi;
  if ((busy_d) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {
    New = atol(arg);

    if (!check_range(&n_stim, New, (uint32_t)0, (uint32_t)MAX_DIGI_STIM))
    {
      print_error(ERR_SHOCK_RANGE);
      return;
    }

    n_pulse = n_stim;
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)
  {
    New = atol(arg);
    if (!check_range(&isi, New, (uint32_t)MIN_DIGI_ISI, (uint32_t)MAX_DIGI_ISI))
    {
      print_error(ERR_SHOCK_ISI);
      return;
    }
  }
  else
  {
    isi = MIN_DIGI_ISI;
  }

  if (debug_mode > 0)
  {
    Serial.print(F("Using PWM Timer3 giving "));
    Serial.print(n_stim);
    Serial.print(F(" pulses, every "));
    Serial.print(isi);
    Serial.println(F(" us"));
  }

  // now prepare Timer3 for PWM
  cli(); // stop interrupts
  Fast_digitalWrite(SHOCK_PIN, LOW);
  // TCCR3A = 0; necessary?
  TCCR3B = 0; // same for TCCR3B

  TIFR3 |= (1 << TOV3);  // very important
  TIMSK3 = (1 << TOIE3); // interrupt when TCNT3 overflows

  TCCR3A = (1 << COM3A1) + (1 << COM3B1) + (1 << WGM31);
  TCCR3B = (1 << WGM33); // normal mode, phase correct PWM ... don't start yet!

  ICR3 = isi / 8; // ticks

  OCR3A = 12; // pulsewidth 8us * 12 = 96us

  TCNT3 = ICR3 - 1; // initialize counter value
  // TCNT3 = 0xFFFF; // initialize counter value
  busy_d = true;
  c_pulse = 0; // reset pulse counter
  print_ok(OK);
  TCCR3B |= (1 << CS30) | (1 << CS31); // Now start timer with prescaler 64 (res 8us, max dur = 0xFFFF x 8 = 524ms)

  sei(); // enable global interrupts
}

void processGETTIME()
// return Thermoino time in ms
{
  Serial.println(millis());
}

void processHELP()
{
  display_help();
  display_error_codes();
}

void processINITCTC()
// initialize CTC storage
{
  char *arg;
  uint16_t tmp;
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {

    reset_ctc(); // ALWAYS reset all ctc_data vars (including flushing the loaded ctc_data itself) when initializing

    tmp = atoi(arg);

    if (check_range(&ctc_bin_ms, tmp, (uint16_t)1, (uint16_t)500))
    {
      print_ok(OK);

      bit_width = bits_required(ctc_bin_ms);                  // determine how many bits we need to store ctc_bin_ms
      bs_max_count = (uint16_t)((CTC_MAX_N * 8) / bit_width); // determine how many entries we can store given bit_width
      bs_init(&bs, ctc_data, sizeof(ctc_data), bit_width);    // initialize bitstream
      if (debug_mode > 0)
      {
        Serial.print(F("CTC bits:  "));
        Serial.println(bit_width);
        Serial.print(F("CTC max entries:  "));
        Serial.println(bs_max_count);
      }
    }
    else
    {
      reset_ctc();
      print_error(ERR_CTC_BIN_WIDTH);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processMAXCTC()
// add an item to the CTC
{
  char *arg;
  int32_t move_ms, t_move_ms;

  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    reset_ctc();
    return;
  }
  Serial.println(bs_max_count - 1);
}

void processLOADCTC()
// add an item to the CTC
{
  char *arg;
  int32_t move_ms, t_move_ms;

  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    reset_ctc();
    return;
  }

  if ((bs.bitpos / bs.bits) >= (bs_max_count - 1)) // we need at least one slot free for the ending zero pulse
  {
    print_error(ERR_CTC_FULL);
    reset_ctc();
    return;
  }
  // get segment duration
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)    // As long as it existed, take it
  {
    move_ms = atol(arg);
    if (check_range_abs(&t_move_ms, move_ms, (int32_t)0, (int32_t)ctc_bin_ms))
    {
      bs_write(&bs, t_move_ms);
      print_ok(OK);
    }
    else
    {
      reset_ctc();
      print_error(ERR_CTC_PULSE_WIDTH);
      return;
    }
  }
  else
  {
    print_error(ERR_NO_PARAM);
    return;
  }
}

void processQUERYCTC()
// return all entries from CTC storage
{
  char *arg;
  uint8_t query_lvl;
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != NULL)
    query_lvl = atoi(arg);
  else
    query_lvl = 1;

  if ((query_lvl == 1) && (ctc_bin_ms > 0) && (bs.count > 0))
  {
    Serial.println(F("Status: READY"));
  }

  if ((query_lvl == 2) && (bs.count > 0))
  {
    Serial.print(F("ctc_bin_ms: "));
    Serial.println(ctc_bin_ms);
    Serial.print(F("n_ctc: "));
    Serial.println(bs.count);
  }

  if ((query_lvl == 3) && (bs.count > 0)) // if required, also return more detailed info
  {
    saved_pos = bs.bitpos;
    bs.bitpos = 0; // reset bitpos to beginning
    for (uint16_t p = 0; p < bs.count; p++)
    {
      Serial.print(p);
      Serial.print(F(" "));
      Serial.println(bs_read(&bs));
    }
    bs.bitpos = saved_pos; // reset bitpos to saved position
  }
}

void processEXECCTC()
// start CTC
{
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    return;
  }

  if (bs.count == 0) // no data
  {
    print_error(ERR_CTC_EMPTY);
    return;
  }

  saved_pos = bs.bitpos;
  bs_write(&bs, 0); // add a zero pulse to the end to circumvent cut-off in ISR

  cli();
  Fast_digitalWrite(UP_PIN, LOW);
  Fast_digitalWrite(DOWN_PIN, LOW);
  TCCR1B = 0;            // Stop timer before configuring
  TIFR1 |= (1 << TOV1);  // very important as this is still set from EXECCTC (clear Timer/Counter1 Overflow Flag)
  TIMSK1 = (1 << TOIE1); // interrupt when TCNT1 overflows

  // we use phase correct PWM mode 10 (with ICR1 as TOP)
  TCCR1A = (1 << COM1A1) + (1 << COM1B1) + (1 << WGM11);
  TCCR1B = (1 << WGM13); // normal mode, phase correct PWM ... don't start yet!

  ctc_bin_ticks = SCK / 2 / PWMPRESCALER * (int32_t)ctc_bin_ms / 1000;
  ICR1 = ctc_bin_ticks + 1; // important to set this BEFORE OCR1A/B
  // +1 hack to allow a pulse width equal ctc_bin_ms (actually ctc_bin_ms is 8 us longer)

  // get the first pulse
  bs.bitpos = 0;            // reset bitpos to beginning
  pulse_ms0 = bs_read(&bs); // read first pulse

  pulse_tick0 = SCK / 2 / PWMPRESCALER * (int32_t)pulse_ms0 / 1000;
  if (pulse_tick0 > 0)
  {
    OCR1B = pulse_tick0;
    OCR1A = 0;
  }
  else
  {
    OCR1B = 0;
    OCR1A = -pulse_tick0;
  }

  TCNT1 = ICR1 - 1; // Should not be zero, because then we will miss first entry as ISR "eats up" ctc_data value
  // ICR1-1 means low latency as OCR1A/B will be updated at ICR1
  busy_t = true;

  TCCR1B |= (1 << CS10) | (1 << CS11); // Now start timer with prescaler 64 (max res 8us, max dur = 0xFFFF x 8 = 524ms)
  print_ok(OK);
  // alternative: prescaler 256 (max res 32us, max dur = 0xFFFF x 32 = 2s)
  sei();
}

void processFLUSHCTC()
// flush CTC storage
{
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
    return; // then we're just not ready...
  }
  reset_ctc();
  print_ok(OK);
}

void processSTATUS()
// returns any activity (thermode or digitimer)
{
  if ((busy_t) || (busy_d) || (OSP1_INPROGRESS()) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY); // anything going on thermode or digitimer
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processSTATUS_D()
// returns if digitimer is busy
{
  if ((busy_d) || (OSP3_INPROGRESS()))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}
void processSTATUS_T()
// returns if thermode is busy
{
  if ((busy_t) || (OSP1_INPROGRESS()))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processKILL()
// stop any ongoing activty (thermode and digitimer)
{
  // TIMER1 stuff
  TCCR1A = 0; // clear all Timer/PWM functionality
  TCCR1B = 0;
  TCNT1 = 0;
  TIMSK1 &= ~(1 << TOIE1); // disable interrupt

  Fast_digitalWrite(DOWN_PIN, LOW); // set ports low
  Fast_digitalWrite(UP_PIN, LOW);   //
  busy_t = false;

  // TIMER3 stuff (digitimer)
  TCCR3A = 0; // clear all Timer/PWM functionality
  TCCR3B = 0;

  c_pulse = 0;
  TCNT3 = 0;                         //
  TIMSK3 &= ~(1 << TOIE3);           // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  Fast_digitalWrite(SHOCK_PIN, LOW); //
  busy_d = false;
  print_ok(OK_READY);
}

void processKILL_D()
// stop ongoing activty of digitimer
{
  // TIMER3 stuff (digitimer)
  TCCR3A = 0; // clear all Timer/PWM functionality
  TCCR3B = 0;

  // PORTE &= ~(1 << PE3); // probably not necessary set PE3 low
  c_pulse = 0;
  TCNT3 = 0;                         //
  TIMSK3 &= ~(1 << TOIE3);           // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  Fast_digitalWrite(SHOCK_PIN, LOW); //
  busy_d = false;
  print_ok(OK_READY);
}

void processKILL_T()
// stop ongoing activty of thermode
{

  // TIMER1 stuff (fast move)
  TCCR1A = 0; // clear all Timer/PWM functionality
  TCCR1B = 0;
  reset_ctc();
  TCNT1 = 0;               // does not help to do a MOVE after EXECCTCPWM...
  TIMSK1 &= ~(1 << TOIE1); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES

  Fast_digitalWrite(DOWN_PIN, LOW); //
  Fast_digitalWrite(UP_PIN, LOW);   //

  busy_t = false;

  print_ok(OK_READY);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command)
{
  print_error(ERR_CMD_NOT_FOUND);
}

//***********************************************************************************
//*********** Interrupt Service Routines ********************************************
//***********************************************************************************
ISR(TIMER1_COMPA_vect) // for slow ramping up/down
{
  if (count_down_ms < 0)
    count_down_ms++;
  if (count_down_ms > 0)
    count_down_ms--;

  if (count_down_ms == 0) // now we are done
  {
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 &= ~(1 << OCIE1A);         // can we disable interrupt  in ISR ??? -> YES
    Fast_digitalWrite(DOWN_PIN, LOW); //
    Fast_digitalWrite(UP_PIN, LOW);   //
    TCNT1 = 0;                        // does not help to do a MOVE after EXECCTCPWM...
    busy_t = false;
  }
}

ISR(TIMER1_OVF_vect) // for CTC
{
  // where all the magic happens
  // ISR is called when counter has finished (after ctc_bin_ms)
  // we simply set the thresholds to fire the next pulse OCR1A for UP
  // OCR1B for DOWN

  if ((bs.bitpos / bs.bits) >= bs.count)

  {
    TCCR1A = 0; // clear all Timer/PWM functionality
    TCCR1B = 0;
    Fast_digitalWrite(DOWN_PIN, LOW); //
    Fast_digitalWrite(UP_PIN, LOW);   //
    TCNT1 = 0;                        // does not help to do a MOVE after EXECCTCPWM...
    TIMSK1 &= ~(1 << TOIE1);          // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_t = false;
    bs.bitpos = saved_pos; // reset bitpos to saved position (i.e. without zero pulse)
    bs.count--;            // remove the zero pulse from count
    return;
  }

  int16_t pulse_ms = bs_read(&bs);
  int32_t pulse_tick = SCK / 2 / PWMPRESCALER * (int32_t)pulse_ms / 1000;

  if (pulse_tick > 0)
  {
    OCR1B = pulse_tick;
    OCR1A = 0;
  }
  else
  {
    OCR1B = 0;
    OCR1A = -pulse_tick;
  }
}

ISR(TIMER3_OVF_vect) // for digitimer shocks
{
  // we simply count the number of pulses and then stop
  // ISR is called when counter has finished i.e. pulse has been generated

  if (c_pulse == n_pulse - 1) // almost done
    OCR3A = 0;                // create null pulse
  // as we fire ISR on TOP, we cut the last pulse in half therefore we do one more and set the last one to 0

  if (c_pulse == n_pulse) // done
  {
    TCCR3A = 0; // clear all Timer/PWM functionality
    TCCR3B = 0;

    c_pulse = 0;
    TCNT3 = 0;               //
    TIMSK3 &= ~(1 << TOIE3); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_d = false;
  }
  c_pulse++;
}

//***********************************************************************************
//*********** helper functions
//***********************************************************************************

void print_error(int8_t error_code)
{
  char buffer[4]; // 3 digits and a zero
  sprintf(buffer, "%03d", -error_code);
  Serial.print(buffer);
  if (debug_mode > 0)
  {
    Serial.print(" ");
    Serial.print(error_str[error_code]);
  }
  Serial.println();
}

void print_ok(int8_t ok_code)
{
  char buffer[4]; // 3 digits and a zero
  sprintf(buffer, "%03d", ok_code);
  Serial.print(buffer);
  if (debug_mode > 0)
  {
    Serial.print(F(" "));
    Serial.print(ok_str[ok_code]);
  }
  Serial.println();
}
//***********************************************************************************
//*********** diagnostics, free RAM (i.e. between stack and heap)
//***********************************************************************************
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

//***********************************************************************************
//*********** status display over serial port evoked bey serial command DIAG;
//***********************************************************************************
void displayStatusSerial()
{
  Serial.print(F("+++"));
  Serial.print(F(" V:"));
  Serial.print(SWversion, 1);
  Serial.print(F(" RAM:"));
  Serial.print(freeRam());
  Serial.println(F(" +++"));

  Serial.print(F("Debug level: "));
  Serial.println(debug_mode);
}

//***********************************************************************************
//*********** status help over serial port evoked by serial command HELP;
//***********************************************************************************
void display_help()
{
  Serial.print(F("Thermoino "));
  Serial.println(SWversion, 1);
  Serial.print(F("Name: "));
  processGETID();
  Serial.println(F("Only compatible with UseThermoinoPTB.m"));
  Serial.println(F("**************************************"));
  Serial.println(F("Misc commands:"));
  Serial.println(F("--------------"));
  Serial.println(F("VER           - Print version"));
  Serial.println(F("DIAG          - Get diagnostics"));
  Serial.println(F("GETTIME       - Get Arduino time in ms"));
  Serial.println(F("DEBUG;XX      - Set debug state (0: OFF) (1: ON))"));
  Serial.println(F("HELP          - This command"));
  Serial.println(F("STATUS        - check whether anything (thermode or digitimer) is running"));
  Serial.println(F("KILL          - stop all activity (thermode and digitimer) "));
  Serial.println(F("SETID         - enter new name for thermoino, saved permanently in EEPROM"));
  Serial.println(F("GETID         - return name of thermoino from EEPROM"));
  Serial.println();
  Serial.println(F("Thermode related commands:"));
  Serial.println(F("--------------------------"));
  Serial.println(F("STATUS_T      - check whether thermode is running"));
  Serial.println(F("KILL_T        - stop activity of thermode"));
  Serial.println(F("MOVE;XX       - Move temp up/down for XX us"));
  Serial.println(F("START         - Send 40ms TTL pulse to start thermode"));
  Serial.println(F("INITCTC;xx    - Initialize complex time courses (ctc_data) with cycle xx in ms (500 max)"));
  Serial.print(F("LOADCTC;xx    - add pulse to ctc_data queue (xx in ms) -xx means temperature decrease, max "));
  Serial.print(CTC_MAX_N);
  Serial.println(F(" items"));

  Serial.println(F("QUERYCTC(;yy) - status of the ctc_data queue (yy=3 to get all entries)"));
  Serial.println(F("MAXCTC        - returns max number of entries for CTC buffer based on bitwidth)"));
  Serial.println(F("EXECCTC       - execute ctc_data queue using precise PWM"));
  Serial.println(F("FLUSHCTC      - reset ctc and clear ctc_data queue"));
  Serial.println();
  Serial.println(F("Digitimer related commands:"));
  Serial.println(F("---------------------------"));
  Serial.println(F("STATUS_D      - check whether digitimer is running"));
  Serial.println(F("KILL_D        - stop activity of digitimer"));
  Serial.println(F("SHOCK;nn(;yy) - Digitimer stimuli number (nn) @interval 1100us OR additionally specify interval between pulses (yy) in us (>1000) "));
  Serial.println(F("D188;xx       - select D188 channel xx (1..8)"));
}

void display_error_codes()
{
  Serial.println(F("Error codes:"));
  for (int8_t i = 1; i < N_ERROR_CODES; i++) // skip first error code
  {
    Serial.print(-i);
    Serial.print(F(" "));
    Serial.println(error_str[i]);
  }

  Serial.println(F("OK codes:"));
  for (int8_t i = 1; i < N_OK_CODES; i++) // skip first ok code
  {
    Serial.print(i);
    Serial.print(F(" "));
    Serial.println(ok_str[i]);
  }
}
//***********************************************************************************
//*********** subfunction to reset ctc_data variables
//***********************************************************************************
void reset_ctc()
{
  ctc_bin_ms = 0;
  memset_volatile(ctc_data, 0, sizeof(ctc_data)); // erase ctc_data
  bs_init(&bs, ctc_data, sizeof(ctc_data), 0);    // set biwdth to 0 to indicate uninitialized
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp(int32_t ms)
{
  if (debug_mode > 0)
  {
    Serial.print(F("ramp_temp: "));
    Serial.print(ms);
    Serial.println(F("ms"));
  }
  cli();
  count_down_ms = ms;
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1B |= (1 << WGM12);
  // set Output Compare Register to (250 - 1) ticks
  OCR1A = 0xF9;
  // TCNT1
  TCNT1 = 0;
  // TIMSK4
  // Set Timer Interrupt Mask Register to
  // Clear Timer on Compare channel A for timer 4
  TIFR1 |= (1 << OCF1A); // very important otherwise ISR will immediately be executed
  TIMSK1 |= (1 << OCIE1A);
  busy_t = true;
  if (ms < 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Ramping down:  "));
      Serial.println(-ms);
    }
    Fast_digitalWrite(DOWN_PIN, HIGH); //
  }
  if (ms > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Ramping up:  "));
      Serial.println(ms);
    }
    Fast_digitalWrite(UP_PIN, HIGH); //
  }
  TCCR1B |= (1 << CS11) | (1 << CS10); // start clock
  sei();
}

//***********************************************************************************
//*********** Prepare Timer 1
//***********************************************************************************

void osp_setup(uint8_t which, int32_t prescaler)
{
  TCCR1B = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  // This keeps anything from happening while we get set up
  TCNT1 = 0; // Start counting at bottom.
  ICR1 = 0;  // Set TOP to 0, Mode 14. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
  // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX
  // and then overflow back to 0 and get locked up again.

  if (which == A)
  {
    OCR1A = 0xffff;
    TCCR1A = (1 << COM1A0) | (1 << COM1A1) | (1 << WGM11); // OC1A=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
  }
  else if (which == B)
  {
    OCR1B = 0xffff;
    TCCR1A = (1 << COM1B0) | (1 << COM1B1) | (1 << WGM11); // OC1B=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
  }

  else
  {
    Serial.println(F("ERROR only OC1A or OC1B supported"));
  }

  //   (using Chris Hahn's notation here)
  // Prescaler  Setup - Choose one of these, then choose a matching "wait" delay statement below.
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10); // Prescaler = 1; Start counting now. Max ~4mS
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler = 8; Start counting now. Max ~32mS, starts in ~10uS or better
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11); // Prescaler = 64; Start counting now. Max ~.26 sec, starts in ~20uS or better
  if (prescaler == 256) // resolution 16 us
  {
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS12); // Prescaler = 256; Start counting now. Max ~1.05 sec, starts in ~64uS or better
  }
  else if (prescaler == 1024) // resolution 64 us
  {
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10) | (1 << CS12); // Prescaler = 1024; Start counting now. Max ~4 sec, starts in ~180uS or better
  }
  else
  {
    Serial.println(F("ERROR only 256 or 1024 supported"));
  }
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp_prec(int32_t o_us) // specifiy in us
{
  int32_t o_tic;

  if (abs(o_us) < 1048560) // we can use 256 as a prescaler
  {
    prescaler = 256; // we get up to ~4 s pulses
  }
  else
  {
    prescaler = 1024;
  }

  cps = SCK / prescaler; // tics per second
  // now convert us into tics
  o_tic = (int64_t)o_us * cps / 1000000; // convert from us to tics

  if (o_tic < 0)
  {
    o_tic = abs(o_tic);

    if (debug_mode > 0)
    {
      Serial.print(F("Precision-ramping down:  "));
      Serial.println(o_tic);
      Serial.print(F("Prescaler:  "));
      Serial.println(prescaler);
    }
    osp_setup(A, prescaler);
    OSP_SET_AND_FIRE_LONG_A(o_tic) //
  }
  else if (o_tic > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print(F("Precision-ramping up:  "));
      Serial.println(o_tic);
      Serial.print(F("Prescaler:  "));
      Serial.println(prescaler);
    }
    osp_setup(B, prescaler);
    OSP_SET_AND_FIRE_LONG_B(o_tic) //
  }
}

void memset_volatile(volatile void *s, char c, size_t n)
{
  volatile char *p = (volatile char *)s;
  while (n-- > 0)
  {
    *p++ = c;
  }
}

uint8_t bits_required(uint16_t n)
{
  if (n < 16)
    return 5; // 4 magnitude bits + sign
  if (n < 32)
    return 6;
  if (n < 64)
    return 7;
  if (n < 128)
    return 8;
  if (n < 256)
    return 9;
  if (n < 512)
    return 10;
}

void bs_init(BitStream *bs, volatile uint8_t *buf, uint16_t size_bytes, uint8_t bits)
{
  bs->buffer = buf;
  bs->bitpos = 0;
  bs->size_bytes = size_bytes;
  bs->bits = bits;
  bs->mask = (1U << bits) - 1;
  bs->count = 0;
}
int bs_write(BitStream *bs, int16_t value)
{
  uint32_t total_bits = (uint32_t)bs->size_bytes * 8;

  if (bs->bitpos + bs->bits > total_bits)
    return -1; // no space left

  uint16_t byte = bs->bitpos >> 3;
  uint8_t shift = bs->bitpos & 7;

  uint16_t v = (uint16_t)(value & bs->mask);

  bs->buffer[byte] |= (uint8_t)(v << shift);

  if (shift + bs->bits > 8)
    bs->buffer[byte + 1] |= (uint8_t)(v >> (8 - shift));

  bs->bitpos += bs->bits;
  bs->count++; // track number of entries
  return 0;
}
int16_t bs_read(BitStream *bs)
{
  uint16_t byte = bs->bitpos >> 3;
  uint8_t shift = bs->bitpos & 7;

  uint32_t raw = bs->buffer[byte] >> shift;

  if (shift + bs->bits > 8)
    raw |= (uint32_t)bs->buffer[byte + 1] << (8 - shift);

  raw &= bs->mask;

  if (raw & (1U << (bs->bits - 1)))
    raw |= ~bs->mask;

  bs->bitpos += bs->bits;
  return (int16_t)raw;
}
