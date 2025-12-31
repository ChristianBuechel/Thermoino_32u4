# 1 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"

//***********************************************************************************
//*********** defines
//***********************************************************************************
// Thermoino_32_4 v 1.0 Port of Thermode_PWM to Atmel 32u4
// (C) Christian Büchel
// Faster USB connectivity
// Allows for simple custom PCBs that can include optocouplers for Thermode outputs
// Need to reallocate the Timers as follows:

// Timer1: Thermode PWM up/down
// Timer3: Digitimer PWM shock
// Timer4: Thermode slow ramp
// But there was no real reason why slow ramp had to use Timer4, could also do this with Timer1
// maybe even using the OC1A OC1B outputs


// clean up unused stuff
// now using fastio.h for fast port access


# 23 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 2
# 24 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 2
# 25 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 2
# 26 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 2
# 27 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 2

// use C preproc to generate enums and char arrays with the same content
# 45 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
enum error_codes
{
  ERR_NULL, ERR_NO_PARAM, ERR_CMD_NOT_FOUND, ERR_CTC_BIN_WIDTH, ERR_CTC_PULSE_WIDTH, ERR_CTC_NOT_INIT, ERR_CTC_FULL, ERR_CTC_EMPTY, ERR_SHOCK_RANGE, ERR_SHOCK_ISI, ERR_BUSY, ERR_DEBUG_RANGE, ERR_CHANNEL_RANGE, N_ERROR_CODES
}; // N_ERROR_CODES gets the number of elements in enum


const char *error_str[] = {"ERR_NULL", "ERR_NO_PARAM", "ERR_CMD_NOT_FOUND", "ERR_CTC_BIN_WIDTH", "ERR_CTC_PULSE_WIDTH", "ERR_CTC_NOT_INIT", "ERR_CTC_FULL", "ERR_CTC_EMPTY", "ERR_SHOCK_RANGE", "ERR_SHOCK_ISI", "ERR_BUSY", "ERR_DEBUG_RANGE", "ERR_CHANNEL_RANGE",};


// define error codes
# 63 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
enum ok_codes
{
  OK_NULL, OK, OK_READY, OK_MOVE_SLOW, OK_MOVE_PREC, N_OK_CODES
}; // N_OK_CODES gets the number of elements in enum


const char *ok_str[] = {"OK_NULL", "OK", "OK_READY", "OK_MOVE_SLOW", "OK_MOVE_PREC",};





// TIMER1
//  OC1A = GPIO port PB5 = Arduino Digital Pin D9 32u4 --> DOWN blue
//  OC1B = GPIO port PB6 = Arduino Digital Pin D10 32u4 --> UP green

// TIMER3
// OC3A = GPIO port PC6 = Arduino	Digital pin D5 Leonardo  --> PWM shock
# 97 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
// max entries for complex time-course (2500 + 1 zero)
// given a max ctc_bin_ms of 500ms we can define ctc_data of 1000s
# 110 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
// For prescaler = 8, 64, 256, 1024 use OSP_SET_AND_FIRE_LONG(cycles) instead. The "wait" time-waster makes it work!
// #define wait {delayMicroseconds(2);} // Un-comment this for prescaler = 8
// #define wait {delayMicroseconds(5);} // ...for prescaler = 64, make sure we get at least one clock
// #define wait {delayMicroseconds(17);} // ...for prescaler = 256
# 137 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
//***********************************************************************************
//*********** initialize global variables
//***********************************************************************************

const float SWversion = 1.0;

String last_cmd; // last cmd goes here

int32_t cps, prescaler;
uint8_t debug_mode;

uint16_t ctc_bin_ms; // 0 to 500; 500ms with 2500 CTC entries this allows 1250s (~20min) of waveform
volatile int32_t ctc_bin_ticks;
volatile int16_t ctc_data[500 + 1]; // intwernally we have to add a zero pulse at the end
volatile uint16_t n_ctc, c_ctc;

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
  Fast_pinMode(0xC6 /* Pin 5 OC3A*/, 0x1);
  Fast_digitalWrite(0xC6 /* Pin 5 OC3A*/, 0x0);

  Fast_pinMode(0xB6 /*Pin 10 OC1B*/, 0x1);
  Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0);
  Fast_pinMode(0xB5 /*Pin 9 OC1A*/, 0x1);
  Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0);
  Fast_pinMode(0xD4 /*Pin 4*/, 0x1);
  Fast_digitalWrite(0xD4 /*Pin 4*/, 0x0);

  Fast_pinMode(0xC7 /*Pin 13*/, 0x1);
  Fast_digitalWrite(0xC7 /*Pin 13*/, 0x0);

  Fast_pinMode(0xF4 /* Pin A3/D17*/, 0x1);
  Fast_digitalWrite(0xF4 /* Pin A3/D17*/, 0x0);
  Fast_pinMode(0xF5 /* Pin A2/D16*/, 0x1);
  Fast_digitalWrite(0xF5 /* Pin A2/D16*/, 0x0);
  Fast_pinMode(0xF6 /* Pin A1/D15*/, 0x1);
  Fast_digitalWrite(0xF6 /* Pin A1/D15*/, 0x0);
  Fast_pinMode(0xF7 /* Pin A0/D14*/, 0x1);
  Fast_digitalWrite(0xF7 /* Pin A0/D14*/, 0x0);


  // kill Timer1
  
# 198 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 198 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  
# 199 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 199 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  
# 200 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 200 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; // set counter to zero

  // kill Timer3
  
# 203 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x90)) 
# 203 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;
  
# 204 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 204 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;
  
# 205 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x94)) 
# 205 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0;

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
  last_cmd = "SETID;";
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 259 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 259 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // if there is more, take it
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
      Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 270 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 270 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    "Not authorized !"
# 270 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    ); &__c[0];}))
# 270 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    ))); // debug level needs to be 2
    }
  }
}

void processGETID()
// returns name of Thermoino from EEPROM
{
  char *arg;
  char Buffer[16] = {0};
  last_cmd = "GETID;";
  EEPROM.get(0, Buffer);
  Serial.println(Buffer);
}

void processVER()
// returns the FW version of the Thermoino
{
  last_cmd = "VER;";
  Serial.println(SWversion);
}

void processDIAG()
// returns all available commands
{
  char *arg;
  last_cmd = "DIAG;";
  displayStatusSerial();
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 298 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 298 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "Available commands:"
# 298 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 298 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  s_cmd.printCommands();
}

void processDEBUG()
//sets the debug level i.e. verbosity level 2 allows to set a new name
{
  char *arg;
  last_cmd = "DEBUG;";
  uint8_t New;
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 309 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 309 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // if there is more, take it
  {
    New = atoi(arg);
    if (check_range(&debug_mode, New, (uint8_t)0, (uint8_t)2)) //setting DEBUG to 2 allows to change the ID in EEprom
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
//sets the debug level i.e. verbosity level 2 allows to set a new name

{
  char *arg;
  last_cmd = "D188;";
  uint8_t New, channel;
  if ((busy_d) || ((
# 334 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x94)) 
# 334 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 341 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 341 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // if there is more, take it
  {
    New = atoi(arg);
    if (check_range(&channel, New, (uint8_t)1, (uint8_t)8)) // channel 1..8
    {
      print_ok(OK);
      // now set D188 channel outputs
      uint8_t out = channel; // convert 1..8 → 0..7
      busy_d = true;
      Fast_digitalWrite(0xF4 /* Pin A3/D17*/, (out >> 0) & 1);
      Fast_digitalWrite(0xF5 /* Pin A2/D16*/, (out >> 1) & 1);
      Fast_digitalWrite(0xF6 /* Pin A1/D15*/, (out >> 2) & 1);
      Fast_digitalWrite(0xF7 /* Pin A0/D14*/, (out >> 3) & 1);
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
//main thermode command moving temperature up (pos) or down (neg); times are in µs
{
  char *arg;
  int32_t us;
  last_cmd = "MOVE;";

  if ((busy_t) || ((
# 378 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 378 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 385 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 385 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // As long as it exists, take it
  {
    last_cmd = last_cmd + arg;

    us = atol(arg);
    if (((us)>0?(us):-(us)) < 4194240 /* this is the longest in us the timer can do precisely. Then we switch to ms*/)
    {
      ramp_temp_prec(us);
      print_ok(OK_MOVE_PREC);
    }
    else
    {
      if (debug_mode > 0)
      {
        Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 399 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 399 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    "Pulse time longer than "
# 399 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    ); &__c[0];}))
# 399 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    )));
        Serial.println(4194240 /* this is the longest in us the timer can do precisely. Then we switch to ms*/);
        Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 401 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 401 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      "using ramp_temp"
# 401 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                      ); &__c[0];}))
# 401 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      )));
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
//simple command to create a pulse (40ms) to start thermode
{
  last_cmd = "START;";
  if ((busy_t) || ((
# 419 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 419 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }
  Fast_digitalWrite(0xD4 /*Pin 4*/, 0x1);
  delay(40); // wait 40ms
  Fast_digitalWrite(0xD4 /*Pin 4*/, 0x0);
  print_ok(OK);
}

void processSHOCK()
//create a train of shocks
{
  char *arg;
  uint32_t New, n_stim, isi;
  last_cmd = "SHOCK;";
  if ((busy_d) || ((
# 436 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x94)) 
# 436 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 443 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 443 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;
    New = atol(arg);

    if (!check_range(&n_stim, New, (uint32_t)0, (uint32_t)30000 /* why not?*/))
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
  if (arg != 
# 462 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 462 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )
  {
    last_cmd = last_cmd + arg;
    New = atol(arg);
    if (!check_range(&isi, New, (uint32_t)1100 /* in us*/, (uint32_t)65000 /* in us should allow freqeuncies down to 15 Hz*/))
    {
      print_error(ERR_SHOCK_ISI);
      return;
    }
  }
  else
  {
    isi = 1100 /* in us*/;
  }

  if (debug_mode > 0)
  {
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 479 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 479 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Using PWM Timer3 giving "
# 479 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 479 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.print(n_stim);
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 481 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 481 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                " pulses, every "
# 481 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 481 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.print(isi);
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 483 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 483 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  " us"
# 483 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 483 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
  }

  // now prepare Timer3 for PWM
  
# 487 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("cli" ::: "memory")
# 487 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ; // stop interrupts
  Fast_digitalWrite(0xC6 /* Pin 5 OC3A*/, 0x0);
  //TCCR3A = 0; necessary?
  
# 490 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 490 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // same for TCCR3B

  
# 492 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)((0x18) + 0x20)) 
# 492 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       |= (1 << 
# 492 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                0
# 492 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    ); // very important
  
# 493 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x71)) 
# 493 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 493 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                0
# 493 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ); // interrupt when TCNT3 overflows

  
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x90)) 
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                7
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      ) + (1 << 
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                5
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                      ) + (1 << 
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                1
# 495 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                     );
  
# 496 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 496 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 496 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                4
# 496 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ); // normal mode, phase correct PWM ... don't start yet!

  
# 498 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x96)) 
# 498 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      = isi / 8; // ticks

  
# 500 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x98)) 
# 500 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 12; // pulsewidth 8us * 12 = 96us

  
# 502 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x94)) 
# 502 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 
# 502 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
         (*(volatile uint16_t *)(0x96)) 
# 502 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              - 1; // initialize counter value
  // TCNT3 = 0xFFFF; // initialize counter value
  busy_d = true;
  c_pulse = 0; // reset pulse counter
  print_ok(OK);
  
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        |= (1 << 
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                 0
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ) | (1 << 
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                               1
# 507 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                   ); // Now start timer with prescaler 64 (res 8us, max dur = 0xFFFF x 8 = 524ms)

  
# 509 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 509 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ; // enable global interrupts
}

void processGETTIME()
//return Thermoino time in ms
{
  last_cmd = "GETTIME;";
  Serial.println(millis());
}

void processHELP()
{
  char *arg;
  last_cmd = "HELP;";
  display_help();
  display_error_codes();
}

void processINITCTC()
//initialize CTC storage
{
  char *arg;
  uint16_t tmp;
  last_cmd = "INITCTC;";
  if ((busy_t) || ((
# 533 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 533 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }

  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 540 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 540 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;

    reset_ctc(); // ALWAYS reset all ctc_data vars (including flushing the loaded ctc_data itself) when initializing

    tmp = atoi(arg);

    if (check_range(&ctc_bin_ms, tmp, (uint16_t)1, (uint16_t)500))
      print_ok(OK);
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

void processLOADCTC()
// add an item to the CTC
{
  char *arg;
  int32_t move_ms, t_move_ms;

  last_cmd = "LOADCTC;";
  if ((busy_t) || ((
# 571 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 571 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
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

  if (n_ctc == 500) // full
  {
    print_error(ERR_CTC_FULL);
    reset_ctc();
    return;
  }
  // get segment duration
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 592 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 592 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                ) // As long as it existed, take it
  {
    last_cmd = last_cmd + arg;
    move_ms = atol(arg);
    if (check_range_abs(&t_move_ms, move_ms, (int32_t)0, (int32_t)ctc_bin_ms))
    {
      ctc_data[n_ctc] = t_move_ms;
      n_ctc++; // increment pulse counter CAVE, this is the number of pulses, not the index of the last pulse
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
//return all entries from CTC storage
{
  char *arg;
  uint8_t query_lvl;
  last_cmd = "QUERYCTC;";
  if ((busy_t) || ((
# 622 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 622 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }
  arg = s_cmd.next(); // Get the next argument from the SerialCommand object buffer
  if (arg != 
# 628 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3 4
            __null
# 628 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )
    query_lvl = atoi(arg);
  else
    query_lvl = 1;

  if ((query_lvl == 1) && (ctc_bin_ms > 0) && (n_ctc > 0))
  {
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 635 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 635 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "Status: READY"
# 635 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 635 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
  }

  if ((query_lvl == 2) && (n_ctc > 0))
  {
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 640 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 640 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "ctc_bin_ms: "
# 640 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 640 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.println(ctc_bin_ms);
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 642 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 642 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "n_ctc: "
# 642 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 642 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.println(n_ctc);
  }

  if ((query_lvl == 3) && (n_ctc > 0)) // if required, also return more detailed info
  {
    for (uint16_t p = 0; p < n_ctc; p++)
    {
      Serial.print(p);
      Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 651 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 651 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  " "
# 651 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 651 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
      Serial.println(ctc_data[p]);
    }
  }
}

void processEXECCTC()
//start CTC
{
  last_cmd = "EXECCTC;";

  if ((busy_t) || ((
# 662 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 662 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return;
  }

  if (ctc_bin_ms == 0) // not initialized
  {
    print_error(ERR_CTC_NOT_INIT);
    return;
  }

  if (n_ctc == 0) // no data
  {
    print_error(ERR_CTC_EMPTY);
    return;
  }

  c_ctc = 0; // first entry
  ctc_data[n_ctc] = 0; // add a zero pulse to the end to circumvent cut-off in ISR
  n_ctc++; // increment pulse counter

  
# 684 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("cli" ::: "memory")
# 684 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ;
  Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0);
  Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0);
  
# 687 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 687 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // Stop timer before configuring
  
# 688 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)((0x16) + 0x20)) 
# 688 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       |= (1 << 
# 688 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                0
# 688 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    ); // very important as this is still set from EXECCTC (clear Timer/Counter1 Overflow Flag)
  
# 689 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x6F)) 
# 689 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 689 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                0
# 689 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ); // interrupt when TCNT1 overflows

  // we use phase correct PWM mode 10 (with ICR1 as TOP)
  
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                7
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      ) + (1 << 
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                5
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                      ) + (1 << 
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                1
# 692 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                     );
  
# 693 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 693 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = (1 << 
# 693 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                4
# 693 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ); // normal mode, phase correct PWM ... don't start yet!

  ctc_bin_ticks = 16E6 /* clock at 16 MHz*/ / 2 / 64 /* prescaler for PWM mode*/ * (int32_t)ctc_bin_ms / 1000;
  
# 696 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x86)) 
# 696 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      = ctc_bin_ticks + 1; // important to set this BEFORE OCR1A/B
  // +1 hack to allow a pulse width equal ctc_bin_ms (actually ctc_bin_ms is 8 us longer)
  // get the first pulse
  pulse_ms0 = ctc_data[c_ctc];

  c_ctc++; // and increment counter, next ctc_data value will be set in interrupt
  pulse_tick0 = 16E6 /* clock at 16 MHz*/ / 2 / 64 /* prescaler for PWM mode*/ * (int32_t)pulse_ms0 / 1000;
  if (pulse_tick0 > 0)
  {
    
# 705 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 705 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = pulse_tick0;
    
# 706 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 706 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0;
  }
  else
  {
    
# 710 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 710 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0;
    
# 711 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 711 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = -pulse_tick0;
  }

  
# 714 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 714 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 
# 714 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
         (*(volatile uint16_t *)(0x86)) 
# 714 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              - 1; // Should not be zero, because then we will miss first entry as ISR "eats up" ctc_data value
  // ICR1-1 means low latency as OCR1A/B will be updated at ICR1
  busy_t = true;
  // here we could also start the thermode
  
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        |= (1 << 
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                 0
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ) | (1 << 
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                               1
# 718 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                   ); // Now start timer with prescaler 64 (max res 8us, max dur = 0xFFFF x 8 = 524ms)
  print_ok(OK);

  // alternative: prescaler 256 (max res 32us, max dur = 0xFFFF x 32 = 2s)

  
# 723 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 723 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ;
}

void processFLUSHCTC()
//flush CTC storage
{
  last_cmd = "FLUSHCTC;";
  if ((busy_t) || ((
# 730 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 730 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
    return; // then we're just not ready...
  }
  reset_ctc();
  print_ok(OK);
}

void processSTATUS()
//returns any activity (thermode or digitimer)
{
  last_cmd = "STATUS;";
  if ((busy_t) || (busy_d) || ((
# 743 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                              (*(volatile uint16_t *)(0x84)) 
# 743 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                              > 0)) || ((
# 743 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                     (*(volatile uint16_t *)(0x94)) 
# 743 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                     > 0)))
  {
    print_error(ERR_BUSY); // anything going on thermode or digitimer
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processSTATUS_D()
//returns if digitimer is busy
{
  last_cmd = "STATUS_D;";
  if ((busy_d) || ((
# 757 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x94)) 
# 757 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}
void processSTATUS_T()
//returns if thermode is busy
{
  last_cmd = "STATUS_T;";
  if ((busy_t) || ((
# 770 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (*(volatile uint16_t *)(0x84)) 
# 770 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  > 0)))
  {
    print_error(ERR_BUSY);
  }
  else
  {
    print_ok(OK_READY);
  }
}

void processKILL()
//stop any ongoing activty (thermode and digitimer)
{
  last_cmd = "KILL;";


  // TIMER1 stuff (fast move)
  
# 787 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 787 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // clear all Timer/PWM functionality
  
# 788 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 788 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;
  
# 789 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 789 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; //
  
# 790 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x6F)) 
# 790 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        &= ~(1 << 
# 790 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  0
# 790 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ); // disable interrupt

  Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0); // set ports low
  Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0); //
  Fast_digitalWrite(0xC7 /*Pin 13*/, 0x0); //
  busy_t = false;

  // TIMER3 stuff (digitimer)
  
# 798 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x90)) 
# 798 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // clear all Timer/PWM functionality
  
# 799 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 799 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;

  c_pulse = 0;
  
# 802 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x94)) 
# 802 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; //
  
# 803 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x71)) 
# 803 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        &= ~(1 << 
# 803 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  0
# 803 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  Fast_digitalWrite(0xC6 /* Pin 5 OC3A*/, 0x0); //
  busy_d = false;

  print_ok(OK_READY);
}

void processKILL_D()
//stop ongoing activty of digitimer
{
  last_cmd = "KILL_D;";

  // TIMER3 stuff (digitimer)
  
# 816 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x90)) 
# 816 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // clear all Timer/PWM functionality
  
# 817 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x91)) 
# 817 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;

  // PORTE &= ~(1 << PE3); // probably not necessary set PE3 low
  c_pulse = 0;
  
# 821 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x94)) 
# 821 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; //
  
# 822 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x71)) 
# 822 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        &= ~(1 << 
# 822 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  0
# 822 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
  Fast_digitalWrite(0xC6 /* Pin 5 OC3A*/, 0x0); //
  busy_d = false;
  print_ok(OK_READY);
}

void processKILL_T()
//stop ongoing activty of thermode
{
  last_cmd = "KILL_T;";

  // TIMER1 stuff (fast move)
  
# 834 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 834 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // clear all Timer/PWM functionality
  
# 835 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 835 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;
  c_ctc = 0;
  
# 837 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 837 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; // does not help to do a MOVE after EXECCTCPWM...
  
# 838 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x6F)) 
# 838 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        &= ~(1 << 
# 838 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  0
# 838 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES

  Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0); //
  Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0); //
  Fast_digitalWrite(0xC7 /*Pin 13*/, 0x0); //

  busy_t = false;

  print_ok(OK_READY);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command)
{
  last_cmd = "say what?"; // could print the whole command
  print_error(ERR_CMD_NOT_FOUND);
}


//***********************************************************************************
//*********** Interrupt Service Routines ********************************************
//***********************************************************************************

# 860 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
extern "C" void __vector_17 /* Timer/Counter1 Compare Match A */ (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_17 /* Timer/Counter1 Compare Match A */ (void) 
# 860 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      // for slow ramping up/down
{
  if (count_down_ms < 0)
    count_down_ms++;
  if (count_down_ms > 0)
    count_down_ms--;

  if (count_down_ms == 0) // now we are done
  {
    
# 869 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 869 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0;
    
# 870 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 870 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0;
    
# 871 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x6F)) 
# 871 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          &= ~(1 << 
# 871 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    1
# 871 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                          ); // can we disable interrupt  in ISR ??? -> YES
    Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0); //
    Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0); //
    Fast_digitalWrite(0xC7 /*Pin 13*/, 0x0); //
    
# 875 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x84)) 
# 875 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0; // does not help to do a MOVE after EXECCTCPWM...
    busy_t = false;
  }
}


# 880 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
extern "C" void __vector_20 /* Timer/Counter1 Overflow */ (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_20 /* Timer/Counter1 Overflow */ (void) 
# 880 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    // for CTC
{
  // where all the magic happens
  // ISR is called when counter has finished (after ctc_bin_ms)
  // we simply set the thresholds to fire the next pulse OCR1A for UP
  // OCR1B for DOWN

  if (c_ctc == n_ctc) // we are done
  {
    
# 889 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 889 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0; // clear all Timer/PWM functionality
    
# 890 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 890 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0;
    Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x0); //
    Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x0); //
    c_ctc = 0;
    
# 894 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x84)) 
# 894 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0; // does not help to do a MOVE after EXECCTCPWM...
    
# 895 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x6F)) 
# 895 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          &= ~(1 << 
# 895 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    0
# 895 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                         ); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_t = false;
    n_ctc--; // get rid of the null pulse (so we can add pulses with loadctc)
    return;
  }

  int16_t pulse_ms = ctc_data[c_ctc];
  c_ctc++;

  int32_t pulse_tick = 16E6 /* clock at 16 MHz*/ / 2 / 64 /* prescaler for PWM mode*/ * (int32_t)pulse_ms / 1000;

  if (pulse_tick > 0)
  {
    
# 908 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 908 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = pulse_tick;
    
# 909 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 909 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0;
  }
  else
  {
    
# 913 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 913 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0;
    
# 914 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 914 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = -pulse_tick;
  }
}


# 918 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
extern "C" void __vector_35 /* Timer/Counter3 Overflow */ (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_35 /* Timer/Counter3 Overflow */ (void) 
# 918 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                    // for digitimer shocks
{
  // we simply count the number of pulses and then stop
  // ISR is called when counter has finished i.e. pulse has been generated

  if (c_pulse == n_pulse - 1) // almost done
    
# 924 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x98)) 
# 924 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0; // create null pulse
  // as we fire ISR on TOP, we cut the last pulse in half therefore we do one more and set the last one to 0

  if (c_pulse == n_pulse) // done
  {
    
# 929 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x90)) 
# 929 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0; // clear all Timer/PWM functionality
    
# 930 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x91)) 
# 930 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = 0;

    // PORTE &= ~(1 << PE3); // probably not necessary set PE3 low
    c_pulse = 0;
    
# 934 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x94)) 
# 934 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0; //
    
# 935 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x71)) 
# 935 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          &= ~(1 << 
# 935 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                    0
# 935 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                         ); // can we disable interrupt when TCNT1 overflows in ISR ??? -> YES
    busy_d = false;
  }
  c_pulse++;
}

//***********************************************************************************
//*********** helper functions
//***********************************************************************************

void print_error(int8_t error_code)
{
  char buffer[4]; //3 digits and a zero
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
  char buffer[4]; //3 digits and a zero
  sprintf(buffer, "%03d", ok_code);
  Serial.print(buffer);
  if (debug_mode > 0)
  {
    Serial.print(" ");
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

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 987 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 987 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "+++"
# 987 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 987 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 988 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 988 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              " V:"
# 988 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 988 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.print(SWversion, 1);
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 990 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 990 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              " RAM:"
# 990 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 990 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.print(freeRam());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 992 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 992 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                " +++"
# 992 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 992 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 994 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 994 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "Debug level: "
# 994 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 994 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.println(debug_mode);

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 997 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 997 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "Last serial command: "
# 997 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 997 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.println(last_cmd);
}

//***********************************************************************************
//*********** status help over serial port evoked by serial command HELP;
//***********************************************************************************
void display_help()
{
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1006 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1006 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "Thermoino "
# 1006 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 1006 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.println(SWversion, 1);
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1008 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1008 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "Name: "
# 1008 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 1008 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  processGETID();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1010 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1010 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Only compatible with UseThermoinoPTB.m"
# 1010 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1010 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1011 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1011 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "**************************************"
# 1011 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1011 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1012 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1012 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Misc commands:"
# 1012 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1012 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1013 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1013 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "--------------"
# 1013 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1013 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1014 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1014 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "VER           - Print version"
# 1014 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1014 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1015 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1015 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "DIAG          - Get diagnostics"
# 1015 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1015 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1016 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1016 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "GETTIME       - Get Arduino time in ms"
# 1016 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1016 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1017 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1017 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "DEBUG;XX      - Set debug state (0: OFF) (1: ON))"
# 1017 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1017 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1018 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1018 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "HELP          - This command"
# 1018 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1018 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1019 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1019 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "STATUS        - check whether anything (thermode or digitimer) is running"
# 1019 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1019 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1020 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1020 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "KILL          - stop all activity (thermode and digitimer) "
# 1020 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1020 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1021 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1021 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "SETID         - enter new name for thermoino, saved permanently in EEPROM"
# 1021 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1021 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1022 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1022 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "GETID         - return name of thermoino from EEPROM"
# 1022 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1022 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1024 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1024 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Thermode related commands:"
# 1024 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1024 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1025 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1025 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "--------------------------"
# 1025 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1025 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1026 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1026 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "STATUS_T      - check whether thermode is running"
# 1026 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1026 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1027 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1027 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "KILL_T        - stop activity of thermode"
# 1027 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1027 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1028 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1028 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "MOVE;XX       - Move temp up/down for XX us"
# 1028 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1028 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1029 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1029 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "START         - Send 40ms TTL pulse to start thermode"
# 1029 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1029 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1030 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1030 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "INITCTC;xx    - Initialize complex time courses (ctc_data) with cycle xx in ms (500 max)"
# 1030 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1030 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1031 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1031 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              "LOADCTC;xx    - add pulse to ctc_data queue (xx in ms) -xx means temperature decrease, max "
# 1031 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
              ); &__c[0];}))
# 1031 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
              )));
  Serial.print(500);
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1033 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1033 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                " items"
# 1033 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1033 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));

  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1035 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1035 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "QUERYCTC(;yy) - status of the ctc_data queue (yy=3 to get all entries)"
# 1035 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1035 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1036 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1036 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "EXECCTC       - execute ctc_data queue using precise PWM"
# 1036 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1036 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1037 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1037 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "FLUSHCTC      - reset ctc and clear ctc_data queue"
# 1037 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1037 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1039 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1039 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Digitimer related commands:"
# 1039 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1039 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1040 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1040 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "---------------------------"
# 1040 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1040 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1041 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1041 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "STATUS_D      - check whether digitimer is running"
# 1041 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1041 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1042 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1042 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "KILL_D        - stop activity of digitimer"
# 1042 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1042 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1043 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1043 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "SHOCK;nn(;yy) - Digitimer stimuli number (nn) @interval 1100us OR additionally specify interval between pulses (yy) in us (>1000) "
# 1043 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1043 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1044 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1044 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "D188;xx       - select D188 channel xx (1..8)"
# 1044 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1044 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
}

void display_error_codes()
{
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1049 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1049 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "Error codes:"
# 1049 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1049 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  for (int8_t i = 1; i < N_ERROR_CODES; i++) // skip first error code
  {
    Serial.print(-i);
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1053 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1053 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                " "
# 1053 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1053 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.println(error_str[i]);
  }

  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1057 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1057 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "OK codes:"
# 1057 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1057 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
  for (int8_t i = 1; i < N_OK_CODES; i++) // skip first ok code
  {
    Serial.print(i);
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1061 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1061 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                " "
# 1061 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1061 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.println(ok_str[i]);
  }
}
//***********************************************************************************
//*********** subfunction to reset ctc_data variables
//***********************************************************************************
void reset_ctc()
{
  ctc_bin_ms = 0;
  n_ctc = 0;
  c_ctc = 0;
  memset_volatile(ctc_data, 0, sizeof(ctc_data));
  // reset everything
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp(int32_t ms)
{
  if (debug_mode > 0)
  {
    Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1085 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1085 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                "ramp_temp: "
# 1085 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                ); &__c[0];}))
# 1085 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                )));
    Serial.print(ms);
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1087 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1087 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "ms"
# 1087 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1087 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
  }
  
# 1089 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("cli" ::: "memory")
# 1089 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ;
  count_down_ms = ms;
  
# 1091 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 1091 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;
  
# 1092 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 1092 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0;

  
# 1094 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 1094 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        |= (1 << 
# 1094 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                 3
# 1094 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                      );
  // set Output Compare Register to (250 - 1) ticks
  
# 1096 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x88)) 
# 1096 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0xF9;
  // TCNT1
  
# 1098 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 1098 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0;
  // TIMSK4
  // Set Timer Interrupt Mask Register to
  // Clear Timer on Compare channel A for timer 4
  
# 1102 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)((0x16) + 0x20)) 
# 1102 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       |= (1 << 
# 1102 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                1
# 1102 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ); // very important otherwise ISR will immediately be executed
  
# 1103 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x6F)) 
# 1103 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        |= (1 << 
# 1103 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                 1
# 1103 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       );
  busy_t = true;
  if (ms < 0)
  {
    if (debug_mode > 0)
    {
      Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1109 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1109 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "Ramping down:  "
# 1109 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1109 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
      Serial.println(-ms);
    }
    Fast_digitalWrite(0xC7 /*Pin 13*/, 0x1); //
    Fast_digitalWrite(0xB5 /*Pin 9 OC1A*/, 0x1); //
  }
  if (ms > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1119 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1119 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "Ramping up:  "
# 1119 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1119 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
      Serial.println(ms);
    }
    Fast_digitalWrite(0xC7 /*Pin 13*/, 0x1); //
    Fast_digitalWrite(0xB6 /*Pin 10 OC1B*/, 0x1); //
  }
  
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        |= (1 << 
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                 1
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                     ) | (1 << 
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                               0
# 1125 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                   ); // start clock
  
# 1126 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 1126 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      ;
}

//***********************************************************************************
//*********** Prepare timer 1
//***********************************************************************************

void osp_setup(uint8_t which, int32_t prescaler)
{
  
# 1135 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 1135 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
        = 0; // Halt counter by setting clock select bits to 0 (No clock source).
  // This keeps anything from happening while we get set up

  
# 1138 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x84)) 
# 1138 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
       = 0; // Start counting at bottom.

  
# 1140 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
 (*(volatile uint16_t *)(0x86)) 
# 1140 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
      = 0; // Set TOP to 0, Mode 14. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
  // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX
  // and then overflow back to 0 and get locked up again.

  if (which == 1 /**/)
  {
    
# 1146 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 1146 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0xffff;
    
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = (1 << 
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  6
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                        ) | (1 << 
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                  7
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                        ) | (1 << 
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                  1
# 1147 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                       ); // OC1A=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
    // Set OC1A to output, pick your board- Uno vs 2560
    // DDRB = (1<<1);     // Set pin to output (Note that OC1A = GPIO port PB1 = Arduino Digital Pin D9 Uno)
    //DDRB = (1 << 5); // Set pin to output (Note that OC1A = GPIO port PB5 = Arduino Digital Pin D11 Mega2560)
  }
  else if (which == 2 /**/)
  {
    
# 1154 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 1154 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
         = 0xffff;
    
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = (1 << 
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  4
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                        ) | (1 << 
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                  5
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                        ) | (1 << 
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                  1
# 1155 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                       ); // OC1B=Set on Match, clear on BOTTOM. Mode 14 Fast PWM. p.131
    // Set OC1B to output, pick your board- Uno vs 2560
    // DDRB = (1<<2);     // Set pin to output (Note that OC1B = GPIO port PB2 = Arduino Digital Pin D10 Uno)
    //DDRB = (1 << 6); // Set pin to output (Note that OC1B = GPIO port PB6 = Arduino Digital Pin D12 Mega2560)
  }

  else
  {
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1163 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1163 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "ERROR only OC1A or OC1B supported"
# 1163 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1163 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
  }

  //   (using Chris Hahn's notation here)
  // Prescaler  Setup - Choose one of these, then choose a matching "wait" delay statement below.
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10); // Prescaler = 1; Start counting now. Max ~4mS
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler = 8; Start counting now. Max ~32mS, starts in ~10uS or better
  // TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS11); // Prescaler = 64; Start counting now. Max ~.26 sec, starts in ~20uS or better
  if (prescaler == 256)
  {
    
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = (1 << 
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  3
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ) | (1 << 
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                 4
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                      ) | (1 << 
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                2
# 1173 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                    ); // Prescaler = 256; Start counting now. Max ~1.05 sec, starts in ~64uS or better
  }
  else if (prescaler == 1024)
  {
    
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
          = (1 << 
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  3
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                       ) | (1 << 
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                 4
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                      ) | (1 << 
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                0
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                    ) | (1 << 
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                                                              2
# 1177 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                                                                  ); // Prescaler = 1024; Start counting now. Max ~4 sec, starts in ~180uS or better
  }
  else
  {
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1181 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1181 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "ERROR only 256 or 1024 supported"
# 1181 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1181 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
  }
}

//***********************************************************************************
//*********** Function to ramp up temperature
//***********************************************************************************

void ramp_temp_prec(int32_t o_us) // specifiy in us
{
  int32_t o_tic;
  // Serial.print(F("ramp_temp_prec: "));
  // Serial.print(o_us);
  // Serial.println(F("us"));

  if (((o_us)>0?(o_us):-(o_us)) < 1048560) // we can use 256 as a prescaler
  {
    prescaler = 256; // we get up to ~4 s pulses
  }
  else
  {
    prescaler = 1024;
  }

  cps = 16E6 /* clock at 16 MHz*/ / prescaler; // tics per second
  // now convert us into tics
  o_tic = (int64_t)o_us * cps / 1000000; // convert from us to tics

  if (o_tic < 0)
  {
    o_tic = ((o_tic)>0?(o_tic):-(o_tic));

    if (debug_mode > 0)
    {
      Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1215 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1215 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "Precision-ramping down:  "
# 1215 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1215 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
      Serial.println(o_tic);
    }
    osp_setup(1 /**/, prescaler);
    { uint16_t m = 0xffff - (o_tic - 1); 
# 1219 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x88)) 
# 1219 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
   = m; { _delay_us(65); } /* ...for prescaler = 1024*/; 
# 1219 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x84)) 
# 1219 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
   = m - 1; } /* for prescaler > 1*/ // Use this for prescaler > 1!
  }
  else if (o_tic > 0)
  {
    if (debug_mode > 0)
    {
      Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1225 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1225 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  "Precision-ramping up:  "
# 1225 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
                  ); &__c[0];}))
# 1225 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
                  )));
      Serial.println(o_tic);
    }
    osp_setup(2 /**/, prescaler);
    { uint16_t m = 0xffff - (o_tic - 1); 
# 1229 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x8A)) 
# 1229 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
   = m; { _delay_us(65); } /* ...for prescaler = 1024*/; 
# 1229 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino" 3
   (*(volatile uint16_t *)(0x84)) 
# 1229 "c:\\Users\\buechel\\Documents\\Arduino\\Thermoino_32u4\\Thermoino_32u4.ino"
   = m - 1; } // Use this for prescaler > 1!
  }
}

void memset_volatile(volatile void *s, char c, size_t n)
{
  volatile char *p = s;
  while (n-- > 0)
  {
    *p++ = c;
  }
}
