/******************************************************************************
 Copyright © 2012-2015 Martin Karsten

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
#include "kernel/Output.h"
#include "machine/APIC.h"
#include "machine/Machine.h"
#include "devices/Keyboard.h"

// define constants for non-character keys
enum : Keyboard::KeyCode {
  // special ASCII codes
  KEY_BACKSPACE = '\b', // 0x08
  KEY_TAB       = '\t', // 0x09
  KEY_ENTER     = '\n', // 0x0D
  KEY_ESCAPE    = 0x1B,
  KEY_BACKSLASH = '\\', // 0x5C
  KEY_QUOTE     = '\'', // 0x60
  KEY_DELETE    = 0x7F,

  // Function keys
  KEY_F1 = 0x1000,
  KEY_F2,
  KEY_F3,
  KEY_F4,
  KEY_F5,
  KEY_F6,
  KEY_F7,
  KEY_F8,
  KEY_F9,
  KEY_F10,
  KEY_F11,
  KEY_F12,

  // Other top row keys
  KEY_PRTSCR,
  KEY_SCROLLLOCK,
  KEY_PAUSE,

  // Keys on main key block: TAB, BACKSPACE and ENTER have ASCII codes
  KEY_CAPSLOCK,
  KEY_LSHIFT,
  KEY_LCTRL,
  KEY_LALT,
  KEY_LGUI,
  KEY_RSHIFT,
  KEY_RCTRL,
  KEY_RALT,
  KEY_RGUI,

  // Navigation keys
  KEY_UP,
  KEY_DOWN,
  KEY_LEFT,
  KEY_RIGHT,
  KEY_INSERT,
  KEY_HOME,
  KEY_END,
  KEY_PGUP,
  KEY_PGDOWN,

  // Numeric keypad
  KEY_KP_DOT,
  KEY_KP_0,
  KEY_KP_1,
  KEY_KP_2,
  KEY_KP_3,
  KEY_KP_4,
  KEY_KP_5,
  KEY_KP_6,
  KEY_KP_7,
  KEY_KP_8,
  KEY_KP_9,
  KEY_KP_NUMLOCK,
  KEY_KP_DIVIDE,
  KEY_KP_ASTERISK,
  KEY_KP_MINUS,
  KEY_KP_PLUS,
  KEY_KP_ENTER,

  KEY_UNKNOWN
};

// xt scan code set; index is make code
static const int tableXTtoKey[][2] = {
    // scancode      // extended
  { KEY_UNKNOWN,     KEY_UNKNOWN }, //0x00
  { KEY_ESCAPE,      KEY_UNKNOWN }, //0x01, escape
  { '1',             KEY_UNKNOWN },
  { '2',             KEY_UNKNOWN },
  { '3',             KEY_UNKNOWN },
  { '4',             KEY_UNKNOWN },
  { '5',             KEY_UNKNOWN },
  { '6',             KEY_UNKNOWN },
  { '7',             KEY_UNKNOWN }, //0x08
  { '8',             KEY_UNKNOWN },
  { '9',             KEY_UNKNOWN },
  { '0',             KEY_UNKNOWN },
  { '-',             KEY_UNKNOWN },
  { '=',             KEY_UNKNOWN },
  { KEY_BACKSPACE,   KEY_UNKNOWN },
  { KEY_TAB,         KEY_UNKNOWN },
  { 'q',             KEY_UNKNOWN }, //0x10
  { 'w',             KEY_UNKNOWN },
  { 'e',             KEY_UNKNOWN },
  { 'r',             KEY_UNKNOWN },
  { 't',             KEY_UNKNOWN },
  { 'y',             KEY_UNKNOWN },
  { 'u',             KEY_UNKNOWN },
  { 'i',             KEY_UNKNOWN },
  { 'o',             KEY_UNKNOWN }, //0x18
  { 'p',             KEY_UNKNOWN },
  { '[',             KEY_UNKNOWN },
  { ']',             KEY_UNKNOWN },
  { KEY_ENTER,       KEY_KP_ENTER },
  { KEY_LCTRL,       KEY_RCTRL },
  { 'a',             KEY_UNKNOWN },
  { 's',             KEY_UNKNOWN },
  { 'd',             KEY_UNKNOWN }, //0x20
  { 'f',             KEY_UNKNOWN },
  { 'g',             KEY_UNKNOWN },
  { 'h',             KEY_UNKNOWN },
  { 'j',             KEY_UNKNOWN },
  { 'k',             KEY_UNKNOWN },
  { 'l',             KEY_UNKNOWN },
  { ';',             KEY_UNKNOWN },
  { '\'',            KEY_UNKNOWN }, //0x28, quote
  { '`',             KEY_UNKNOWN },
  { KEY_LSHIFT,      KEY_PRTSCR },
  { '\\',            KEY_UNKNOWN }, //0x2b, backslash
  { 'z',             KEY_UNKNOWN },
  { 'x',             KEY_UNKNOWN },
  { 'c',             KEY_UNKNOWN },
  { 'v',             KEY_UNKNOWN },
  { 'b',             KEY_UNKNOWN }, //0x30
  { 'n',             KEY_UNKNOWN },
  { 'm',             KEY_UNKNOWN },
  { ',',             KEY_UNKNOWN },
  { '.',             KEY_UNKNOWN },
  { '/',             KEY_KP_DIVIDE },
  { KEY_RSHIFT,      KEY_UNKNOWN },
  { KEY_KP_ASTERISK, KEY_PRTSCR },
  { KEY_LALT,        KEY_RALT },    //0x38
  { ' ',             KEY_UNKNOWN },
  { KEY_CAPSLOCK,    KEY_UNKNOWN },
  { KEY_F1,          KEY_UNKNOWN },
  { KEY_F2,          KEY_UNKNOWN },
  { KEY_F3,          KEY_UNKNOWN },
  { KEY_F4,          KEY_UNKNOWN },
  { KEY_F5,          KEY_UNKNOWN },
  { KEY_F6,          KEY_UNKNOWN }, //0x40
  { KEY_F7,          KEY_UNKNOWN },
  { KEY_F8,          KEY_UNKNOWN },
  { KEY_F9,          KEY_UNKNOWN },
  { KEY_F10,         KEY_UNKNOWN },
  { KEY_KP_NUMLOCK,  KEY_UNKNOWN },
  { KEY_SCROLLLOCK,  KEY_UNKNOWN },
  { KEY_KP_7,        KEY_HOME },
  { KEY_KP_8,        KEY_UP },      //0x48
  { KEY_KP_9,        KEY_PGUP },
  { KEY_KP_MINUS,    KEY_UNKNOWN },
  { KEY_KP_4,        KEY_LEFT },
  { KEY_KP_5,        KEY_UNKNOWN },
  { KEY_KP_6,        KEY_RIGHT },
  { KEY_KP_PLUS,     KEY_UNKNOWN },
  { KEY_KP_1,        KEY_END },
  { KEY_KP_2,        KEY_DOWN },    //0x50
  { KEY_KP_3,        KEY_PGDOWN },
  { KEY_KP_0,        KEY_INSERT },
  { KEY_KP_DOT,      KEY_DELETE },
  { KEY_UNKNOWN,     KEY_UNKNOWN },
  { KEY_UNKNOWN,     KEY_UNKNOWN },
  { KEY_UNKNOWN,     KEY_UNKNOWN },
  { KEY_F11,         KEY_UNKNOWN },
  { KEY_F12,         KEY_UNKNOWN }
};

// translation table from AT to XT scan codes
static const uint8_t tableATtoXT[0x80] = {
  0xff,0x43,0x41,0x3f,0x3d,0x3b,0x3c,0x58,0x64,0x44,0x42,0x40,0x3e,0x0f,0x29,0x59,
  0x65,0x38,0x2a,0x70,0x1d,0x10,0x02,0x5a,0x66,0x71,0x2c,0x1f,0x1e,0x11,0x03,0x5b,
  0x67,0x2e,0x2d,0x20,0x12,0x05,0x04,0x5c,0x68,0x39,0x2f,0x21,0x14,0x13,0x06,0x5d,
  0x69,0x31,0x30,0x23,0x22,0x15,0x07,0x5e,0x6a,0x72,0x32,0x24,0x16,0x08,0x09,0x5f,
  0x6b,0x33,0x25,0x17,0x18,0x0b,0x0a,0x60,0x6c,0x34,0x35,0x26,0x27,0x19,0x0c,0x61,
  0x6d,0x73,0x28,0x74,0x1a,0x0d,0x62,0x6e,0x3a,0x36,0x1c,0x1b,0x75,0x2b,0x63,0x76,
  0x55,0x56,0x77,0x78,0x79,0x7a,0x0e,0x7b,0x7c,0x4f,0x7d,0x4b,0x47,0x7e,0x7f,0x6f,
  0x52,0x53,0x50,0x4c,0x4d,0x48,0x01,0x45,0x57,0x4e,0x51,0x4a,0x37,0x49,0x46,0x54,
};

// invalid scan code. Used to indicate the last scan code is not to be reused
// static const int INVALID_SCANCODE = 0;

static int scanCodeSet = 2;

// X11 keycodes (see xmodmap -pk) -> not used currently
#if 0
static const uint8_t x11Code[2][0x80] = {
{   0,   9,  10,  11,  12,  13,  14,  15, 
   16,  17,  18,  19,  20,  21,  22,  23, 
   24,  25,  26,  27,  28,  29,  30,  31, 
   32,  33,  34,  35,  36,  37,  38,  39, 
   40,  41,  42,  43,  44,  45,  46,  47, 
   48,  49,  50,  51,  52,  53,  54,  55, 
   56,  57,  58,  59,  60,  61,  62,  63, 
   64,  65,  66,  67,  68,  69,  70,  71, 
   72,  73,  74,  75,  76,  77,  78,  79, 
   80,  81,  82,  83,  84,  85,  86,  87, 
   88,  89,  90,  91,   0,   0,   0,  95, 
   96,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0 },
{   0,  75,   0,  71,  69,  67,  68,  96, 
    0,  76,  74,  72,  70,  23,  49,   0, 
    0,  64,  50,   0,  37,  24,  10,   0, 
    0,   0,  52,  39,  38,  25,  11,   0, 
    0,  54,  53,  40,  26,  13,  12,   0, 
    0,  65,  55,  41,  28,  27,  14,   0, 
    0,  57,  56,  43,  42,  29,  15,   0, 
    0,   0,  58,  44,  30,  16,  17,   0, 
    0,  59,  45,  31,  32,  19,  18,   0, 
    0,  60,  61,  46,  47,  33,  20,   0, 
    0,   0,  48,   0,  34,  21,   0,   0, 
   66,  62,  36,  35,   0,  51,   0,   0,
    0,   0,  22,   0,   0,  87,   0,  83,
   79,   0,   0,   0,  90,  91,  88,  84,
   85,  80,   9,  77,  95,  86,  89,  82,
   63,  81,  78,   0,   0,   0,   0,  73 }
};
#endif

static const uint8_t DataPort        = 0x60;
static const uint8_t StatusRegister  = 0x64;
static const uint8_t CommandRegister = 0x64;

enum CTRL_CMDS {
  CTRL_CMD_READ_CONFIG       = 0x20,
  CTRL_CMD_WRITE_CONFIG      = 0x60,
  CTRL_CMD_DISABLE_P2        = 0xA7,
  CTRL_CMD_ENABLE_P2         = 0xA8,
  CTRL_CMD_TEST_P2           = 0xA9,
  CTRL_CMD_SELF_TEST         = 0xAA,
  CTRL_CMD_TEST_P1           = 0xAB,
  CTRL_CMD_DISABLE_P1        = 0xAD,
  CTRL_CMD_ENABLE_P1         = 0xAE,
  CTRL_CMD_READ_CTRL_INPUT   = 0xC0,
  CTRL_CMD_READ_CTRL_OUTPUT  = 0xD0,
  CTRL_CMD_WRITE_CTRL_OUTPUT = 0xD1,
  CTRL_CMD_WRITE_P1_OUTPUT   = 0xD2, // fake receive
  CTRL_CMD_WRITE_P2_OUTPUT   = 0xD3, // fake receive
  CTRL_CMD_WRITE_TO_P2       = 0xD4,
  CTRL_CMD_READ_TEST_INPUTS  = 0xE0,
  CTRL_CMD_SYSTEM_RESET      = 0xFE,
};

enum CTRL_STATS_MASK {
  CTRL_STATS_MASK_OUT_BUF    = 0x01, //00000001
  CTRL_STATS_MASK_IN_BUF     = 0x02, //00000010
  CTRL_STATS_MASK_SYSTEM     = 0x04, //00000100
  CTRL_STATS_MASK_CMD_DATA   = 0x08, //00001000
  CTRL_STATS_MASK_LOCKED     = 0x10, //00010000
  CTRL_STATS_MASK_AUX_BUF    = 0x20, //00100000
  CTRL_STATS_MASK_TIMEOUT    = 0x40, //01000000
  CTRL_STATS_MASK_PARITY     = 0x80  //10000000
};

enum DEV_CMDS {
  DEV_CMD_SET_LED            = 0xED,
  DEV_CMD_ECHO               = 0xEE,
  DEV_CMD_SCAN_CODE_SET      = 0xF0,
  DEV_CMD_ID                 = 0xF2,
  DEV_CMD_AUTODELAY          = 0xF3,
  DEV_CMD_ENABLE             = 0xF4,
  DEV_CMD_DISABLE            = 0xF5,
  DEV_CMD_DEFAULT_PARAMETERS = 0xF6,
  DEV_CMD_ALL_AUTO           = 0xF7,
  DEV_CMD_ALL_MAKEBREAK      = 0xF8,
  DEV_CMD_ALL_MAKEONLY       = 0xF9,
  DEV_CMD_ALL_MAKEBREAK_AUTO = 0xFA,
  DEV_CMD_SINGLE_AUTOREPEAT  = 0xFB,
  DEV_CMD_SINGLE_MAKEBREAK   = 0xFC,
  DEV_CMD_SINGLE_BREAKONLY   = 0xFD,
  DEV_CMD_RESEND             = 0xFE,
  DEV_CMD_RESET              = 0xFF
};

// scan error codes ------------------------------------------
enum ERROR {
  ERR_BUF_OVERRUN            = 0x00,
  ERR_ID_RET                 = 0x83AB,
  ERR_BAT                    = 0xAA,
  ERR_ECHO_RET               = 0xEE,
  ERR_ACK                    = 0xFA,
  ERR_BAT_FAILED             = 0xFC,
  ERR_DIAG_FAILED            = 0xFD,
  ERR_RESEND_CMD             = 0xFE,
  ERR_KEY                    = 0xFF
};

static bool ControllerCheckReadBuffer() {
  return CPU::in8(StatusRegister) & CTRL_STATS_MASK_OUT_BUF;
}

static bool ControllerCheckWriteBuffer() {
  return CPU::in8(StatusRegister) & CTRL_STATS_MASK_IN_BUF;
}

static void ControllerCommand(uint8_t cmd) {
  while (ControllerCheckWriteBuffer());
  CPU::out8(CommandRegister, cmd);   // send command byte to controller
}

static void DrainReadBuffer() {
  while (ControllerCheckReadBuffer()) {
    uint8_t data = CPU::in8(DataPort);
    DBG::out1(DBG::Devices, ' ', FmtHex(data));
  }
}

static uint8_t DataRead() {
  while (!ControllerCheckReadBuffer());
  return CPU::in8(DataPort);         // read data byte from device/controller
}

template<bool second=false>
static void DataWrite(uint8_t data) {
  while (ControllerCheckWriteBuffer());
  if (second) ControllerCommand(CTRL_CMD_WRITE_TO_P2); // send next byte to moouse
  CPU::out8(DataPort, data);         // send data byte to device/controller
}

template<bool second=false>
static uint8_t DeviceCommandOnce(uint8_t cmd) {
  DataWrite<second>(cmd);
  return DataRead();
}

template<bool second=false>
static uint8_t DeviceCommand(uint8_t cmd) {
  uint8_t retcode;
  do retcode = DeviceCommandOnce<second>(cmd); while (retcode == ERR_RESEND_CMD);
  return retcode;
}

template<bool second=false>
static void DeviceCommandCheck(uint8_t cmd) {
  uint8_t retcode = DeviceCommand(cmd);
  KCHECK1(retcode == ERR_ACK, FmtHex(retcode));
}

static bool SelfTest() {
  ControllerCommand(CTRL_CMD_SELF_TEST); // send command
  return (DataRead() == 0x55);
}

// sets LEDs: VirtualBox sends ACKs asynchronously
static void SetLeds(uint8_t led) {
  DataWrite(DEV_CMD_SET_LED);
  DataWrite(led);
}

void Keyboard::init() {                      // assume interrupts are disabled
  DBG::out1(DBG::Devices, "PS/2:");
  ControllerCommand(CTRL_CMD_ENABLE_P1);     // try enable keyboard
  ControllerCommand(CTRL_CMD_ENABLE_P2);     // try enable mouse
  DrainReadBuffer();

  DBG::out1(DBG::Devices, " CFG:");
  ControllerCommand(CTRL_CMD_READ_CONFIG);   // ask for configuration byte
  uint8_t config = DataRead();               // read configuration byte
  DBG::out1(DBG::Devices, FmtHex(config));
  config &= ~(1 << 0);                       // disable keyboard interrupts
  config &= ~(1 << 1);                       // disable mouse interrupts
  config &= ~(1 << 6);                       // disable XT translation
  ControllerCommand(CTRL_CMD_WRITE_CONFIG);  // announce configuration byte
  DataWrite(config);                         // write back configuration byte

  if (!SelfTest()) return;
  DBG::out1(DBG::Devices, " ST");

  if (!(config & (1 << 4))) {                // test for keyboard
    DBG::out1(DBG::Devices, " keyb:");
    ControllerCommand(CTRL_CMD_TEST_P1);     // test device
    uint8_t data = DataRead();
    KCHECK1( data == 0x00, FmtHex(data));
    DBG::out1(DBG::Devices, 'T');
    DrainReadBuffer();
#if 0
    // TODO: should try the command sequence with timeouts
    DeviceCommandCheck(DEV_CMD_DISABLE);     // disable scanning
    DBG::out1(DBG::Devices, 'S');
    DeviceCommandCheck(DEV_CMD_RESET);       // reset device
    DBG::out1(DBG::Devices, 'R');
    data = DataRead();
    KCHECK1( data == ERR_BAT, FmtHex(data));
    DBG::out1(DBG::Devices, 'R');
    DeviceCommandCheck(DEV_CMD_SCAN_CODE_SET); // get/set scancode set
    DeviceCommandCheck(0x02);                  // set AT scancode set
    DeviceCommandCheck(DEV_CMD_SCAN_CODE_SET); // get/set scancode set
    data = DeviceCommand(0x00);                // get scancode set (qemu quirk)
    if (data == ERR_ACK) data = DataRead();    // normal: read after ACK
    DBG::out1(DBG::Devices, '/', FmtHex(data));
    switch (data) {
      case 0x01: case 0x43: scanCodeSet = 1; break; // XT scancode set
      case 0x02: case 0x41: scanCodeSet = 2; break; // AT scancode set
      case 0x03: case 0x3F: scanCodeSet = 3; break; // PS/2 scancode set
      default: KABORT0();
    }
    DBG::out1(DBG::Devices, '/', FmtHex(scanCodeSet));

    DeviceCommandCheck(DEV_CMD_ENABLE);      // enable scanning
    DBG::out1(DBG::Devices, "/A");
#endif
    config |= (1 << 0);                      // enable interrupts (below)
  }

#if 0
  if (!(config & (1 << 5))) {                 // test for mouse
    DBG::out1(DBG::Devices, " mouse:");
    ControllerCommand(CTRL_CMD_TEST_P2);      // test device
    uint8_t data = DataRead();
    KCHECK1( data == 0x00, FmtHex(data));
    DBG::out1(DBG::Devices, 'T');
    DrainReadBuffer();
    DeviceCommandCheck<true>(DEV_CMD_DISABLE);// disable scanning
    DBG::out1(DBG::Devices, 'S');
    DeviceCommandCheck<true>(DEV_CMD_RESET);  // reset device
    DBG::out1(DBG::Devices, 'R');
    data = DataRead();
    KCHECK1( data == ERR_BAT, FmtHex(data));
    DBG::out1(DBG::Devices, 'R');

    DeviceCommandCheck<true>(DEV_CMD_ENABLE); // enable scanning
    DBG::out1(DBG::Devices, "/A");
    config |= (1 << 1);                       // enable interrupts (below)
  }
#endif

  DBG::outl(DBG::Devices, " - ", FmtHex(config));

  if (config & (1 << 0)) Machine::registerIrqAsync(PIC::Keyboard, (funcvoid1_t)irqHandler, this);
  if (config & (1 << 1)) Machine::registerIrqSync(PIC::Mouse, 0xfc);

  ControllerCommand(CTRL_CMD_WRITE_CONFIG);  // write back configuration byte
  DataWrite(config);
}

inline void Keyboard::irqHandlerInternal() {
  // read scan code while the output buffer is full (scan code available)
  while (ControllerCheckReadBuffer()) {
    KeyCode code = DataRead();          // read the scan code

#if TESTING_KEYCODE_LOOP
    kbq.trySend(code);
  continue;
#endif

    switch (code) {                     // watch for errors
      case 0xE0:    KASSERT0(!extended); extended = 1; continue;
      case 0xE1:    KASSERT0(!extended); extended = 2; continue;
      case ERR_ACK: KCHECK0(acks > 0);  acks -= 1;    continue;
    }
    KCHECK0(!acks);

    if (scanCodeSet == 1) {
      if (code & 0x80) {                // check for encoded break code
        code &= ~0x80;
        is_break = true;
      }
    } else {
      if (code == 0xf0) {               // check for separate break code
        is_break = true;
  continue;
      } else {
        KASSERT1(code >= 0 && code < 0x80, code);
        code = tableATtoXT[code];       // translate to XT scancode set
      }
    }

    KeyCode key;
    if (extended == 2 && code == 0x1D) {        // 'pause' key sequence
      extended = 3; 
  continue;
    } else if (extended == 3 && code == 0x45) { // 'pause' key sequence
      key = KEY_PAUSE;
    } else {
      KASSERT1(extended >= 0 && extended <= 1, extended);
      key = tableXTtoKey[code][extended]; // get key code from XT set
    }
    extended = 0;

    if (is_break) {                     // break code
      is_break = false;
      switch (key) {                    // special keys released?
        case KEY_LCTRL:  case KEY_RCTRL:  ctrl =  false; break;
        case KEY_LSHIFT: case KEY_RSHIFT: shift = false; break;
        case KEY_LALT:   case KEY_RALT:   alt =   false; break;
      }
  continue;
    }

    switch (key) {                    // special keys pressed?
      case KEY_LCTRL:  case KEY_RCTRL:  ctrl =  true; continue;
      case KEY_LSHIFT: case KEY_RSHIFT: shift = true; continue;
      case KEY_LALT:   case KEY_RALT:   alt =   true; continue;
      case KEY_SCROLLLOCK: led ^= 1; SetLeds(led); acks += 2; continue;
      case KEY_KP_NUMLOCK: led ^= 2; SetLeds(led); acks += 2; continue;
      case KEY_CAPSLOCK:   led ^= 4; SetLeds(led); acks += 2; continue;
    }

    if (alt) {
      if (ctrl && key == KEY_DELETE) Reboot(); // ctrl+alt+del
      else kbq.trySend(KEY_ESCAPE);            // insert escape
    }

    if ((shift || get_capslock()) && key >= 'a' && key <= 'z') {
      key -= 32;
      goto send;
    }

    if (ctrl && key >= 'a' && key <= 'z') {
      key -= 96;
      goto send;
    }

    if (shift) switch (key) {
      case '1':  key = '!'; goto send;
      case '2':  key = '@'; goto send;
      case '3':  key = '#'; goto send;
      case '4':  key = '$'; goto send;
      case '5':  key = '%'; goto send;
      case '6':  key = '^'; goto send;
      case '7':  key = '&'; goto send;
      case '8':  key = '*'; goto send;
      case '9':  key = '('; goto send;
      case '0':  key = ')'; goto send;
      case '-':  key = '_'; goto send;
      case '=':  key = '+'; goto send;
      case ',':  key = '<'; goto send;
      case '.':  key = '>'; goto send;
      case '/':  key = '?'; goto send;
      case ';':  key = ':'; goto send;
      case '\'': key = '"'; goto send;
      case '[':  key = '{'; goto send;
      case ']':  key = '}'; goto send;
      case '`':  key = '~'; goto send;
      case '\\': key = '|'; goto send;
    }

    if (get_numlock()) switch (key) {
      case KEY_KP_DOT: key = '.'; goto send;
      case KEY_KP_0:   key = '0'; goto send;
      case KEY_KP_1:   key = '1'; goto send;
      case KEY_KP_2:   key = '2'; goto send;
      case KEY_KP_3:   key = '3'; goto send;
      case KEY_KP_4:   key = '4'; goto send;
      case KEY_KP_5:   key = '5'; goto send;
      case KEY_KP_6:   key = '6'; goto send;
      case KEY_KP_7:   key = '7'; goto send;
      case KEY_KP_8:   key = '8'; goto send;
      case KEY_KP_9:   key = '9'; goto send;
    }

    switch (key) {
      case KEY_KP_DOT:      key = KEY_DELETE;  goto send;
      case KEY_KP_0:        key = KEY_INSERT;  goto send;
      case KEY_KP_1:        key = KEY_END;     goto send;
      case KEY_KP_2:        key = KEY_DOWN;    goto send;
      case KEY_KP_3:        key = KEY_PGDOWN;  goto send;
      case KEY_KP_4:        key = KEY_LEFT;    goto send;
      case KEY_KP_5:        key = KEY_UNKNOWN; goto send;
      case KEY_KP_6:        key = KEY_RIGHT;   goto send;
      case KEY_KP_7:        key = KEY_HOME;    goto send;
      case KEY_KP_8:        key = KEY_UP;      goto send;
      case KEY_KP_9:        key = KEY_PGUP;    goto send;
      case KEY_KP_DIVIDE:   key = '/';         goto send;
      case KEY_KP_ASTERISK: key = '*';         goto send;
      case KEY_KP_MINUS:    key = '-';         goto send;
      case KEY_KP_PLUS:     key = '+';         goto send;
      case KEY_KP_ENTER:    key = KEY_ENTER;   goto send;
    }

send:
    kbq.trySend(key); // add key to key buffer!
  }
}

void Keyboard::irqHandler(Keyboard* keyb) {
  keyb->irqHandlerInternal();
}

// system reset: 11111110 to output port sets reset-system line low
void Keyboard::reset_system() {
  ControllerCommand(CTRL_CMD_WRITE_CTRL_OUTPUT);
  DataWrite(CTRL_CMD_SYSTEM_RESET);
}
