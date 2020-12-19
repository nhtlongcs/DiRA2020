#include "LCDI2C.h"
#include <thread>
namespace dira_lcd {
LCDI2C::~LCDI2C() { LCDClear(); }

void LCDI2C::LCDInit(I2C *i2c_dev, unsigned char lcd_addr,
                     unsigned char lcd_cols, unsigned char lcd_rows) {
  addr = lcd_addr;
  cols = lcd_cols;
  rows = lcd_rows;
  backlight = LCD_BACKLIGHT;
  i2c_device = i2c_dev;
  LCDBegin(lcd_cols, lcd_rows);
}

void LCDI2C::LCDBegin(unsigned char cols, unsigned char rows,
                      unsigned char char_size) {
  display_function = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  num_lines = rows;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  expanderWrite(backlight);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  write4Bits(0x03 << 4);
  std::this_thread::sleep_for(std::chrono::microseconds(4500));
  write4Bits(0x03 << 4);
  std::this_thread::sleep_for(std::chrono::microseconds(4500));
  write4Bits(0x03 << 4);
  std::this_thread::sleep_for(std::chrono::microseconds(150));
  write4Bits(0x02 << 4);

  LCDCommand(LCD_FUNCTIONSET | display_function);
  display_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LCDOn();
  LCDClear();
  display_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  LCDCommand(LCD_ENTRYMODESET | display_mode);
  LCDHome();
}

void LCDI2C::LCDClear() {
  LCDCommand(LCD_CLEARDISPLAY);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void LCDI2C::LCDHome() {
  LCDCommand(LCD_RETURNHOME);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void LCDI2C::LCDOff() {
  display_control &= ~LCD_DISPLAYON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDOn() {
  display_control |= LCD_DISPLAYON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDBlinkOn() {
  display_control |= LCD_BLINKON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDBlinkOff() {
  display_control &= ~LCD_BLINKON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDCursorOn() {
  display_control |= LCD_CURSORON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDCursorOff() {
  display_control &= ~LCD_CURSORON;
  LCDCommand(LCD_DISPLAYCONTROL | display_control);
}

void LCDI2C::LCDScrollLeft() {
  LCDCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCDI2C::LCDScrollRight() {
  LCDCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void LCDI2C::LCDLeftToRight() {
  display_mode |= LCD_ENTRYLEFT;
  LCDCommand(LCD_ENTRYMODESET | display_mode);
}

void LCDI2C::LCDRightToLeft() {
  display_mode &= ~LCD_ENTRYLEFT;
  LCDCommand(LCD_ENTRYMODESET | display_mode);
}

void LCDI2C::LCDBacklightOn() {
  backlight = LCD_BACKLIGHT;
  expanderWrite(0);
}

void LCDI2C::LCDBacklightOff() {
  backlight = LCD_NOBACKLIGHT;
  expanderWrite(0);
}

void LCDI2C::LCDAutoscroll() {
  display_mode |= LCD_ENTRYSHIFTINCREMENT;
  LCDCommand(LCD_ENTRYMODESET | display_mode);
}

void LCDI2C::LCDNoAutoscroll() {
  display_mode &= ~LCD_ENTRYSHIFTINCREMENT;
  LCDCommand(LCD_ENTRYMODESET | display_mode);
}

void LCDI2C::LCDCreateChar(unsigned char location, unsigned char *char_map) {
  location &= 0x07;
  LCDCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i = 0; i < 8; ++i) {
    (char_map[i]);
  }
}

void LCDI2C::LCDSetCursor(unsigned char col, unsigned char row) {
  int row_offset[] = {0x00, 0x40, 0x14, 0x54};
  if (row > num_lines) {
    row = num_lines - 1;
  }
  LCDCommand(LCD_SETDDRAMADDR | (col + row_offset[row]));
}

void LCDI2C::LCDPrintStr(const char *c) {
  while (*c) {
    LCDWrite(*c);
    c++;
  }
}

void LCDI2C::LCDCommand(unsigned char val) { send(val, 0); }

void LCDI2C::LCDWrite(unsigned char val) { send(val, RS); }

void LCDI2C::send(unsigned char value, unsigned char mode) {
  unsigned char h = value & 0xf0;
  unsigned char l = (value << 4) & 0xf0;
  write4Bits((h) | mode);
  write4Bits((l) | mode);
}

void LCDI2C::write4Bits(unsigned char val) {
  expanderWrite(val);
  pulseEnable(val);
}

void LCDI2C::expanderWrite(unsigned char data) {
  i2c_device->HALWrite(addr, 0x00, data | backlight, "");
}

void LCDI2C::pulseEnable(unsigned char data) {
  expanderWrite(data | EN);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  expanderWrite(data & ~EN);
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
}
}  // namespace dira_lcd