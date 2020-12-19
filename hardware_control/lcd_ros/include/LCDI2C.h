#pragma once
#include "Hal.h"

// commands
int constexpr LCD_CLEARDISPLAY = 0x01;
int constexpr LCD_RETURNHOME = 0x02;
int constexpr LCD_ENTRYMODESET = 0x04;
int constexpr LCD_DISPLAYCONTROL = 0x08;
int constexpr LCD_CURSORSHIFT = 0x10;
int constexpr LCD_FUNCTIONSET = 0x20;
int constexpr LCD_SETCGRAMADDR = 0x40;
int constexpr LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
int constexpr LCD_ENTRYRIGHT = 0x00;
int constexpr LCD_ENTRYLEFT = 0x02;
int constexpr LCD_ENTRYSHIFTINCREMENT = 0x01;
int constexpr LCD_ENTRYSHIFTDECREMENT = 0x00;

// flags for display on/off control
int constexpr LCD_DISPLAYON = 0x04;
int constexpr LCD_DISPLAYOFF = 0x00;
int constexpr LCD_CURSORON = 0x02;
int constexpr LCD_CURSOROFF = 0x00;
int constexpr LCD_BLINKON = 0x01;
int constexpr LCD_BLINKOFF = 0x00;

// flags for display/cursor shift
int constexpr LCD_DISPLAYMOVE = 0x08;
int constexpr LCD_CURSORMOVE = 0x00;
int constexpr LCD_MOVERIGHT = 0x04;
int constexpr LCD_MOVELEFT = 0x00;

// flags for function set
int constexpr LCD_8BITMODE = 0x10;
int constexpr LCD_4BITMODE = 0x00;
int constexpr LCD_2LINE = 0x08;
int constexpr LCD_1LINE = 0x00;
int constexpr LCD_5x10DOTS = 0x04;
int constexpr LCD_5x8DOTS = 0x00;

// flags for backlight control
int constexpr LCD_BACKLIGHT = 0x08;
int constexpr LCD_NOBACKLIGHT = 0x00;

int constexpr EN = 0x04;  // Enable bit
int constexpr RW = 0x02;  // Read/Write bit
int constexpr RS = 0x01;  // Register select bit

namespace dira_lcd {
class LCDI2C {
 public:
  virtual ~LCDI2C();

  void LCDInit(I2C *i2c_dev, unsigned char lcd_addr, unsigned char lcd_cols,
               unsigned char lcd_rows);
  void LCDBegin(unsigned char cols, unsigned char rows,
                unsigned char char_size = LCD_5x8DOTS);
  void LCDClear();
  void LCDHome();
  void LCDOff();
  void LCDOn();
  void LCDBlinkOn();
  void LCDBlinkOff();
  void LCDCursorOn();
  void LCDCursorOff();
  void LCDScrollLeft();
  void LCDScrollRight();
  void LCDLeftToRight();
  void LCDRightToLeft();
  void LCDBacklightOn();
  void LCDBacklightOff();
  void LCDAutoscroll();
  void LCDNoAutoscroll();
  void LCDCreateChar(unsigned char location, unsigned char *char_map);
  void LCDSetCursor(unsigned char col, unsigned char row);
  void LCDPrintStr(const char *c);
  void LCDCommand(unsigned char val);
  void LCDWrite(unsigned char val);

 private:
  I2C *i2c_device;

  unsigned char addr;
  unsigned char display_function;
  unsigned char display_control;
  unsigned char display_mode;
  unsigned char num_lines;
  unsigned char cols;
  unsigned char rows;
  unsigned char backlight;

 private:
  void send(unsigned char value, unsigned char mode);
  void write4Bits(unsigned char val);
  void expanderWrite(unsigned char data);
  void pulseEnable(unsigned char data);
};  // namespace dira_lcd
}  // namespace dira_lcd