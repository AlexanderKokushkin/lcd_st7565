// STE2007 LCD display library
#ifndef __lcd_st7565__
#define __lcd_st7565__

#include "driver/gpio.h"
#include <stdint.h>
#include <esp32/rom/ets_sys.h>

#include <fonts_5x8.h>
#include <fonts_18x32.h> // mostly for clocks
#include <fonts_16x24.h> // 6 digits not stretched, 3 stretched

class Lcd7565{ 

  static constexpr gpio_num_t lcd_rst = GPIO_NUM_32;
  static constexpr gpio_num_t lcd_rs  = GPIO_NUM_33; // A0/RegisterSelect/DataOrCtrl 
  static constexpr gpio_num_t lcd_scl = GPIO_NUM_25; // clk~clock
  static constexpr gpio_num_t lcd_si  = GPIO_NUM_26; // sid~SerialInputData

 public:
  static const uint16_t type = 7565;
  static const uint8_t txtWidth = 21; 
  static const uint8_t txtHeigth = 8;
  inline static char lazy[30] = {0}; 
  static void init(); 
  static void gotoXY( uint8_t x, uint8_t y);
  static void string(const char* ch, bool inverted = false);
  static void stretchedString(uint8_t x, uint8_t y, const char* ch, bool inverted = false);  
  static void clear();

  // forced (?) to became public
  static void character(uint8_t ch, bool inverted = false);
  static void stretchedCharacter(uint8_t ch, uint8_t subRow, bool inverted = false);
  static void bitmap_P(const uint8_t* bitmap, uint16_t size );
  // 16x24 -> stretched -> 32x48
  static void threeStretchedDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace=4);
  // 16x24 not stretched
  static void sixBigDigits(uint32_t value, uint8_t _row=0, uint8_t invertedDecimalPlace=7);

  // forced as well
  static const uint8_t cmd = 0;
  static const uint8_t data = 1;
  static void write(uint8_t dc, uint8_t data);
  inline static bool doubled = false; 
 //protected:
  static const uint8_t INTERNAL_RESET = 0xE2;
  static const uint8_t maxX = 128; // not used here
  static const uint8_t maxY = 64; // not used here
  static uint8_t stretchVerticalBar(const uint8_t v, const uint8_t subRow);
};
#endif           