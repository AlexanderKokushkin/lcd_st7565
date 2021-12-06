// ST7565 LCD display library
#ifndef __lcd_st7565__
#define __lcd_st7565__

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/mpu_wrappers.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <string.h>

// for ets_delay_us()
#if CONFIG_IDF_TARGET_ESP32
 #include <esp32/rom/ets_sys.h>
#endif
#if CONFIG_IDF_TARGET_ESP32C3
 #include <esp32c3/rom/ets_sys.h>
#endif

#include <fonts_5x8.h>
#include <fonts_18x32.h> // mostly for clocks
#include <fonts_16x24.h> // 6 digits not stretched, 3 stretched

/*******************************************************************************************
* usage:
* class Settings{
*    public:
*        static constexpr gpio_num_t lcd_rst = GPIO_NUM_1; // RSE
*        static constexpr gpio_num_t lcd_rs  = GPIO_NUM_2; // RS/A0/RegisterSelect/DataOrCtrl 
*        static constexpr gpio_num_t lcd_scl = GPIO_NUM_3; // SCL/clk~clock
*        static constexpr gpio_num_t lcd_si  = GPIO_NUM_4; // SI/sid~SerialInputData
* };
* using Lcd = Lcd7565_T<Settings>;
********************************************************************************************/

template<class T>
class Lcd7565_T{ 
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
  // 16x24 -> stretched -> 32x48
  static void threeStretchedDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace=4);
  // 16x24 not stretched
  static void sixBigDigits(uint32_t value, uint8_t _row=0, uint8_t invertedDecimalPlace=7);
 protected:
  static void character(uint8_t ch, bool inverted = false);
  static void stretchedCharacter(uint8_t ch, uint8_t subRow, bool inverted = false);
  static void bitmap_P(const uint8_t* bitmap, uint16_t size );
  static const uint8_t cmd = 0;
  static const uint8_t data = 1;
  static void write(uint8_t dc, uint8_t data);
  static const uint8_t maxX = 128; // not used here
  static const uint8_t maxY = 64; // not used here
  static uint8_t stretchVerticalBar(const uint8_t v, const uint8_t subRow);
};

template<class T> void Lcd7565_T<T>::write(uint8_t dc, uint8_t data){

  gpio_set_level(T::lcd_rs,dc); // set data or cmd mode
  for (uint8_t i = 0; i < 8; ++i){ // shiftout data
    gpio_set_level(T::lcd_scl,0);
    ets_delay_us(5);
    gpio_set_level(T::lcd_si, data & (1 << (7 - i)) );
    ets_delay_us(5);
    gpio_set_level(T::lcd_scl,1);
  }
}

template<class T> void Lcd7565_T<T>::init(){

    gpio_config_t tmp_io_conf{
        .pin_bit_mask = ((1ULL<<T::lcd_rst) | (1ULL<<T::lcd_rs) | (1ULL<<T::lcd_scl) | (1ULL<<T::lcd_si)),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,       
    };
    gpio_config(&tmp_io_conf);
    gpio_set_level(T::lcd_rst,0);
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_set_level(T::lcd_rst,1);

    #define CMD_SET_BIAS_7 0xA3  
    write(cmd,CMD_SET_BIAS_7); // LCD bias select
    #define CMD_SET_ADC_NORMAL  0xA0
    write(cmd,CMD_SET_ADC_NORMAL); // ADC select
    //#define CMD_SET_COM_NORMAL  0xC0
    //write(cmd,CMD_SET_COM_NORMAL); // SHL select

    #define CMD_SET_COM_REVERSE  0xC8
    write(cmd,CMD_SET_COM_REVERSE); // SHL select

    #define CMD_SET_DISP_START_LINE  0x40
    write(cmd,CMD_SET_DISP_START_LINE); // Initial display line
    #define CMD_SET_POWER_CONTROL  0x28
    write(cmd,CMD_SET_POWER_CONTROL | 0x4); // turn on voltage converter (VC=1, VR=0, VF=0)
    vTaskDelay(50 / portTICK_RATE_MS);   
    write(cmd,CMD_SET_POWER_CONTROL | 0x6); // turn on voltage regulator (VC=1, VR=1, VF=0)
    vTaskDelay(50 / portTICK_RATE_MS);
    write(cmd,CMD_SET_POWER_CONTROL | 0x7); // turn on voltage follower (VC=1, VR=1, VF=1)
    vTaskDelay(10 / portTICK_RATE_MS);
    #define CMD_SET_RESISTOR_RATIO  0x20
    write(cmd,CMD_SET_RESISTOR_RATIO | 0x6); // set lcd operating voltage (regulator resistor, ref voltage resistor)

    #define CMD_DISPLAY_ON    0xAF
    write(cmd,CMD_DISPLAY_ON);
    #define CMD_SET_ALLPTS_NORMAL 0xA4
    write(cmd,CMD_SET_ALLPTS_NORMAL);

    //st7565_set_brightness(contrast); 
    #define CMD_SET_VOLUME_FIRST  0x81
    write(cmd,CMD_SET_VOLUME_FIRST);
    #define  CMD_SET_VOLUME_SECOND  0
    const uint8_t contrast = 10; //15;
    write(cmd,CMD_SET_VOLUME_SECOND | (contrast & 0x3f));   

}

template<class T> void Lcd7565_T<T>::gotoXY( uint8_t x, uint8_t y){
 
  write(cmd,0xB0 |  ( y & 0b00000111)     ); // CMD_SET_PAGE         0xB0
  write(cmd,0x10 | (( x & 0b01111111)>>4 )); // CMD_SET_COLUMN_UPPER 0x10
  write(cmd,0x00 | (  x & 0b00001111     )); // CMD_SET_COLUMN_LOWER 0x00
 
}

template<class T> void Lcd7565_T<T>::clear(){

	for(uint8_t row=0; row<8; ++row){
    gotoXY(0,row);
		for(uint8_t j=0; j<128; ++j){ write(data,0x00); }
	}
  gotoXY(0,0);

}

template<class T> void Lcd7565_T<T>::character(uint8_t ch, bool inverted){
  
  // we use only one spacer here - after the character
  for (uint8_t i = 0; i < 5; ++i){
    uint8_t pixels = ASCII_5x8[ch - 0x20][i];
    // uint8_t pixels = pgm_read_byte_near( &ASCII_5x8[ch - 0x20][i] );
    write(data, inverted ?~pixels:pixels );
  }
  write(data,inverted?255:0); // spacer(vertical line)

}

template<class T> void Lcd7565_T<T>::stretchedCharacter(uint8_t ch, uint8_t subRow, bool inverted){
  for (uint8_t i = 0; i < 5; ++i){
    uint8_t pixels = ASCII_5x8[ch - 0x20][i];
    uint8_t stretched_pixels = stretchVerticalBar(pixels,subRow);

    write(data, inverted ?~stretched_pixels:stretched_pixels );
    write(data, inverted ?~stretched_pixels:stretched_pixels );
  }
  write(data,inverted?255:0); // spacer(vertical line)
  write(data,inverted?255:0); // spacer(vertical line)
}

template<class T> void Lcd7565_T<T>::string(const char* ch, bool inverted){
  uint8_t const spacer = ' '; 
  bool alignmentIsNecessary = (strlen(ch)==12); // magic number
  
  if (alignmentIsNecessary){ character(spacer, inverted ); character(spacer, inverted );};
  while (*ch){character(*ch++, inverted );}
  if (alignmentIsNecessary){ character(spacer, inverted ); character(spacer, inverted );};
  
}

template<class T> uint8_t Lcd7565_T<T>::stretchVerticalBar(const uint8_t v, const uint8_t subRow){

    // Warning: Absolutely Unmaintainable Code (see unrolled.txt)
    uint8_t outcome = 0;
    for (uint8_t i=0;i<8;++i){
      if( (v & (1 << ( (subRow==0?3:7) - (i>>1)))) ){
        outcome |= 1<<(7-i);
      }
    }  
    return(outcome);
}

template<class T> void Lcd7565_T<T>::threeStretchedDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace){
  // 16x24 -> 32x48
  const uint8_t digit_capacity = 3;
  uint8_t components[digit_capacity] = {0}; // array of digits : 123->[1,2,3]

  for (uint8_t i=0; i<digit_capacity; ++i){ // fill up the array
    uint16_t power10 = 1;
    for (uint8_t j=0; j<digit_capacity-i-1; ++j){ power10*=10; }
    components[i] = (value / power10)%10;
    value %= power10;
  }

  for(uint8_t row = 0; row < 3; ++row){ // original 16x24 font has 3 rows (3x8bit=24)
    for(uint8_t subRow=0;subRow<2;++subRow){ // every char will be stretched
      gotoXY(0,row*2 + subRow +_row);
      for(uint8_t digit = 0; digit < digit_capacity; ++digit){
        for(uint8_t col = 0; col < 16; ++col){ // original 16x24 font has 3x8bit rows
          uint8_t pixels = digits_16x24[components[digit]][row][col];
          uint8_t s_pixels = stretchVerticalBar(pixels,subRow);
          
          write(data, (digit != invertedDecimalPlace) ? s_pixels : ~s_pixels );
          write(data, (digit != invertedDecimalPlace) ? s_pixels : ~s_pixels );
        } // for every col [0..16]
        } // for every input digit, e.g. {5,4,2}
      } // for every subRow
    } // for every row, i.e. 3
}

template<class T> void Lcd7565_T<T>::sixBigDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace){

  const uint8_t digit_capacity = 6;
  uint8_t components[digit_capacity] = {0};
  
  for (uint8_t i=0; i<digit_capacity; ++i){ // fill up the array
    uint32_t power10 = 1;
    for (uint8_t j=0; j<digit_capacity-i-1; ++j){ power10*=10; }
    components[i] = value / power10;
    value %= power10;
  }
  
  for(uint8_t row = 0; row < 3; ++row){ // 16x24 font has 3 rows
    gotoXY(0,row+_row);
    for(uint8_t digit = 0; digit < digit_capacity; ++digit){
      for(uint8_t col = 0; col < 16; ++col){
        uint8_t pixels = digits_16x24[components[digit]][row][col];
        write(data, (digit != invertedDecimalPlace) ? pixels : ~pixels );
      } // for every col [0..16] 16x24 font has 16 columns
      } // for every input digit, e.g. {0,0,5,4,2,9}
    } // for every row, i.e. 3
}

template<class T> void Lcd7565_T<T>::stretchedString(uint8_t x, uint8_t y, const char* ch, bool inverted){

  gotoXY(x,y);
  const char* ch_ = ch;
  while (*ch_){ stretchedCharacter(*ch_++ ,0, inverted);}
  gotoXY(x,y+1);
  ch_ = ch;
  while (*ch_){ stretchedCharacter(*ch_++ ,1, inverted);}

}

#endif           