#include "lcd_st7565.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/mpu_wrappers.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <string.h>

void Lcd7565::write(uint8_t dc, uint8_t data){

  gpio_set_level(lcd_rs,dc); // set data or cmd mode
  for (uint8_t i = 0; i < 8; ++i){ // shiftout data
    gpio_set_level(lcd_scl,0);
    ets_delay_us(5);
    gpio_set_level(lcd_si, data & (1 << (7 - i)) );
    ets_delay_us(5);
    gpio_set_level(lcd_scl,1);
  }

}

void Lcd7565::init(){

    gpio_config_t tmp_io_conf{
        .pin_bit_mask = ((1ULL<<lcd_rst) | (1ULL<<lcd_rs) | (1ULL<<lcd_scl) | (1ULL<<lcd_si)),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,       
    };
    gpio_config(&tmp_io_conf);
    gpio_set_level(lcd_rst,0);
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_set_level(lcd_rst,1);

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
    const uint8_t contrast = 15;
    write(cmd,CMD_SET_VOLUME_SECOND | (contrast & 0x3f));   

}

void Lcd7565::gotoXY( uint8_t x, uint8_t y){
 
  write(cmd,0xB0 |  ( y & 0b00000111)     ); // CMD_SET_PAGE         0xB0
  write(cmd,0x10 | (( x & 0b01111111)>>4 )); // CMD_SET_COLUMN_UPPER 0x10
  write(cmd,0x00 | (  x & 0b00001111     )); // CMD_SET_COLUMN_LOWER 0x00
 
}

void Lcd7565::clear(){

	for(uint8_t row=0; row<8; ++row){
    gotoXY(0,row);
		for(uint8_t j=0; j<128; ++j){ write(data,0x00); }
	}
  gotoXY(0,0);

}

void Lcd7565::character(uint8_t ch, bool inverted){
  
  // we use only one spacer here - after the character
  for (uint8_t i = 0; i < 5; ++i){
    uint8_t pixels = ASCII_5x8[ch - 0x20][i];
    // uint8_t pixels = pgm_read_byte_near( &ASCII_5x8[ch - 0x20][i] );
    write(data, inverted ?~pixels:pixels );
  }
  write(data,inverted?255:0); // spacer(vertical line)

}

void Lcd7565::stretchedCharacter(uint8_t ch, uint8_t subRow, bool inverted){
  for (uint8_t i = 0; i < 5; ++i){
    uint8_t pixels = ASCII_5x8[ch - 0x20][i];
    uint8_t stretched_pixels = stretchVerticalBar(pixels,subRow);

    write(data, inverted ?~stretched_pixels:stretched_pixels );
    write(data, inverted ?~stretched_pixels:stretched_pixels );
  }
  write(data,inverted?255:0); // spacer(vertical line)
  write(data,inverted?255:0); // spacer(vertical line)
}

void Lcd7565::string(const char* ch, bool inverted){
  uint8_t const spacer = ' '; 
  bool alignmentIsNecessary = (strlen(ch)==12); // magic number
  
  if (alignmentIsNecessary){ character(spacer, inverted ); character(spacer, inverted );};
  while (*ch){character(*ch++, inverted );}
  if (alignmentIsNecessary){ character(spacer, inverted ); character(spacer, inverted );};
  
}

uint8_t Lcd7565::stretchVerticalBar(const uint8_t v, const uint8_t subRow){

    // Warning: Absolutely Unmaintainable Code (see unrolled.txt)
    uint8_t outcome = 0;
    for (uint8_t i=0;i<8;++i){
      if( (v & (1 << ( (subRow==0?3:7) - (i>>1)))) ){
        outcome |= 1<<(7-i);
      }
    }  
    return(outcome);
}

void Lcd7565::threeStretchedDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace){
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

void Lcd7565::sixBigDigits(uint32_t value, uint8_t _row, uint8_t invertedDecimalPlace){

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

void Lcd7565::stretchedString(uint8_t x, uint8_t y, const char* ch, bool inverted){

  gotoXY(x,y);
  const char* ch_ = ch;
  while (*ch_){ stretchedCharacter(*ch_++ ,0, inverted);}
  gotoXY(x,y+1);
  ch_ = ch;
  while (*ch_){ stretchedCharacter(*ch_++ ,1, inverted);}

}
