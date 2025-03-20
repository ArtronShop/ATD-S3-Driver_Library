#pragma once

#include <stdint.h>
#include <string.h>
#include "Common.h"

#include <Arduino_GFX_Library.h>

#include "esp_lcd_panel_rgb.h"

typedef enum {
    LCD_SIZE_800x480,
    LCD_SIZE_480x272,
    LCD_SIZE_MAX
} LCD_Size_t;

typedef struct {
    uint32_t pclk_hz;
    uint16_t width;
    uint16_t height;
    uint16_t hpw;
    uint16_t hfp;
    uint16_t hbp;
    uint16_t vpw;
    uint16_t vfp;
    uint16_t vbp;
} LCD_Info_t;

class LCD {
    private:
        uint16_t lcd_width = 800;
        uint16_t lcd_height = 480;
        uint8_t rotation = 0;
        uint8_t brightness = 100;

        void initRGBInterface(const esp_lcd_rgb_timing_t * timing)  ;
        
        void setWindow(int x_start, int y_start, int x_end, int y_end) ;

        uint16_t auto_sleep_after_sec = 0;

    public:
        LCD();
        void begin(LCD_Size_t size, int rotation) ;
        int getWidth() ;
        int getHeight() ;
        void setRotation(int m) ;
        uint8_t getRotation() ;
        void off() ;
        void on() ;
        void setBrightness(int level) ;
        int getBrightness() ;

        void drawBitmap(int x_start, int y_start, int x_end, int y_end, uint16_t* color_data) ;

        /*
        uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) ;
        uint32_t color24to16(uint32_t color888) ;
        
        void drawPixel(uint16_t x, uint16_t y, uint16_t color) ;
        void fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) ;
        void fillScreen(uint16_t color) ;
        void drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) ;
        void drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) ;
        void drawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) ;
        void drawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) ;
        void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) ;
        void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) ;
        void drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color) ;
        void drawArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) ;
        void fillArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) ;
        */

#if __has_include(<lvgl.h>)
        void useLVGL() ;
        void loop() ;
#endif

        void enableAutoSleep(uint32_t timeout_in_sec) ;
        void disableAutoSleep() ;

};

extern LCD Display;
