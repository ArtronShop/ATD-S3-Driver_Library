#if defined(__has_include)
#if __has_include(<lvgl.h>)
#include <lvgl.h>
#define USE_LVGL
#endif
#else
#error "__has_include not work"
#endif

#define TFT_DE 17
#define TFT_VSYNC 16
#define TFT_HSYNC 15
#define TFT_PCLK 7
#define TFT_R0 1
#define TFT_R1 2
#define TFT_R2 42
#define TFT_R3 41
#define TFT_R4 40
#define TFT_G0 39
#define TFT_G1 38
#define TFT_G2 0
#define TFT_G3 45
#define TFT_G4 48
#define TFT_G5 47
#define TFT_B0 21
#define TFT_B1 14
#define TFT_B2 46
#define TFT_B3 3
#define TFT_B4 18
#define TFT_DISP_EXT 4
#define TFT_BL_EXT 5

#define TOUCH_INT_EXT 7
#define TOUCH_RST_EXT 6

#define SDA_PIN (8)
#define SCL_PIN (9)
