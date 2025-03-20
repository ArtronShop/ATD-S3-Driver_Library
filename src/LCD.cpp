#include <Arduino.h>
#include "LCD.h"
#include "PCA9557.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

static const char * TAG = "LCD";

const esp_lcd_rgb_timing_t lcd_timing[LCD_SIZE_MAX] = {
	{ // 800x480
	  .pclk_hz = 33 * 1000 * 1000,
    .h_res = 800,
    .v_res = 480,
    .hsync_pulse_width = 30,
    .hsync_back_porch = 46,
    .hsync_front_porch = 210,
    .vsync_pulse_width = 13,
    .vsync_back_porch = 23,
    .vsync_front_porch = 22,
    .flags = {
      .pclk_active_neg = 1,
    },
	},
  { // 480x272
    .pclk_hz = 9 * 1000 * 1000,
    .h_res = 480,
    .v_res = 272,
    .hsync_pulse_width = 4,
    .hsync_back_porch = 43,
    .hsync_front_porch = 8,
    .vsync_pulse_width = 4,
    .vsync_back_porch = 12,
    .vsync_front_porch = 8,
    .flags = {
      .pclk_active_neg = 1,
    },
  }
};

static esp_lcd_panel_handle_t panel_handle = NULL;
#ifdef USE_LVGL
static lv_disp_drv_t disp_drv;
#endif

static bool notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel, esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx) ;

LCD::LCD() {

}

void LCD::initRGBInterface(const esp_lcd_rgb_timing_t * timing) {
  esp_lcd_rgb_panel_config_t panel_config = {
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .timings = {},
    .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
    .bits_per_pixel = 0,
    .num_fbs = 1,
    .bounce_buffer_size_px = 40 * timing->h_res,
    .sram_trans_align = 8,
    .psram_trans_align = 64,
    .hsync_gpio_num = TFT_HSYNC,
    .vsync_gpio_num = TFT_VSYNC,
    .de_gpio_num = TFT_DE,
    .pclk_gpio_num = TFT_PCLK,
    .disp_gpio_num = -1,
    .data_gpio_nums = {
      TFT_B0, TFT_B1, TFT_B2, TFT_B3, TFT_B4,
      TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
      TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4,
    },
    .flags = {
      .disp_active_low = 0,
      .refresh_on_demand = 0,
      .fb_in_psram = 1,
      .double_fb = 0,
      .no_fb = 0,
      .bb_invalidate_cache = 0
    }
  };
  memcpy(&panel_config.timings, timing, sizeof(esp_lcd_rgb_timing_t));
  esp_lcd_new_rgb_panel(&panel_config, &panel_handle);

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
}

void LCD::begin(LCD_Size_t size, int rotation) {
  this->rotation = rotation;
  this->brightness = constrain(brightness, 0, 1);
  const esp_lcd_rgb_timing_t * timing = &lcd_timing[size];
  this->lcd_width = timing->h_res;
  this->lcd_height = timing->v_res;

  Wire.begin(SDA_PIN, SCL_PIN, (uint32_t) 400E3);

  io_ext.pinMode(TFT_DISP_EXT, OUTPUT);
  io_ext.pinMode(TFT_BL_EXT, OUTPUT);

  this->initRGBInterface(timing);
  this->setRotation(rotation);
  this->on();
}

int LCD::getWidth() {
  return this->lcd_width;
}

int LCD::getHeight() {
  return this->lcd_height;
}

void LCD::setRotation(int m) {
  switch(m) {
    case 1:
      esp_lcd_panel_mirror(panel_handle, false, false);
      esp_lcd_panel_swap_xy(panel_handle, false);
      break;
    case 2:
      esp_lcd_panel_mirror(panel_handle, true, false);
      esp_lcd_panel_swap_xy(panel_handle, true);
      break;
    case 3:
      esp_lcd_panel_mirror(panel_handle, true, true);
      esp_lcd_panel_swap_xy(panel_handle, false);
      break;
    case 4:
      esp_lcd_panel_mirror(panel_handle, false, true);
      esp_lcd_panel_swap_xy(panel_handle, true);
      break;
  }
  this->rotation = m;
}

uint8_t LCD::getRotation() {
  return this->rotation;
}

// Display OFF
void LCD::off() {
  io_ext.digitalWrite(TFT_DISP_EXT, LOW);
  io_ext.digitalWrite(TFT_BL_EXT, LOW);
}

// Display ON
void LCD::on() {
  io_ext.digitalWrite(TFT_DISP_EXT, HIGH);
  io_ext.digitalWrite(TFT_BL_EXT, HIGH);
}

void LCD::setWindow(int x_start, int y_start, int x_end, int y_end) {
  // not support
}

void LCD::drawBitmap(int x_start, int y_start, int x_end, int y_end, uint16_t* color_data) {
  esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

/*
void LCD::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  // not support
}

uint16_t LCD::color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0b11111000) << 8) | ((green & 0b11111100) << 3) | (blue >> 3);
}

uint32_t LCD::color24to16(uint32_t color888) {
  return this->color565((color888 >> 16) & 0xFF, (color888 >> 8) & 0xFF, color888 & 0xFF);
}

void LCD::fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	if (x1 >= this->lcd_width) return;
	if (x2 >= this->lcd_width) x2 = this->lcd_width - 1;
	if (y1 >= this->lcd_height) return;
	if (y2 >= this->lcd_height) y2 = this->lcd_height - 1;

}

// Fill screen
// color:color
void LCD::fillScreen(uint16_t color) {
	this->fillRect(0, 0, this->lcd_width - 1, this->lcd_height - 1, color);
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
void LCD::drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int i;
	int dx,dy;
	int sx,sy;
	int E;

	// distance between two points
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	// direction of two point
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	// inclination < 1
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			this->drawPixel(x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
			y1 += sy;
			E -= 2 * dx;
		}
	}

	// inclination >= 1
	} else {
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			this->drawPixel(x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
void LCD::drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	this->drawLine(x1, y1, x2, y1, color);
	this->drawLine(x2, y1, x2, y2, color);
	this->drawLine(x2, y2, x1, y2, color);
	this->drawLine(x1, y2, x1, y1, color);
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle:Angle of rectangle
// color:color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void LCD::drawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	int x4,y4;
	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w/2;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = h/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	this->drawLine(x1, y1, x2, y2, color);
	this->drawLine(x1, y1, x3, y3, color);
	this->drawLine(x2, y2, x4, y4, color);
	this->drawLine(x3, y3, x4, y4, color);
}

// Draw triangle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of triangle
// h:Height of triangle
// angle:Angle of triangle
// color:color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void LCD::drawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color) {
	double xd,yd,rd;
	int x1,y1;
	int x2,y2;
	int x3,y3;
	rd = -angle * M_PI / 180.0;
	xd = 0.0;
	yd = h/2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w/2;
	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = 0.0 - w/2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	this->drawLine(x1, y1, x2, y2, color);
	this->drawLine(x1, y1, x3, y3, color);
	this->drawLine(x2, y2, x3, y3, color);
}

// Draw circle
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void LCD::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;

	x=0;
	y=-r;
	err=2-2*r;
	do{
		this->drawPixel(x0-x, y0+y, color);
		this->drawPixel(x0-y, y0-x, color);
		this->drawPixel(x0+x, y0-y, color);
		this->drawPixel(x0+y, y0+x, color);
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;
	} while(y<0);
}

// Draw circle of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void LCD::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	int ChangeX;

	x=0;
	y=-r;
	err=2-2*r;
	ChangeX=1;
	do{
		if(ChangeX) {
			this->drawLine(x0-x, y0-y, x0-x, y0+y, color);
			this->drawLine(x0+x, y0-y, x0+x, y0+y, color);
		} // endif
		ChangeX=(old_err=err)<=x;
		if (ChangeX)			err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;
	} while(y<=0);
}

// Draw rectangle with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// r:radius
// color:color
void LCD::drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color) {
	int x;
	int y;
	int err;
	int old_err;
	unsigned char temp;

	if(x1>x2) {
		temp=x1; x1=x2; x2=temp;
	} // endif

	if(y1>y2) {
		temp=y1; y1=y2; y2=temp;
	} // endif

	if (x2-x1 < r) return; // Add 20190517
	if (y2-y1 < r) return; // Add 20190517

	x=0;
	y=-r;
	err=2-2*r;

	do{
		if(x) {
			this->drawPixel(x1+r-x, y1+r+y, color);
			this->drawPixel(x2-r+x, y1+r+y, color);
			this->drawPixel(x1+r-x, y2-r-y, color);
			this->drawPixel(x2-r+x, y2-r-y, color);
		} // endif
		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;
	} while(y<0);

	this->drawLine(x1+r,y1  ,x2-r,y1	,color);
	this->drawLine(x1+r,y2  ,x2-r,y2	,color);
	this->drawLine(x1  ,y1+r,x1  ,y2-r,color);
	this->drawLine(x2  ,y1+r,x2  ,y2-r,color);
}

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void LCD::drawArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//	 printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	//lcdDrawLine(x0,y0,x1,y1,color);
	this->drawLine(x1, y1, L[0], L[1], color);
	this->drawLine(x1, y1, R[0], R[1], color);
	this->drawLine(L[0], L[1], R[0], R[1], color);
}


// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
void LCD::fillArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color) {
	double Vx= x1 - x0;
	double Vy= y1 - y0;
	double v = sqrt(Vx*Vx+Vy*Vy);
	//printf("v=%f\n",v);
	double Ux= Vx/v;
	double Uy= Vy/v;

	uint16_t L[2],R[2];
	L[0]= x1 - Uy*w - Ux*v;
	L[1]= y1 + Ux*w - Uy*v;
	R[0]= x1 + Uy*w - Ux*v;
	R[1]= y1 - Ux*w - Uy*v;
	//printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	this->drawLine(x0, y0, x1, y1, color);
	this->drawLine(x1, y1, L[0], L[1], color);
	this->drawLine(x1, y1, R[0], R[1], color);
	this->drawLine(L[0], L[1], R[0], R[1], color);

	int ww;
	for(ww=w-1;ww>0;ww--) {
		L[0]= x1 - Uy*ww - Ux*v;
		L[1]= y1 + Ux*ww - Uy*v;
		R[0]= x1 + Uy*ww - Ux*v;
		R[1]= y1 - Ux*ww - Uy*v;
		//printf("Fill>L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);
		this->drawLine(x1, y1, L[0], L[1], color);
		this->drawLine(x1, y1, R[0], R[1], color);
	}
}
*/

#ifdef USE_LVGL
#include "LVGLHelper.h"

static lv_disp_draw_buf_t draw_buf;
static lv_color_t * disp_draw_buf;

static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
  lv_disp_flush_ready(disp);
}

unsigned long last_touch_on_display  = 0;

void display_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
  if((event == LV_EVENT_CLICKED) || (event == LV_EVENT_KEY)) {
    last_touch_on_display = millis();
  }
}

void LCD::useLVGL() {
  lv_init();
  
  uint32_t buffer_size_in_px_cnt = this->lcd_width * this->lcd_height / 10;
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * buffer_size_in_px_cnt, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, buffer_size_in_px_cnt);

  /*Initialize the display*/
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = this->lcd_width;
  disp_drv.ver_res = this->lcd_height;
  disp_drv.flush_cb = disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.user_data = NULL;
  lv_disp_drv_register(&disp_drv);
}

void LCD::loop() {
  { // UI update
    static unsigned long timer = 0;
    if ((millis() < timer) || (timer == 0) || ((millis() - timer) >= 5)) {
      timer = millis();
      lv_timer_handler();
    }
  }

  { // Auto sleep
    static uint8_t state = 0;
    if (this->auto_sleep_after_sec > 0) { // enable auto sleep
      if (state == 0) {
        if ((millis() - last_touch_on_display) >= (this->auto_sleep_after_sec * 1000)) {
          this->off();
          state = 1;
        }
      } else if (state == 1) {
        if ((millis() - last_touch_on_display) < (this->auto_sleep_after_sec * 1000)) {
          lv_obj_invalidate(lv_scr_act());
          lv_timer_handler();
          this->on();
          state = 0;
        }
      }
    } else { // disable auto sleep
      if (state != 0) { // but now in sleep
        lv_obj_invalidate(lv_scr_act());
        lv_timer_handler();
        this->on();
        state = 0;
      }
    }
  }

  if (xSafeUpdateQueue) { // Safe UI update
    SafeUpdateParam_t safe_update_item;
    while(xQueueReceive(xSafeUpdateQueue, &safe_update_item, 0) == pdPASS) {
      if (safe_update_item.cb) {
        safe_update_item.cb(safe_update_item.user_data);
      }
      if (safe_update_item.sync_event_group_handle) {
        xEventGroupSetBits(safe_update_item.sync_event_group_handle, BIT0);
      }
    }
  }
}
#endif

void LCD::enableAutoSleep(uint32_t timeout_in_sec) {
  this->auto_sleep_after_sec = timeout_in_sec;
}

void LCD::disableAutoSleep() {
  this->auto_sleep_after_sec = 0;
}

LCD Display;
