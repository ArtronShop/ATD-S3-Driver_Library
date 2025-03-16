#include <Arduino.h>
#include <Wire.h>
#include "LCD.h"
#include "Touch.h"
#include "PCA9557.h"

static const char * TAG = "Touch";

#define GT911_ADDR 0x5D

#define TOUCH_SDA_PIN (8)
#define TOUCH_SCL_PIN (9)

GT911::GT911() { }

void GT911::begin() {
    io_ext.pinMode(TOUCH_RST_EXT, OUTPUT);
    io_ext.pinMode(TOUCH_INT_EXT, INPUT);

    io_ext.digitalWrite(TOUCH_RST_EXT, LOW);
    delay(20);
    io_ext.digitalWrite(TOUCH_RST_EXT, HIGH);
    delay(50);

    Wire.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, (uint32_t) 400E3);
}

uint8_t GT911::read(uint16_t *cx, uint16_t *cy) {
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x4E);
    if (Wire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Write error !");
        return 0;
    }

    uint8_t count;
    count = Wire.requestFrom(GT911_ADDR, 1);
    if (count != 1) {
        ESP_LOGE(TAG, "Read error !");
        return 0;
    }

    uint8_t touch_info = Wire.read();
    if ((touch_info & 0x80) == 0) { // buffer status are set
        return 0; // not ready and data is not valid
    }

    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Write error !");
        return 0;
    }

    uint8_t touch_point = touch_info & 0x0F;
    if ((touch_point <= 0) || (touch_point > 5)) {
        return 0;
    }

    // Read Coordinate
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x50);
    if (Wire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Write error !");
        return 0;
    }

    count = Wire.requestFrom(GT911_ADDR, 4);
    if (count != 4) {
        ESP_LOGE(TAG, "Read error !");
        return 0;
    }

    // Process Data
    uint8_t TOUCH1_XL = Wire.read();
    uint8_t TOUCH1_XH = Wire.read();
    uint8_t TOUCH1_YL = Wire.read();
    uint8_t TOUCH1_YH = Wire.read();
    /*
    *cx = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
    *cy = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;
    */
    
    uint16_t x = (((uint16_t)TOUCH1_XH&0x0F)<<8)|TOUCH1_XL;
    uint16_t y = (((uint16_t)TOUCH1_YH&0x0F)<<8)|TOUCH1_YL;

    uint8_t m = Display.getRotation();
    if (m == 0) {
        *cx = x;
        *cy = y;
    } else if (m == 1) {
        *cx = y;
        *cy = Display.getHeight() - x;
    } else if (m == 2) {
        *cx = Display.getWidth() - y;
        *cy = x;
    } else if (m == 3) {
        *cx = Display.getHeight() - x;
        *cy = Display.getWidth() - y;
    }

    return touch_point;
}

#ifdef USE_LVGL
static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  GT911 * touch = (GT911 *) indev_driver->user_data;

  uint8_t touchPoint = touch->read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    ESP_LOGV(TAG, "X: %d, Y: %d", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

static void touch_feedback(lv_indev_drv_t *indev_driver, uint8_t event) {
  extern void display_inp_feedback(lv_indev_drv_t *indev_driver, uint8_t event) ;
  display_inp_feedback(indev_driver, event) ;
}

lv_indev_t * lvgl_indev = NULL;

void GT911::useLVGL() {
  static lv_indev_drv_t lvgl_indev_drv;
  lv_indev_drv_init(&lvgl_indev_drv);
  lvgl_indev_drv.type = LV_INDEV_TYPE_POINTER;
  lvgl_indev_drv.read_cb = touchpad_read;
  lvgl_indev_drv.user_data = this;
  lvgl_indev_drv.feedback_cb = touch_feedback;
  lvgl_indev = lv_indev_drv_register(&lvgl_indev_drv);
}
#endif

GT911 Touch;
