#ifndef _2812_H_
#define _2812_H_

#include "main.h"
#define LED_NUM 8
#define RESET_BIT 1

typedef struct {
  uint8_t R;
  uint8_t G;
  uint8_t B;

} Color;

typedef enum {
  green = 0,
  blue,
  red,
  yellow,
  black,
  diy,
} led_color_t;

// ��������
extern Color GREEN;
extern Color BLUE;
extern Color RED;
extern Color YELLOW;
extern Color BLACK;
extern Color WHITE;
// ָʾ�ƶ�Ӧλ��
#define port_CHASSIS 0
#define port_GIMBAL 1
#define port_SHOOTER 2
#define port_VISION 3
#define port_FIRE_CTRL 4
#define port_SPINNING 5
#define port_RC_CTRL 6
#define port_COSTOMER 7

extern led_color_t set_col[LED_NUM];

void light_init(void);
void setcolor(uint8_t id, Color color);
void reset(void);
void sendcolor(void);
void SLEF_SET_COLOR(uint16_t RGB_number, uint16_t Green, uint16_t Blue,
                    uint16_t Red);
void light_send(void);
#endif
