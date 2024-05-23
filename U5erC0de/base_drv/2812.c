#include "2812.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

// 0��1���Ӧ�ıȽ�ֵ
#define CODE1 134
#define CODE0 67

// ��ɫ�������鶨��
led_color_t set_col[LED_NUM];

// ���ú�ת�����鶨��
Color led_color_buffer[LED_NUM];
#define LIGHT_DEGREE 10

// ��ɫԤ�裬˳��Ϊ{RED,GREEN,BLUE}
Color GREEN = {0, LIGHT_DEGREE, 0};
Color BLUE = {0, 0, LIGHT_DEGREE};
Color RED = {LIGHT_DEGREE, 0, 0};
Color YELLOW = {LIGHT_DEGREE, LIGHT_DEGREE, 0};
Color BLACK = {0};
Color WHITE = {4, 4, 8};
// ����ָʾ��
Color CHASSIS;
Color GIMBAL;
Color SHOOTER;
Color VISION;
Color FIRE_CTRL;
Color SPINNING;
Color RC_CTRL;
Color COSTOMER;

// ��������
uint32_t pixel_buffer[LED_NUM + RESET_BIT][24];

// ���ú���ֵ
void setcolor(uint8_t id, Color color) {
  uint8_t i;
  if (id > LED_NUM)
    return;
  for (i = 0; i < 8; i++)
    pixel_buffer[id][i] = ((color.G & (1 << (7 - i))) ? CODE1 : CODE0);
  for (i = 8; i < 16; i++)
    pixel_buffer[id][i] = ((color.R & (1 << (15 - i))) ? CODE1 : CODE0);
  for (i = 16; i < 24; i++)
    pixel_buffer[id][i] = ((color.B & (1 << (23 - i))) ? CODE1 : CODE0);
}

// �洢���ڷ���һ������ɫ��ĵȴ���
void reset() {
  uint8_t i;
  for (i = 0; i < 24; i++) {
    pixel_buffer[LED_NUM][i] = 0;
  }
}

// ���ͺ���
void send_color() {
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pixel_buffer,
                        (LED_NUM + 1) * 24);
  // HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t*)pixel_buffer[1],(pixel_num)*24);
}

// �Զ���RGB_number����ɫ
void SLEF_SET_COLOR(uint16_t RGB_number, uint16_t Green, uint16_t Blue,
                    uint16_t Red) {
  uint16_t i;
  Color buffer = {Green, Blue, Red};
  for (i = 0; i < RGB_number; i++) {
    setcolor(i, buffer);
  }
  reset();
  send_color();
}

// �û�������ɫ��ĳ�ʼ��
void light_init() {
  uint16_t i;

  for (i = 0; i < 8; i++) {
    if (set_col[i] == green) {
      led_color_buffer[i] = GREEN;
    } else if (set_col[i] == red) {
      led_color_buffer[i] = RED;
    } else if (set_col[i] == blue) {
      led_color_buffer[i] = BLUE;
    } else if (set_col[i] == black) {
      led_color_buffer[i] = BLACK;
    } else if (set_col[i] == yellow) {
      led_color_buffer[i] = YELLOW;
    } else {
      led_color_buffer[i] = WHITE;
    }
  }

  CHASSIS = led_color_buffer[0];
  GIMBAL = led_color_buffer[1];
  SHOOTER = led_color_buffer[2];
  VISION = led_color_buffer[3];
  FIRE_CTRL = led_color_buffer[4];
  SPINNING = led_color_buffer[5];
  RC_CTRL = led_color_buffer[6];
  COSTOMER = led_color_buffer[7];
}

// �ϲ㷢�ͺ���
void light_send() {
  setcolor(port_CHASSIS, CHASSIS);
  setcolor(port_GIMBAL, GIMBAL);
  setcolor(port_SHOOTER, SHOOTER);
  setcolor(port_VISION, VISION);
  setcolor(port_FIRE_CTRL, FIRE_CTRL);
  setcolor(port_SPINNING, SPINNING);
  setcolor(port_RC_CTRL, RC_CTRL);
  setcolor(port_COSTOMER, COSTOMER);
  reset();
  send_color();
}
