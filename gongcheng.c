#include "./algorithm/crc.h"
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_referee/referee.h"
#include "./base_drv/drv_uart.h"
#include "./base_drv/graphic/drv_graphic.h"
#include "./base_drv/graphic/graphic_def.h"
#include "stm32f4xx_hal.h"

// #include "./robot_core/weapon/vision.h"
// #include "./algorithm/imu_fusion.h"
#include <math.h>
#include <string.h>

#include "./algorithm/util.h"
#include "./robot_core/ctrl_core/chassis.h"
#include "./robot_core/ctrl_core/chassis_motor_ctrl.h"
#include "./robot_core/ctrl_core/gimbal_motor_ctrl.h"
#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/weapon/shooter.h"
#include "./robot_core/weapon/vision.h"
#define WIDTH 30
#define BAR 16

extern UART_HandleTypeDef huart6;
#define UI_HUART huart6

// ext_client_custom_graphic_seven_t test_data;
interaction_figure_2_t spin_fric_dot;
interaction_figure_3_t dynamic_layer_data_extra;

uint8_t power1, power2, power3, power4; // 吸盘开关
float uplift, extend;                   // 抬升和伸出百分比

int uplift_len_ui = 180; // ui长度
int extend_len_ui = 300;

uint16_t UI_count;

#if CLEAN_FLAG == 1
extern float max_buffer_c, max_chassis_power;
extern power_parameter_t power_parameter;
#else
float max_buffer_c, max_chassis_power;
struct power_parameter_t {
  float buffer_c;
} power_parameter;
#endif

char cover_text[30] = "COVER:";
char spin_text[30] = "SPIN:";
char tank_text[30] = "TANK:";
char state_text[30] = "CVR X\nNOBUF\nSMALLBUF\nBIGBUF";

ext_client_custom_character_t test_data;
extern interaction_figure_4_t dynamic_layer_data1;
extern interaction_figure_4_t dynamic_layer_data2;
extern interaction_figure_4_t dynamic_layer_data3;

void init_UI(void) {
  char_data_init(); // 勿动
  //  float_data_init();//勿动
  //  int_data_init();  //勿动
  /*
          八个参数：1.序号 2、种类 3、颜色 4-7x和y 8、线宽
          如果类型是圆，请在details_d变量写入半径
          可以画24个图案，0-6刷新率高，7-20刷新率并不算高
  */

  Static_UI(0, GRAPHIC_LINE, COLOR_CYAN, 125, 320, 125, 500, 12); // 抬升

  Static_UI(1, GRAPHIC_CIRCLE, COLOR_GREEN, 1600, 700, 30, 0, 8); // POWER1
  Static_UI(2, GRAPHIC_CIRCLE, COLOR_GREEN, 1600, 600, 30, 0, 8); // power2
  Static_UI(3, GRAPHIC_CIRCLE, COLOR_GREEN, 1600, 500, 30, 0, 8); // power3
  Static_UI(5, GRAPHIC_CIRCLE, COLOR_GREEN, 1600, 400, 30, 0,
            8); // power4 // POWER2
  Static_UI(4, GRAPHIC_LINE, COLOR_PURPLE, 200, 325, 500, 325, 12); // 伸出

  Static_UI(7, GRAPHIC_SQUARE, COLOR_WHITE, 100, 500, 150, 320, 2); // 抬升框
  Static_UI(8, GRAPHIC_SQUARE, COLOR_WHITE, 200, 350, 500, 300, 2); // 伸出框
}
float awaw = 0;

void update_UI(void) { // 静态图层的设置

  UI_count++;

  if (UI_count % 3 == 0) // 动态图形，注意：请把动态图形放入0-6的序号中
  {

    // BEGIN DYNAMIC
    // --------------------------------------------------------------------------
    /*
                    modify(6,test_status,COLOR_CYAN,0,0);
                    1.序号 2.出现 3.颜色 4.x 5.y
    */

    if (power1 == 1) {
      modify(1, 1, COLOR_CYAN, 0, 0);
    } else {
      modify(1, 0, COLOR_CYAN, 0, 0);
    }
    if (power2 == 1) {
      modify(2, 1, COLOR_CYAN, 0, 0);
    } else {
      modify(2, 0, COLOR_CYAN, 0, 0);
    }
    if (power3 == 1) {
      modify(3, 1, COLOR_CYAN, 0, 0);
    } else {
      modify(3, 0, COLOR_CYAN, 0, 0);
    }
    if (power4 == 1) {
      modify(5, 1, COLOR_CYAN, 0, 0);
    } else {
      modify(5, 0, COLOR_CYAN, 0, 0);
    }

    modify(0, 2, COLOR_CYAN, 125, (uplift)*uplift_len_ui);

    modify(4, 2, COLOR_PURPLE, (extend)*extend_len_ui, 325);

    send_7(dynamic_layer_data1); // 勿动

    // END DYNAMIC
    // --------------------------------------------------------------------------
  }
  if (UI_count % 16 == 0) {

    // BEGIN BACKGROUND  -------------------------------
    //		char* test_char = "TESTING";

    //		send_char(1,980,540,COLOR_WHITE,30,2,test_char);

    // END BACKGROUND    -------------------------------
    send_7(dynamic_layer_data2); // 勿动
    send_7(dynamic_layer_data3); // 勿动
  }

  if (UI_count == 33) { // 所有要变化（MOD）东西的ADD来一遍 防止被刷掉了
    UI_count = 1;
    modify(0, 2, COLOR_CYAN, 125, (uplift)*uplift_len_ui);

    modify(4, 2, COLOR_PURPLE, (extend)*extend_len_ui, 325);
    send_7(dynamic_layer_data1); // 勿动
  }
}
