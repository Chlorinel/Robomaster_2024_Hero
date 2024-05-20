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
#define FRONT_HEAT 0
#define RIGHT_HEAT 1
#define BACK_HEAT 2
#define LEFT_HEAT 3
// ext_client_custom_graphic_seven_t test_data;
interaction_figure_2_t spin_fric_dot;
interaction_figure_3_t dynamic_layer_data_extra;

int8_t use_buffer = 1;
uint8_t cover_state;
uint8_t battle_45degreee;
int8_t spin_direction;
extern cap_data_t cap_data;

int cap_len_ui = 630;
int HP_len_ui = 284;

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

// char cover_text[30] = "COVER:";
// char spin_text[30] = "SPIN:";
// char tank_text[30] = "TANK:";
// char state_text[30] = "CVR X\nNOBUF\nSMALLBUF\nBIGBUF";

// ext_client_custom_character_t test_data;
extern interaction_figure_4_t dynamic_layer_data1;
extern interaction_figure_4_t dynamic_layer_data2;
extern interaction_figure_4_t dynamic_layer_data3;

extern ext_game_robot_HP_t game_robot_HP;
extern ext_bullet_remaining_t bullet_remaining;
uint16_t robot_HP_data_RED[7];
uint16_t robot_HP_data_BLUE[7];
uint16_t RED_outpost;
uint16_t BLUE_outpost;
uint16_t robot_HP_max_RED[7];
uint16_t robot_HP_max_BLUE[7];
uint16_t show_robot_HP;
uint16_t show_robot_HP_max;
uint16_t fov_point[4] = {712, 750, 1230, 287};

void input_HP(void);
void test_char(void);

void init_UI(void) {
  char_data_init(); // 勿动
  //  float_data_init();//勿动
  //  int_data_init();  //勿动
  /*
          八个参数：1.序号 2、种类 3、颜色 4-7x和y 8、线宽
          如果类型是圆，请在details_d变量写入半径
          可以画24个图案，0-6刷新率高，7-20刷新率并不算高
  */

  Static_UI(0, GRAPHIC_LINE, COLOR_CYAN, 645, 200, 1270, 200, 12); // 电容条

  Static_UI(1, GRAPHIC_LINE, COLOR_MAIN_RB, 1600, 550, 1600, 620,
            5); // 底盘指示
  Static_UI(2, GRAPHIC_ELLIPSE, COLOR_GREEN, 1600, 800, 20, 20,
            4); // 控制方式ui   仅在配置模式出现  //fuck 模式开启
  Static_UI(3, GRAPHIC_ELLIPSE, COLOR_YELLOW, 1600, 700, 20, 20,
            4); // 中心火控控制   仅在配置模式出现  //当前预瞄准目标模式
  Static_UI(4, GRAPHIC_SQUARE, COLOR_ORANGE, fov_point[0], fov_point[1],
            fov_point[2], fov_point[3], 2);
  Static_UI(5, GRAPHIC_CIRCLE, COLOR_PINK, 1600, 400, 20, 20,
            4); // 当前自瞄颜色  出现为蓝色 仅在配置模式出现
  Static_UI(6, GRAPHIC_LINE, COLOR_MAIN_RB, 900, 490, 1020, 590,
            8); // 摩擦轮指示
  Static_UI(7, GRAPHIC_SQUARE, COLOR_WHITE, 640, 190, 1280, 210, 2); // 电容框
  Static_UI(8, GRAPHIC_LINE, COLOR_MAIN_RB, 930, 640, 970, 640,
            2); // 6m前哨站顶部
  Static_UI(9, GRAPHIC_LINE, COLOR_MAIN_RB, 920, 443, 980, 443,
            2); // 未定横线
  Static_UI(12, GRAPHIC_CIRCLE, COLOR_YELLOW, 1600, 550, 70, 0,
            2); // 底盘指示圆

  // Static_UI(10,GRAPHIC_LINE,COLOR_CYAN, 960,110,720,61,2);//斜线2
  // 铅垂线
  Static_UI(11, GRAPHIC_LINE, COLOR_CYAN, 950, 110, 950, 900, 2);
  // 瞄准刻度线
  //		Static_UI(12,GRAPHIC_LINE,COLOR_CYAN,860,300,1060,300,2);
  //		Static_UI(13,GRAPHIC_LINE,COLOR_CYAN,885,350,1035,350,2);
  //		Static_UI(14,GRAPHIC_LINE,COLOR_CYAN,910,400,1010,400,2);
  // 中心框
  //		Static_UI(15,GRAPHIC_LINE,COLOR_GREEN,600,640,650,690,2);
  Static_UI(16, GRAPHIC_LINE, COLOR_GREEN, 1552, 620, 1648, 620, 4); // 车头方向
  Static_UI(17, GRAPHIC_LINE, COLOR_GREEN, 580, 0, 730, 450, 4);
  Static_UI(18, GRAPHIC_LINE, COLOR_GREEN, 1280, 0, 1150, 450, 4);

  // 视场角

  // test_char();
}

void update_UI(void) { // 静态图层的设置

  UI_count++;

  float ang_del = -chassis_motors._all_chassis_motors[4].real.abs_angle;

  float sin_yaw = sinf(ang_del); // 角度差转换为弧度
  float cos_yaw = cosf(ang_del);

  if (UI_count % 3 == 0) // 动态图形，注意：请把动态图形放入0-6的序号中
  {
    // input_HP();

    modify(1, 2, COLOR_MAIN_RB, (uint16_t)(1600 - 70 * sin_yaw),
           (uint16_t)(550 + 70 * cos_yaw));
    // BEGIN DYNAMIC
    // --------------------------------------------------------------------------
    /*
                    modify(6,test_status,COLOR_CYAN,0,0);
                    1.序号 2.出现 3.颜色 4.x 5.y
    */
    // send_char(3, 2, 1100, 650, COLOR_GREEN, 25, 2, spin_text); // 小陀螺指示
    if (robot.robot_flag.vt_config_flag) {
      if (robot.ctrl_mode == 0) {
        modify(2, 1, COLOR_CYAN, 0, 0);
      } else {
        modify(2, 0, COLOR_CYAN, 0, 0);
      }
      if (robot.is_center_fire == 0) {
        modify(3, 1, COLOR_CYAN, 0, 0);
      } else {
        modify(3, 0, COLOR_CYAN, 0, 0);
      }
      if (vision_request.local_color) {
        modify(5, 1, COLOR_CYAN, 0, 0);
      } else {
        modify(5, 0, COLOR_CYAN, 0, 0);
      }
    } else {
      if (robot.robot_flag.gimbal_fuck_mode_flag == 1) {
        modify(2, 1, COLOR_CYAN, 50, 0);
      } else {
        modify(2, 0, COLOR_CYAN, 50, 0);
      }
      if (attack_target_type == 0 || attack_target_type == 4) {
        modify(3, 1, COLOR_YELLOW, 50, 0);
      } else {
        modify(3, 0, COLOR_YELLOW, 50, 0);
      }

      if (robot_hurt.armor_id == FRONT_HEAT) {
        modify(5, 1, COLOR_YELLOW, 0, 220);
      } else if (robot_hurt.armor_id == RIGHT_HEAT) {
      } else if (robot_hurt.armor_id == BACK_HEAT) {
      } else if (robot_hurt.armor_id == LEFT_HEAT) {
      } else {
      }
    }
    if (vision_ctrl_data.target_found) {
      modify(4, 0, COLOR_MAIN_RB, fov_point[2], fov_point[3]);
    } else {
      modify(4, 1, COLOR_MAIN_RB, fov_point[2], fov_point[3]);
    }

    if (shooter_debug.is_ready_to_fire) {
      modify(6, 0, COLOR_MAIN_RB, 0, 0);
    } else {
      modify(6, 1, COLOR_MAIN_RB, 0, 0);
    }

    if (use_buffer == 1)
      modify(0, 2, COLOR_CYAN,
             645 + (cap_data.cap_voltage / VCAP_MAX) * cap_len_ui, 200);
    else
      modify(0, 2, COLOR_WHITE,
             645 + (cap_data.cap_voltage / VCAP_MAX) * cap_len_ui, 200);

    send_7(dynamic_layer_data1); // 勿动
    //		modify(4,2,COLOR_PURPLE,1000+BAR/2+HP_len_ui,875-WIDTH-BAR/2);
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
    modify(1, 1, COLOR_MAIN_RB, (uint16_t)(1600 - 70 * sin_yaw),
           (uint16_t)(550 + 70 * cos_yaw));
    modify(0, 1, COLOR_YELLOW,
           645 + (cap_data.cap_voltage / VCAP_MAX) * cap_len_ui, 250);
    // modify(4, 1, COLOR_PURPLE, (uint16_t)CLAMP0((755 + show_robot_HP), 1920),
    //        837);
    send_7(dynamic_layer_data1); // 勿动
  }
}

void input_HP(void) {
  uint8_t i;
  memcpy(robot_HP_data_RED, (void *)&(game_robot_HP.red_1_robot_HP),
         sizeof(uint16_t) * 7);
  memcpy(robot_HP_data_BLUE, (void *)(&(game_robot_HP.red_1_robot_HP) + 8),
         sizeof(uint16_t) * 7);
  RED_outpost = game_robot_HP.red_outpost_HP;
  BLUE_outpost = game_robot_HP.blue_outpost_HP;

  // 用一直更新的方法知道大概的血量上限
  for (i = 0; i < 7; i++) {
    robot_HP_max_RED[i] = (robot_HP_max_RED[i] < robot_HP_data_RED[i])
                              ? robot_HP_data_RED[i]
                              : robot_HP_max_RED[i];
    robot_HP_max_BLUE[i] = (robot_HP_max_BLUE[i] < robot_HP_data_BLUE[i])
                               ? robot_HP_data_BLUE[i]
                               : robot_HP_max_BLUE[i];
  }

  if (vision_ctrl_data.target_found) {
    if (vision_ctrl_data.id_num == 1 || vision_ctrl_data.id_num == 3 ||
        vision_ctrl_data.id_num == 4 || vision_ctrl_data.id_num == 5) {
      i = vision_ctrl_data.id_num;

      if (game_robot_status.robot_id > 100) { // 判断己方颜色 大于一百为蓝
        show_robot_HP = robot_HP_data_RED[i - 1];
        show_robot_HP_max = robot_HP_max_RED[i - 1];
      } else {
        show_robot_HP = robot_HP_data_BLUE[i - 1];
        show_robot_HP_max = robot_HP_max_BLUE[i - 1];
      }
    } else if (vision_ctrl_data.id_num == 6) {
      if (game_robot_status.robot_id > 100)
        show_robot_HP = robot_HP_data_RED[5];
      else
        show_robot_HP = robot_HP_data_BLUE[5]; // 哨兵
      show_robot_HP_max = 1000;
    } else if (vision_ctrl_data.id_num == 0) {
      if (game_robot_status.robot_id > 100)
        show_robot_HP = RED_outpost;
      else
        show_robot_HP = BLUE_outpost;
      show_robot_HP_max = 1500; // outpost max HP
    } else {
      show_robot_HP = 0;
      show_robot_HP_max = 100;
    }
  } else {
    show_robot_HP = 0;
    show_robot_HP_max = 100;
  }
}
