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
#define SELF_RED 0
#define SELF_BLUE 1
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
#define SHOOTER_DELAY 0.1
#define ADD_ANGLE 60
uint16_t max_spin_speed_len_ui = 600;
uint16_t max_move_speed_len_ui = 600;

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
uint16_t RED_base;
uint16_t BLUE_outpost;
uint16_t BLUE_base;
uint16_t robot_HP_max_RED[7];
uint16_t robot_HP_max_BLUE[7];
uint16_t show_robot_HP;
uint16_t show_robot_HP_max;
uint16_t fov_point[4] = {712, 750, 1230, 287};
uint8_t show_flag;
uint8_t show_count;

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
  Static_UI(2, GRAPHIC_LINE, COLOR_GREEN, 1620, 200, 1620, 800, 12); // 移动速度
  Static_UI(3, GRAPHIC_LINE, COLOR_GREEN, 1700, 200, 1700, 800,
            12); // 小陀螺速度
  Static_UI(4, GRAPHIC_SQUARE, COLOR_CYAN, fov_point[0], fov_point[1],
            fov_point[2], fov_point[3], 2); // 命中  相机角度
  Static_UI(5, GRAPHIC_LINE, COLOR_PINK, 645, 100, 1270, 100,
            12); // 计时器
  Static_UI(6, GRAPHIC_LINE, COLOR_MAIN_RB, 900, 490, 1020, 590,
            8); // 摩擦轮指示
  Static_UI(7, GRAPHIC_SQUARE, COLOR_WHITE, 640, 190, 1280, 210, 2); // 电容框
  Static_UI(8, GRAPHIC_SQUARE, COLOR_WHITE, 1610, 390, 1710, 610,
            2); // 移动速度框
  Static_UI(9, GRAPHIC_LINE, COLOR_MAIN_RB, 900, 640, 1000, 640,
            1); // 6m前哨站顶部
  Static_UI(10, GRAPHIC_LINE, COLOR_MAIN_RB, 850, 485, 1050, 485,
            1); // 13.7m前哨站顶部横线
  Static_UI(11, GRAPHIC_LINE, COLOR_MAIN_RB, 800, 300, 1100, 300,
            1); // 21m基地顶部绿灯
  Static_UI(12, GRAPHIC_CIRCLE, COLOR_YELLOW, 1600, 550, 70, 0,
            2); // 底盘指示圆

  Static_UI(13, GRAPHIC_LINE, COLOR_MAIN_RB, 960, 568, 1040, 568,
            1); // 9m前哨站顶部横线

  Static_UI(14, GRAPHIC_LINE, COLOR_MAIN_RB, 920, 503, 980, 503,
            1); // 8m环高基地顶部绿灯

  Static_UI(15, GRAPHIC_LINE, COLOR_WHITE, 930, 110, 930, 900, 1);
  // 8m前哨站旋转预瞄右
  Static_UI(16, GRAPHIC_LINE, COLOR_WHITE, 970, 110, 970, 900, 1);
  // 8m前哨站旋转预瞄左

  Static_UI(17, GRAPHIC_LINE, COLOR_GREEN, 580, 0, 730, 450, 4);
  Static_UI(18, GRAPHIC_LINE, COLOR_GREEN, 1280, 0, 1150, 450, 4);
  Static_UI(19, GRAPHIC_LINE, COLOR_CYAN, 950, 110, 950, 900, 1);
  // 瞄准刻度线
  Static_UI(20, GRAPHIC_SQUARE, COLOR_WHITE, 640, 90, 1280, 110,
            2); // 计时器框
                // Static_UI(20, GRAPHIC_CIRCLE, COLOR_PINK, 200, 550, 200, 20,
                //           2); // 当前攻击模式
  // 视场角

  // test_char();
}
uint32_t now_ticking_time;
uint32_t ticking_time;
void update_UI(void) { // 静态图层的设置

  UI_count++;

  float ang_del = -chassis_motors._all_chassis_motors[4].real.abs_angle;
  float attack_mode_angle;
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
    // send_char(3, 2, 1100, 650, COLOR_GREEN, 25, 2, spin_text); //
    // 小陀螺指示

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
    input_HP();
    if (show_flag) {
      modify(6, 1, COLOR_ORANGE, 0, 0);
      show_count++;
    }

    if (show_count > 15) {
      show_count = 0;
      show_flag = 0;
    }
    if (robot.robot_flag.chassis_super_cap_enable_flag == 1)
      modify(0, 2, COLOR_CYAN,
             645 + (cap_data.cap_voltage / VCAP_MAX) * cap_len_ui, 200);
    else
      modify(0, 2, COLOR_WHITE,
             645 + (cap_data.cap_voltage / VCAP_MAX) * cap_len_ui, 200);
    modify(2, 2, COLOR_CYAN, 1620,
           200 + (robot.base_speed / MAX_MOVE_SPEED) * max_move_speed_len_ui);

    modify(3, 2, COLOR_CYAN, 1700,
           200 + (robot.spin_speed / MAX_SPIN_SPEED) * max_spin_speed_len_ui);

    ticking_time = 10000 - ((HAL_GetTick()) % 10000);

    modify(5, 2, COLOR_ORANGE, 645 + (ticking_time / 10000) * cap_len_ui, 100);
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
    // modify(4, 1, COLOR_PURPLE, (uint16_t)CLAMP0((755 +
    // show_robot_HP), 1920),
    //        837);
    modify(2, 1, COLOR_CYAN, 1620,
           400 + (robot.base_speed / MAX_MOVE_SPEED) * max_move_speed_len_ui);

    modify(3, 1, COLOR_CYAN, 1700,
           400 + (robot.spin_speed / MAX_SPIN_SPEED) * max_spin_speed_len_ui);
    modify(5, 1, COLOR_ORANGE, 645 + (ticking_time / 10000) * cap_len_ui, 100);
    send_7(dynamic_layer_data1); // 勿动
  }
}

bool self_color;
uint16_t attack_aim_hp;
uint16_t last_attack_aim_hp;

void input_HP(void) {

  self_color = referee_info.robot_id < 100 ? SELF_BLUE : SELF_RED;
  RED_outpost = game_robot_HP.red_outpost_HP;
  RED_base = game_robot_HP.red_base_HP;
  BLUE_outpost = game_robot_HP.blue_outpost_HP;
  BLUE_base = game_robot_HP.blue_base_HP;
  if (self_color) {
    if (BLUE_outpost > 0) {
      attack_aim_hp = BLUE_outpost;
      if (last_attack_aim_hp - attack_aim_hp >= 200) {
        show_flag = 1;
        show_count = 0;
      }
      last_attack_aim_hp = attack_aim_hp;
    } else {
      attack_aim_hp = BLUE_base;
      if (last_attack_aim_hp - attack_aim_hp >= 200) {
        show_flag = 1;
        show_count = 0;
      }
      last_attack_aim_hp = attack_aim_hp;
    }
  } else {
    if (RED_outpost > 0) {
      attack_aim_hp = RED_outpost;
      if (last_attack_aim_hp - attack_aim_hp >= 200) {
        show_flag = 1;
        show_count = 0;
      }
      last_attack_aim_hp = attack_aim_hp;
    } else {
      attack_aim_hp = RED_base;
      if (last_attack_aim_hp - attack_aim_hp >= 200) {
        show_flag = 1;
        show_count = 0;
      }
      last_attack_aim_hp = attack_aim_hp;
    }
  }
}
