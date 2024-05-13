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

int8_t use_buffer = 1;
uint8_t cover_state;
uint8_t battle_45degreee;
int8_t spin_direction;
extern cap_data_t cap_data;
uint8_t target_has_found;

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
  Static_UI(2, GRAPHIC_ELLIPSE, COLOR_GREEN, 960, 110, 20, 20,
            4); // 小陀螺指示
  // Static_UI(4, GRAPHIC_LINE, COLOR_PURPLE, 755, 850, 1165, 850, 16); //
  // 血量条 Static_UI(5, GRAPHIC_SQUARE, COLOR_PINK, 0, 950, 1920, 1080,
  //    200); // 弹仓盖指示
  // send_char(5,375,550,COLOR_GREEN,25,2,"ON");//弹仓盖指示
  Static_UI(6, GRAPHIC_LINE, COLOR_MAIN_RB, 900, 490, 1020, 590,
            8); // 摩擦轮指示
  Static_UI(7, GRAPHIC_SQUARE, COLOR_WHITE, 640, 190, 1280, 210, 2); // 电容框
  // Static_UI(8, GRAPHIC_SQUARE, COLOR_WHITE, 750, 840, 1170, 860, 2); //
  // 血量框
  Static_UI(20, GRAPHIC_LINE, COLOR_WHITE, 855, 870, 855, 800, 2); // 斩杀线
  // send_char(2,250,650,COLOR_GREEN,25,2,spin_text);//小陀螺指示
  // send_char(22,250,600,COLOR_GREEN,25,2,tank_text);//坦克模式
  // send_char(23,250,550,COLOR_GREEN,25,2,cover_text);//弹仓盖
  Static_UI(12, GRAPHIC_CIRCLE, COLOR_YELLOW, 1600, 550, 70, 0,
            2); // 底盘指示圆
  Static_UI(9, GRAPHIC_CIRCLE, COLOR_YELLOW, 1600, 700, 40, 0, 8); // 遥控器控制
//  Static_UI(
//      10, GRAPHIC_CIRCLE, COLOR_ORANGE, 1600, 700, 40, 0,
//      8); // 图传控制
          // Static_UI(10,GRAPHIC_LINE,COLOR_CYAN, 960,110,720,61,2);//斜线2
  // 铅垂线
  Static_UI(11, GRAPHIC_LINE, COLOR_CYAN, 960, 110, 960, 534, 2);
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
  Static_UI(19, GRAPHIC_SQUARE, COLOR_ORANGE, fov_point[0], fov_point[1],
            fov_point[2], fov_point[3], 2);
  // test_char();
}
float awaw = 0;

void update_UI(void) { // 静态图层的设置

  UI_count++;

  float ang_del = chassis_motors._all_chassis_motors[4].real.abs_angle;

  float sin_yaw = sinf(ang_del); // 角度差转换为弧度
  float cos_yaw = cosf(ang_del);
  awaw = sin_yaw;
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
    if (robot.ctrl_mode == 0) {
      modify(2, 1, COLOR_CYAN, 0, 0);
    } else {
      modify(2, 0, COLOR_CYAN, 0, 0);
    }
    if (target_has_found == 1) {
      modify(19, 0, COLOR_MAIN_RB, fov_point[2], fov_point[3]);
    } else {
      modify(19, 1, COLOR_MAIN_RB, fov_point[2], fov_point[3]);
    }

    // modify(1,2,COLOR_PURPLE,1200-BAR/2-2*cap_data_ui,450-WIDTH+BAR/2);
    /* modify(1,2,COLOR_PURPLE,1200-BAR/2-2*joint_100[1],450-WIDTH+BAR/2);
    modify(2,2,COLOR_PURPLE,1400-BAR/2-2*joint_100[2],450-WIDTH+BAR/2);
    modify(3,2,COLOR_PURPLE,1300-BAR/2-2*joint_100[3],400-WIDTH+BAR/2);
    modify(4,2,COLOR_PINK,1500,450+pump_num*300); */

    //		send_int(4,1300,300,COLOR_CYAN,40,4,(int)joint_draw[4]*100);
    //		send_int(5,1300,250,COLOR_CYAN,40,4,(int)joint_draw[5]*100);
    //		send_int(6,1300,200,COLOR_CYAN,40,4,(int)joint_draw[6]*100);

    /*  char_data.interaction_figure.figure_name[2] = 0x24;
     char_data.interaction_figure.start_x = 560;
     char_data.interaction_figure.start_y = 200;
     char_data.interaction_figure.details_a = 20;
     char_data.interaction_figure.details_b = strlen(spin_text);
     memset(char_data.data, 0, 30);
     memcpy(char_data.data, fric_text, strlen(spin_text));
     total_len = fill_tx_buffer((uint8_t *)&char_data, sizeof(char_data));
     UI_UART_SEND(&UI_HUART, tx_buffer, total_len); */
    //		uint16_t line23_end[4];
    //
    //		if(chassis_spin_state == SPINNING)
    //		{
    //			if(spin_direction == (-1)){
    //				line23_end[0] = 300;
    //				line23_end[1] = 1320;
    //				line23_end[2] = 400;
    //				line23_end[3] = 640;
    //			}else{
    //				line23_end[0] = 600;
    //				line23_end[1] = 1620;
    //				line23_end[2] = 640;
    //				line23_end[3] = 400;
    //			}
    //		}else if(battle_45degreee){
    //			line23_end[0] = 640;
    //			line23_end[1] = 1280;
    //			line23_end[2] = 640;
    //			line23_end[3] = 640;
    //		}else{
    //			line23_end[0] = 600;
    //			line23_end[1] = 1320;
    //			line23_end[2] = 640;
    //			line23_end[3] = 640;
    //		}

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

    // modify(4, 2, COLOR_PURPLE,
    //        (uint16_t)CLAMP0((755 + 500 /*show_robot_HP*/), 755 + 600), 850);

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
