/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./base_drv/drv_conf.h"
#include "./imu.h"

#include gimbal_module_core_h
#include chassis_module_core_h
#include gimbal_module_motor_ctrl_h
#include chassis_module_motor_ctrl_h
#include "./base_drv/drv_buzzer/buzzer.h"
#include "./base_drv/drv_cap/super_cap.h"
#include "./base_drv/drv_motor/motor_ctrl.h"
#include "./base_drv/drv_rc/rc.h"
#include "./base_drv/drv_referee/referee.h"
#include "./base_drv/graphic/drv_graphic.h"
#include "./base_drv/graphic/graphic_def.h"
#include "./base_drv/power_ctrl.h"

#include "./robot_core/ctrl_core/robot.h"
#include "./robot_core/interface/interface_BTB.h"
#include "./robot_core/tool/vofa.h"
#include "./robot_core/weapon/vision.h"

#include "./robot_core/tool/utils/trice.h"

#define TRICE_FILE Id(0)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t MAX_time_CNT = 10;
HAL_CAN_StateTypeDef StateofCAN1 = HAL_CAN_STATE_ERROR;
HAL_CAN_StateTypeDef StateofCAN2 = HAL_CAN_STATE_ERROR;

float charge_power = 30;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float temp_shoot_data = 0;

void System_Reset(void) {
  TRICE(Id(35751), "MSG: try to reset robot,RCC_CSR=0x%x\n", RCC->CSR);

  shutdown_all_chassis_motor();
  shutdown_all_gimbal_motor();

  for (uint32_t i = 0; i < 1000; i++) {

    set_all_chassis_motor_output();
    set_all_gimbal_motor_output();
  }
  __set_FAULTMASK(1); // 关闭�???有中�???
  __disable_irq();
  NVIC_SystemReset(); // 复位
}

/**
 *	@breif 看门狗初始化
 *	@param[1] hiwdg.Instance->KR
 *						键寄存器
 *						当写�?????????0XCCCC就启动看门狗的工作；
 *						当写�?????????0X5555时表示允许访问IWDG_PR和IWDG_RLR寄存器，只有允许访问这两个地�?????????以后，才能改变独立看门狗的预分频值和重装载�?�；
 *						当写�?????????0XAAAA的时候，独立看门狗进行更新，防止产生复位�?????????
 *	@param[2] hiwdg.Instance->PR
 *						预分频寄存器
 *						分频值为4*2^PR
 *	@param[3] hiwdg.Instance->RLR
 *						重装载寄存器
 *						当IWDG的�?�减�?????????0以后，系统复位，IWDG的重装载寄存器的值就会加载到递减计数器中进行重新计数�?????????
 *						当IWDG及时“喂狗�?�，以后，IWDG的重装载寄存器的值也会加载到递减计数器中，使IWDG重新计数�?????????
 *	@param[4] hiwdg.Instance->SR
 *						状�?�寄存器,�?????????
 *	@note[1]:	C板看门狗是接到LSI 32khz,喂狗用时Tout=( (4×2^PR) ×
 *(RLP+1) )/LSI,此处设定频率�?????????400hz
 *						(贴着控制频率容易�?????????直卡�?????????,建议留多点余�?????????)
 *	@note[2]:
 *调用MX_IWDG_Init的时候就已经�?????????启看门狗�?????????,为了防止robot_init用时过久疯狗咬人,
 *						请自行在MX_IWDG_Init里加�?????????#if
 *0
 *......#endif注释�?????????
 *	@note[3]:
 *平常�?????????debug调试时记得关掉看门狗,不然�?????????打断点就会触发看门狗
 */
bool IS_SYSTEM_RESET = true;
void watchdog_init(void) {
  //  hiwdg.Instance = IWDG;
  //  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  //  hiwdg.Init.Reload = 20 - 1;
  // #if enable_watch_dog
  //  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
  //    Error_Handler();
  //  }
  // #endif
  //  HAL_IWDG_Refresh(&hiwdg);
  /*
  hiwdg.Instance->KR=0x5555;
  hiwdg.Instance->PR=0;		//设定时钟4分频
  hiwdg.Instance->RLR=10-1;
  hiwdg.Instance->KR=0xCCCC;
  */
}

// #define DISABLE_USB

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rst_cnt = 0;
uint8_t error_emergency = 0;
void error_show(uint8_t emergency) {
  switch (emergency) {
  case 1:
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

    break;
  case 2:
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    break;
  default:
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    break;
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if 0
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
#endif
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  //

  // init_UI();
  // init_UI();
  referee_recv_dma_init();
  // 云台初始�??????????(含电机�?�遥控器、板间交互�?�tim6与tim14中断�??????????�??????????)
  robot_init();
  HAL_Delay(500);

  watchdog_init();
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    HAL_Delay(10);

    // 手动断控
    extern chassis_motors_t chassis_motors;
    extern gimbal_motors_t gimbal_motors;

    // 更新裁判系统
    update_referee_data_to_BTB();

    // 蜂鸣器警�??????????
    //     if (is_the_receiver_offline())
    //       beep(1, 1000, 0.01); // 1s

    StateofCAN1 = HAL_CAN_GetState(&hcan1);
    StateofCAN2 = HAL_CAN_GetState(&hcan2);
    inc_referee_rx_lost();
    if (robot.ctrl_mode != 0) {
      error_emergency = 2;
    } else if (robot.gimbal_motor_status != 0 ||
               robot.chassis_motor_status != 0) {
      error_emergency = 1;
    } else {
      error_emergency = 0;
    }

    error_show(error_emergency);
    update_UI();

    //    extern int cap_data_ui;
    //    extern chassis_power_lim_t chassis_power_lim;
    //    cap_data_ui = 100 * chassis_power_lim.real_cap_ratio;
    // update_UI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
