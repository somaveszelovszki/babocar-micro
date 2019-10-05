/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "common.h"
#include "pi_controller.h"
#include "encoder.h"
#include "dc_motor.h"

#include <micro/utils/types.h>

#include <micro/panel/MotorPanelData.h>
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
/* Private variables ---------------------------------------------------------*/

static volatile bool isMotorEnabled = false;
static volatile bool useSafetyEnableSignal = true;
static volatile encoder_t encoder;
static volatile uint32_t rc_recv_in_speed = 1500;
static volatile bool newCmd = false;
static volatile pi_controller_t speedCtrl;
static volatile float speed_measured_mps = 0.0f;

motorPanelDataIn_t inData;
motorPanelDataOut_t outData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void update_motor_enabled(void) {

    static const uint8_t ERROR_LIMIT = 3;
    static uint8_t err_cntr = 0;

    bool enabled = true;

    if (useSafetyEnableSignal) {

        // copies received RC pwm atomically
        __disable_irq();
        const uint32_t recvPwm = rc_recv_in_speed;
        __enable_irq();

        // after a given number of errors, stops motor
        if ((recvPwm < 900 || recvPwm > 2100) && ++err_cntr >= ERROR_LIMIT) {
            enabled = false;
        } else {
            err_cntr = 0;
            enabled = (recvPwm >= 1700);
        }
    }

    __disable_irq();
    isMotorEnabled = enabled;
    __enable_irq();
}

void send_speed(void) {
    __disable_irq();
    outData.actualSpeed_mmps = (int16_t)(speed_measured_mps * 1000);
    __enable_irq();

    // transmits actual (measured) speed back to main panel
    HAL_UART_Transmit_DMA(uart_cmd, (uint8_t*)&outData, sizeof(motorPanelDataOut_t));
}

void handle_cmd(void) {

    __disable_irq();
    speedCtrl.desired = inData.targetSpeed_mmps / 1000.0f;
    pi_controller_set_Ti((pi_controller_t*)&speedCtrl, inData.controller_Ti_us);
    pi_controller_set_Kc((pi_controller_t*)&speedCtrl, inData.controller_Kc);
    useSafetyEnableSignal = !!(inData.flags & MOTOR_PANEL_FLAG_USE_SAFETY_SIGNAL);
    __enable_irq();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(uart_cmd, (uint8_t*)&inData, sizeof(motorPanelDataIn_t));

  dc_motor_initialize();
  encoder_initialize((encoder_t*)&encoder, ENCODER_MAX_VALUE);
  pi_controller_initialize((pi_controller_t*)&speedCtrl, SPEED_CTRL_PERIOD_US, SPEED_CTRL_Ti_US, SPEED_CTRL_Kc, SPEED_CTRL_DEADBAND_MPS, -1.0f, 1.0f, SPEED_CTRL_MAX_DELTA);

  uint32_t lastCmdTime               = HAL_GetTick();
  uint32_t lastSafetySignalCheckTime = HAL_GetTick();
  uint32_t lastSpeedSendTime         = HAL_GetTick();
  uint32_t lastLedToggleTime         = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  speedCtrl.desired = 0.0f;

  while (1)
  {
      // TODO
      {
          __disable_irq();
          volatile int32_t recvPwm = (int32_t)rc_recv_in_speed;
          __enable_irq();

          // after a given number of errors, stops motor
          if (recvPwm > 900 && recvPwm < 2100) {
              speedCtrl.desired = map(recvPwm, 1000, 2000, -1.0f, 1.0f);
          }
      }

      const uint32_t currentTime = HAL_GetTick();
      if (newCmd) {
          newCmd = false;
          lastCmdTime = currentTime;
          handle_cmd();
      }

      if (currentTime - lastCmdTime > MAX_CMD_DELAY_MS) {
          __disable_irq();
          //speedCtrl.desired = 0.0f;
          __enable_irq();
      }

      if (currentTime >= lastSpeedSendTime + SPEED_SEND_PERIOD_MS) {
    	  lastSpeedSendTime = currentTime;
    	  send_speed();
      }

      if (currentTime >= lastSafetySignalCheckTime + SAFETY_SIGNAL_CHECK_PERIOD_MS) {
          lastSafetySignalCheckTime = currentTime;
          update_motor_enabled();
      }

      if (currentTime >= lastLedToggleTime + 500) {
          lastLedToggleTime = currentTime;
          HAL_GPIO_TogglePin(gpio_user_led, gpio_pin_user_led);
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == tim_rc_recv) {
		rc_recv_in_speed = __HAL_TIM_GET_COMPARE(tim_rc_recv, chnl_rc_recv);
		__HAL_TIM_SET_COUNTER(tim_rc_recv, 0); 	//resets counter after input capture interrupt occurs
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_cmd) {
        newCmd = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == tim_speedControllerPeriod) {

		const int32_t diff = encoder_get_diff((encoder_t*)&encoder);
		speed_measured_mps = diff * ENCODER_TO_MPS_RATIO;

		if (isMotorEnabled || 1)
		{
	        pi_controller_update((pi_controller_t*)&speedCtrl, speed_measured_mps);
	        //dc_motor_write(speedCtrl.output);
	        dc_motor_write(speedCtrl.desired);
		} else {
		    dc_motor_write(0.0f);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
