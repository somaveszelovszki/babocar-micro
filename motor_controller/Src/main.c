
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "config.h"
#include "pi_controller.h"
#include "encoder.h"
#include "dc_motor.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef  * const tim_motor                 = &htim1;
TIM_HandleTypeDef  * const tim_encoder               = &htim3;
TIM_HandleTypeDef  * const tim_rc_recv               = &htim14;
TIM_HandleTypeDef  * const tim_speedControllerPeriod = &htim17;
UART_HandleTypeDef * const uart_cmd                  = &huart1;

const    uint32_t          chnl_fwd                  = 1;
const    uint32_t          chnl_bwd                  = 2;
const    uint32_t          chnl_rc_recv              = 1;

volatile uint32_t          rc_recv_in_speed          = 1500;
volatile uint8_t           newCmd                    = 0;

volatile pi_controller_t   speedCtrl;
volatile encoder_t         encoder;
volatile float             speed_measured_mps        = 0.0f;
volatile uint8_t           useSafetyEnableSignal     = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t is_motor_enabled() {
    if (useSafetyEnableSignal) {
        static const uint8_t ERROR_LIMIT = 3;
        static uint8_t err_cntr = 0;

        // copies received RC pwm atomically
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        int32_t recvPwm = (int32_t)rc_recv_in_speed;
        if (primask) __enable_irq();

        // after a given number of errors, stops motor
        if ((recvPwm < 900 || recvPwm > 2100) && ++err_cntr >= ERROR_LIMIT) {
            return 0;
        }

        return recvPwm >= 1700 ? 1 : 0;
    }

    return 1;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  typedef enum {
      Code_SAFETY_ENABLE_SIGNAL = 1,    // Defines if safety enable signal is enabled (1: enabled, 0: disabled)
      Code_TARGET_SPEED         = 2,    // Sets target speed
      Code_PI_CONTROLLER_Ti     = 3,    // Sets speed controller's Ti [us]
      Code_PI_CONTROLLER_Kc     = 4     // Sets speed controller's Kc
  } Code;

  uint8_t ACK_template[TX_SIZE] = { 0, 0, 0, 0, 0 };

  uint8_t rxBuffer[RX_SIZE], txBuffer[TX_SIZE];
  HAL_UART_Receive_DMA(uart_cmd, rxBuffer, RX_SIZE);

  dc_motor_initialize();
  encoder_initialize((encoder_t*)&encoder, ENCODER_MAX_VALUE);
  pi_controller_initialize((pi_controller_t*)&speedCtrl, SPEED_CTRL_PERIOD_US, SPEED_CTRL_Ti_us, SPEED_CTRL_Kc, SPEED_CTRL_DEADBAND_MPS, -SPEED_CTRL_OUT_MAX, SPEED_CTRL_OUT_MAX);

  uint32_t lastCmdTime = HAL_GetTick();
  uint32_t lastSpeedSendTime = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      const uint32_t currentTime = HAL_GetTick();
      if (newCmd) {
          newCmd = 0;
          lastCmdTime = currentTime;

          const Code code = (Code)rxBuffer[0];

          switch (code) {

          case Code_SAFETY_ENABLE_SIGNAL:
              useSafetyEnableSignal = rxBuffer[1];
              break;

          case Code_TARGET_SPEED:
          {
              if (is_motor_enabled()) {
                  __disable_irq();
                  speedCtrl.desired = *(float*)(&rxBuffer[1]);
                  __enable_irq();
              } else {
                  __disable_irq();
                  speedCtrl.desired = 0.0f;
                  __enable_irq();
              }

              break;
          }

          case Code_PI_CONTROLLER_Ti:
              __disable_irq();
              pi_controller_set_Ti((pi_controller_t*)&speedCtrl, *(uint32_t*)(&rxBuffer[1]));
              __enable_irq();
              break;

          case Code_PI_CONTROLLER_Kc:
              __disable_irq();
              pi_controller_set_Ti((pi_controller_t*)&speedCtrl, *(float*)(&rxBuffer[1]));
              __enable_irq();
              break;
          }

          ACK_template[0] = (uint8_t)code;
          HAL_UART_Transmit_DMA(uart_cmd, ACK_template, TX_SIZE);
      }

      if (currentTime - lastCmdTime > MAX_CMD_DELAY_MS) {
          __disable_irq();
          speedCtrl.desired = 0.0f;
          __enable_irq();
      }

      if (currentTime >= lastSpeedSendTime + SPEED_SEND_PERIOD_MS) {
    	  lastSpeedSendTime = currentTime;

    	  __disable_irq();
    	  *(float*)txBuffer = speed_measured_mps;
    	  __enable_irq();

          // transmits actual (measured) speed back to main panel
          HAL_UART_Transmit_DMA(uart_cmd, txBuffer, 4);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
        newCmd = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == tim_speedControllerPeriod) {
		const int32_t diff = encoder_get_diff((encoder_t*)&encoder);
		speed_measured_mps = diff * ENCODER_TO_MPS_RATIO;
		pi_controller_update((pi_controller_t*)&speedCtrl, speed_measured_mps);
		dc_motor_write(speedCtrl.output, useSafetyEnableSignal);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
