/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include <inttypes.h>
#include <tuple>
#include <vector>
#include <utility>

#include "stm32f3xx_hal.h"
#include "usb_device.h"

#include "newlib_stubs.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define ECHO 0
#define TEST_DATA 1
#define CONTINUE 2

#define MORE_DATA 0
#define COMPLETELY_DONE 1
#define TEST_TOP_MUX 2
#define TEST_TOP_DIRECT 3
#define NO_TEST_TOP 4

// This pin turns on the green LED.
static GPIO_TypeDef* PASS_LED_GPIOx = GPIOC;
static uint16_t PASS_LED_Pin = GPIO_PIN_13;

// This pin turns on the yellow LED.
static GPIO_TypeDef* BUSY_LED_GPIOx = GPIOC;
static uint16_t BUSY_LED_Pin = GPIO_PIN_14;

// This pin turns on the red LED.
static GPIO_TypeDef* FAIL_LED_GPIOx = GPIOC;
static uint16_t FAIL_LED_Pin = GPIO_PIN_15;

// Pins for spi to communicate with MPU.


// This pin is connected to the button and will hang the process until pressed.
static GPIO_TypeDef* UART_TX_GPIOx = GPIOD;
static uint16_t UART_TX_Pin = GPIO_PIN_8;
static GPIO_TypeDef* UART_RX_GPIOx = GPIOD;
static uint16_t UART_RX_Pin = GPIO_PIN_9;
static uint8_t UART_command_length = 8;

const std::vector<std::tuple<GPIO_TypeDef*, uint16_t, uint16_t>> dataPins = {
  std::make_tuple(GPIOB, GPIO_PIN_8, 0), // TIM1
  std::make_tuple(GPIOB, GPIO_PIN_9, 1), // TIM2
  std::make_tuple(GPIOB, GPIO_PIN_10, 2), // TIM3
  std::make_tuple(GPIOA, GPIO_PIN_2, 3), // TIM4
  std::make_tuple(GPIOA, GPIO_PIN_13, 1001), // GPIO1
  std::make_tuple(GPIOA, GPIO_PIN_14, 1000), // GPIO2
  std::make_tuple(GPIOE, GPIO_PIN_4, 4), // GPIO3
  std::make_tuple(GPIOE, GPIO_PIN_3, 5), // GPIO4
  std::make_tuple(GPIOE, GPIO_PIN_2, 6), // GPIO5
  std::make_tuple(GPIOB, GPIO_PIN_7, 7), // GPIO6
  //{GPIOA, GPIO_PIN_10), // i2c lines that are pulled high by default.
  //{GPIOA, GPIO_PIN_9}
  // HEIGHT_4
  // HEIGHT_2
  // HEIGHT_1
  // 13: 3V3 LL
  // 3V3 E
  // {GPIOC, GPIO_PIN_0), // +Batt
  // 16: 5V
  // UART8_TX
  // UART8_RX
  std::make_tuple(GPIOB, GPIO_PIN_10, 19), // UART7_TX
  std::make_tuple(GPIOB, GPIO_PIN_11, 20), // UART7_RX
  std::make_tuple(GPIOA, GPIO_PIN_2,  21),  // UART6_TX
  std::make_tuple(GPIOA, GPIO_PIN_3,  22),  // UART6_RX
  std::make_tuple(GPIOC, GPIO_PIN_12, 23), // UART5_TX
  std::make_tuple(GPIOD, GPIO_PIN_2,  24), // UART5_RX
  std::make_tuple(GPIOC, GPIO_PIN_10, 25), // UART4_TX
  std::make_tuple(GPIOC, GPIO_PIN_11, 26), // UART4_RX
  std::make_tuple(GPIOE, GPIO_PIN_0,  27), // UART3_TX
  std::make_tuple(GPIOE, GPIO_PIN_1,  28), // UART3_RX
  std::make_tuple(GPIOD, GPIO_PIN_5,  29), // UART2_TX
  std::make_tuple(GPIOD, GPIO_PIN_6,  30), // UART2_RX
  //{GPIOD, GPIO_PIN_8), // UART1_TX These work if we got as far as talking to this program.
  //{GPIOD, GPIO_PIN_9), // UART1_RX

  // Right side
  std::make_tuple(GPIOC, GPIO_PIN_9,  31),  // TIMG1_CH1
  std::make_tuple(GPIOC, GPIO_PIN_8,  32),  // TIMG1_CH2
  std::make_tuple(GPIOC, GPIO_PIN_7,  33),  // TIMG1_CH3
  std::make_tuple(GPIOC, GPIO_PIN_6,  34),  // TIMG1_CH4
  std::make_tuple(GPIOD, GPIO_PIN_15, 35), // TIMG2_CH1
  std::make_tuple(GPIOD, GPIO_PIN_14, 36), // TIMG2_CH2
  std::make_tuple(GPIOD, GPIO_PIN_13, 37), // TIMG2_CH3
  std::make_tuple(GPIOD, GPIO_PIN_12, 38), // TIMG2_CH4
  std::make_tuple(GPIOC, GPIO_PIN_1,  39),  // ADC1
  std::make_tuple(GPIOC, GPIO_PIN_2,  40),  // ADC2
  // Reserved pins
  // GND
  // BOOT0
  // Reset
  // Two Reserved
  // SPI3_NSS
  // SPI3_SCK
  // SPI3_MOSI
  // SPI3_MISO
  std::make_tuple(GPIOA, GPIO_PIN_15, 54), // SPI2_NSS
  std::make_tuple(GPIOB, GPIO_PIN_3,  55), // SPI2_SCK
  std::make_tuple(GPIOB, GPIO_PIN_4,  56), // SPI2_MISO
  std::make_tuple(GPIOB, GPIO_PIN_5,  57), // SPI2_MOSI
  std::make_tuple(GPIOB, GPIO_PIN_12, 58), // SPI1_NSS
  std::make_tuple(GPIOB, GPIO_PIN_13, 59), // SPI1_SCK
  std::make_tuple(GPIOB, GPIO_PIN_14, 60), // SPI1_MISO
  std::make_tuple(GPIOB, GPIO_PIN_15, 61) // SPI1_MOSI
};

// Only tested for shorts, not connectivity.
const std::vector<std::pair<GPIO_TypeDef*, uint16_t>> internalDataPins = {
  {GPIOA, GPIO_PIN_11}, // USB D+
  {GPIOA, GPIO_PIN_12}, // USB D-
  {GPIOC, GPIO_PIN_13}, // LED0
  {GPIOC, GPIO_PIN_14}, // LED1
  {GPIOC, GPIO_PIN_15}, // LED2
  {GPIOC, GPIO_PIN_3},  // Current
  {GPIOA, GPIO_PIN_4},  // MPU_SPI_NSS
  {GPIOA, GPIO_PIN_5},  // MPU_SPI_SCK
  {GPIOA, GPIO_PIN_6},  // MPU_SPI_MISO
  {GPIOA, GPIO_PIN_7},  // MPU_SPI_MOSI
  {GPIOB, GPIO_PIN_0},  // MPU_INT
  {GPIOE, GPIO_PIN_9},  // MOTOR1
  {GPIOE, GPIO_PIN_11}, // MOTOR2
  {GPIOE, GPIO_PIN_13}, // MOTOR3
  {GPIOE, GPIO_PIN_14}, // MOTOR4
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Input_Z(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Output_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Enable_SWD(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static UART_HandleTypeDef hUART;

HAL_StatusTypeDef receive(uint8_t* buffer) {
  return HAL_UART_Receive(&hUART, (unsigned char*) buffer, UART_command_length, 1000);
}

void wait(void) {
  uint8_t receiveBuffer[UART_command_length];
  while (receive(receiveBuffer) != HAL_OK) {
    HAL_GPIO_TogglePin(BUSY_LED_GPIOx, BUSY_LED_Pin);
  }
}

void send(uint8_t status, uint8_t* buffer, uint8_t buffer_length) {
  uint8_t transmitBuffer[UART_command_length + 1];
  transmitBuffer[0] = MORE_DATA;
  uint8_t total_transmitted = 0;
  while (total_transmitted < buffer_length) {
    if (buffer_length - total_transmitted <= UART_command_length) {
      transmitBuffer[0] = status;
    }
    for (int i = 0; i < UART_command_length; i++) {
      if (total_transmitted < buffer_length) {
        transmitBuffer[i+1] = buffer[total_transmitted];
      } else {
        transmitBuffer[i+1] = 0;
      }
      total_transmitted++;
    }
    if (HAL_UART_Transmit(&hUART, (unsigned char*) transmitBuffer, UART_command_length + 1, 1000) != HAL_OK) {
      Output_High(FAIL_LED_GPIOx, FAIL_LED_Pin);
      Output_High(BUSY_LED_GPIOx, BUSY_LED_Pin);
      for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(FAIL_LED_GPIOx, FAIL_LED_Pin);
        HAL_GPIO_TogglePin(BUSY_LED_GPIOx, BUSY_LED_Pin);
        HAL_Delay(50);
      }
    }
    HAL_Delay(100);
  }
}

void packPinInfo(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t* output) {
  // GPIO address
  output[0] = ((uint32_t) GPIOx) >> 24;
  output[1] = ((uint32_t) GPIOx) >> 16;
  output[2] = ((uint32_t) GPIOx) >> 8;
  output[3] = ((uint32_t) GPIOx);
  // Pin
  output[4] = GPIO_Pin >> 8;
  output[5] = GPIO_Pin;
}

void testData(void) {
  // Set everything to input.
  for (uint i = 0; i < dataPins.size(); ++i) {
    Input_Z(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]));
  }
  HAL_Delay(1);

  for (uint i = 0; i < dataPins.size(); ++i) {
    Output_High(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]));
    HAL_Delay(10);

    // Check where we read high from our other pins.
    uint16_t shorted[16];
    uint8_t numShorted = 0;
    for (uint j = i + 1; j < dataPins.size(); ++j) {
      if (numShorted == 16) break;
      if (std::get<0>(dataPins[j]) == std::get<0>(dataPins[i]) &&
         std::get<1>(dataPins[j]) == std::get<1>(dataPins[i])) {
        continue;
      }
      if (HAL_GPIO_ReadPin(std::get<0>(dataPins[j]), std::get<1>(dataPins[j])) == GPIO_PIN_SET) {
        shorted[numShorted++] = j;
      }
    }

    uint32_t packetLength = 6 * (numShorted + 1) + 1;
    uint8_t pinInfo[packetLength];
    packPinInfo(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]), pinInfo+1);
    uint16_t readIndex = std::get<2>(dataPins[i]);
    uint8_t status = TEST_TOP_MUX;
    if (readIndex >= 1000) {
      pinInfo[0] = (uint8_t) readIndex - 1000;
      status = TEST_TOP_DIRECT;
    } else {
      pinInfo[0] = (uint8_t) readIndex;
    }
    for (int j = 0; j < numShorted; j++) {
      packPinInfo(std::get<0>(dataPins[shorted[j]]), std::get<1>(dataPins[shorted[j]]), pinInfo + 7 + 6 * j);
    }

    bool isLastPin = i == dataPins.size() - 1;
    if (isLastPin) {
      status = COMPLETELY_DONE;
    }

    send(status, pinInfo, packetLength);

    if (!isLastPin) {
      // Wait for the test jig to read the top.
      wait();
      HAL_Delay(50);
    }

    Input_Z(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]));
    HAL_Delay(10);
  }

  // Deinit all of the data pins so they return to their default state. This
  // includes the SWD pins.
  for (uint i = 0; i < dataPins.size(); ++i) {
    HAL_GPIO_DeInit(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]));
  }
  Enable_SWD();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  __USART3_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = UART_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(UART_TX_GPIOx, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = UART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(UART_RX_GPIOx, &GPIO_InitStruct);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  Input_Z(UART_TX_GPIOx, UART_TX_Pin);
  Input_Z(UART_RX_GPIOx, UART_RX_Pin);
}

void cBoardCommsOn(void) {
  hUART.Instance = USART3;
  hUART.Init.BaudRate = 115200;
  hUART.Init.WordLength = UART_WORDLENGTH_8B;
  hUART.Init.StopBits = UART_STOPBITS_1;
  hUART.Init.Parity = UART_PARITY_NONE;
  hUART.Init.Mode = UART_MODE_TX_RX;
  hUART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hUART.Init.OverSampling = UART_OVERSAMPLING_16;
  hUART.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  hUART.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&hUART);
}

void cBoardCommsOff(void) {
  HAL_UART_DeInit(&hUART);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(10);

  Output_High(BUSY_LED_GPIOx, BUSY_LED_Pin);
  Output_High(FAIL_LED_GPIOx, FAIL_LED_Pin);
  Output_High(PASS_LED_GPIOx, PASS_LED_Pin);

  // Set all data pins as pulled down inputs so they can drain any rogue charge.
  cBoardCommsOn();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool bad_sequence = false;
  while (!bad_sequence)
  {
    uint8_t receiveBuffer[UART_command_length];
    HAL_StatusTypeDef status = receive(receiveBuffer);
    if (status == HAL_OK) {
      switch(receiveBuffer[0]) {
        case ECHO:
          // Don't respond immediately so the test jig can do a little
          // processing before listening more.
          HAL_GPIO_TogglePin(PASS_LED_GPIOx, PASS_LED_Pin);
          HAL_Delay(50);
          send(COMPLETELY_DONE, receiveBuffer, UART_command_length);
          break;
        case TEST_DATA:
          testData();
          break;
        case CONTINUE:
          bad_sequence = true;
          break;
        default:
          break;
      }
    } else {
      HAL_GPIO_TogglePin(FAIL_LED_GPIOx, FAIL_LED_Pin);
    }
  }
  cBoardCommsOff();

  Output_High(FAIL_LED_GPIOx, FAIL_LED_Pin);
  while (1)
  {
    HAL_Delay(100);
    HAL_GPIO_TogglePin(FAIL_LED_GPIOx, FAIL_LED_Pin);
  }
  /* USER CODE END 3 */

}

void Input_Z(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Output_High(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void Enable_SWD(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
