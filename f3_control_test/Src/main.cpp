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
#include <math.h>
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

#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

#define ECHO 0
#define TEST_DATA 1
#define CONTINUE 2
#define TEST_INTERNAL_DATA 3
#define READ_UNIQUE_ID 4
#define TEST_GYRO 5

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
static uint16_t MPU_NSS_Pin = GPIO_PIN_4;
static uint16_t MPU_SCK_Pin = GPIO_PIN_5;
static uint16_t MPU_MISO_Pin = GPIO_PIN_6;
static uint16_t MPU_MOSI_Pin = GPIO_PIN_7;
static GPIO_TypeDef* MPU_GPIOx = GPIOA;


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
  {GPIOE, GPIO_PIN_9},  // MOTOR1
  {GPIOE, GPIO_PIN_11}, // MOTOR2
  {GPIOE, GPIO_PIN_13}, // MOTOR3
  {GPIOE, GPIO_PIN_14},  // MOTOR4
  {GPIOA, GPIO_PIN_11}, // USB D-
  {GPIOA, GPIO_PIN_12}, // USB D+
  {GPIOC, GPIO_PIN_13}, // LED0
  {GPIOC, GPIO_PIN_14}, // LED1
  {GPIOC, GPIO_PIN_15} // LED2
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

void testInternalData(void) {
  // Set everything to input.
  for (uint i = 0; i < internalDataPins.size(); ++i) {
    Input_Z(internalDataPins[i].first, internalDataPins[i].second);
  }
  HAL_Delay(1);

  for (uint i = 0; i < internalDataPins.size(); ++i) {
    Output_High(internalDataPins[i].first, internalDataPins[i].second);
    HAL_Delay(10);

    // Check where we read high from our other pins.
    uint16_t shorted[16];
    uint8_t numShorted = 0;
    for (uint j = i + 1; j < internalDataPins.size(); ++j) {
      if (numShorted == 16) break;
      if (HAL_GPIO_ReadPin(internalDataPins[j].first, internalDataPins[j].second) == GPIO_PIN_SET) {
        shorted[numShorted++] = j;
      }
    }

    uint32_t packetLength = 6 * (numShorted + 1);
    uint8_t pinInfo[packetLength];
    packPinInfo(internalDataPins[i].first, internalDataPins[i].second, pinInfo);
    uint8_t status = NO_TEST_TOP;
    for (int j = 0; j < numShorted; j++) {
      packPinInfo(internalDataPins[shorted[j]].first, internalDataPins[shorted[j]].second, pinInfo + 6 + 6 * j);
    }

    bool isLastPin = i == internalDataPins.size() - 1;
    if (isLastPin) {
      status = COMPLETELY_DONE;
    }

    send(status, pinInfo, packetLength);

    Input_Z(internalDataPins[i].first, internalDataPins[i].second);
    HAL_Delay(10);
  }

  // Deinit all of the data pins so they return to their default state. This
  // includes the SWD pins.
  for (uint i = 0; i < dataPins.size(); ++i) {
    HAL_GPIO_DeInit(std::get<0>(dataPins[i]), std::get<1>(dataPins[i]));
  }
  Enable_SWD();
}

void readUniqueId(void) {
  uint8_t idBuffer[3 * 4];
  idBuffer[0] = U_ID_0 >> 24;
  idBuffer[1] = U_ID_0 >> 16;
  idBuffer[2] = U_ID_0 >> 8;
  idBuffer[3] = U_ID_0;
  idBuffer[4] = U_ID_1 >> 24;
  idBuffer[5] = U_ID_1 >> 16;
  idBuffer[6] = U_ID_1 >> 8;
  idBuffer[7] = U_ID_1;
  idBuffer[8] = U_ID_2 >> 24;
  idBuffer[9] = U_ID_2 >> 16;
  idBuffer[10] = U_ID_2 >> 8;
  idBuffer[11] = U_ID_2;
  send(COMPLETELY_DONE, idBuffer, 3 * 4);
}

SPI_HandleTypeDef hspi;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = MPU_SCK_Pin | MPU_MISO_Pin | MPU_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(MPU_GPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MPU_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(MPU_GPIOx, &GPIO_InitStruct);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_DISABLE();
  }
}

#define FS_250DPS 0x00

#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_TRIMS			0x0D

#define MPU_READ  0x80
#define MPU_WRITE 0x00

#define MPU_CLK_SEL_PLLGYROZ	    0x03

#define BIT_H_RESET 0x80
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

#define MPUREG_ACC 0x3B

#define MPUREG_PRODUCT_ID       0x0C
#define MPUREG_SIGNAL_PATH_RESET    0x68
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_WHO_AM_I 0x75

#define MPU_TEST_SAMPLES 100

#define MPU_WHO_AM_I_VALUE 0x68

uint8_t MPURead(uint8_t address) {
  uint8_t transmitBuffer[2] = {MPU_READ | address, 0};
  uint8_t receiverBuffer[2] = {0, 0};
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi, transmitBuffer, receiverBuffer, 2, 50);
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    return 0xff;
  }
  return receiverBuffer[1];
}

uint8_t MPUWrite(uint8_t address, uint8_t value) {
  uint8_t transmitBuffer[2] = {MPU_WRITE | address, value};
  uint8_t receiverBuffer[2] = {0, 0};
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_RESET);
  // NOTE(tannewt): Don't use HAL_SPI_Transmit! It doesn't manage the receive
  // buffer and will lead to shifted receive values from subsequent
  // TransmitReceive calls.
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi, transmitBuffer, receiverBuffer, 2, 50);
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK) {
    return 0xff;
  }
  return 0x09;
}

int16_t
int16_t_from_bytes(uint8_t bytes[])
{
  union {
    uint8_t    b[2];
    int16_t    w;
  } u;

  u.b[1] = bytes[0];
  u.b[0] = bytes[1];

  return u.w;
}

// Based on PX4 code.
void testGyro(void) {
  hspi.Instance = SPI1;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi.Init.NSS = SPI_NSS_SOFT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi.Init.CRCPolynomial = 7;
  hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi);
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);

  uint8_t testOutput[64];
  uint8_t totalBytes = 0;
  testOutput[totalBytes++] = MPUWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  // Wait for startup.
  bool gyroFound = false;
  for (int i = 0; i < 5; i++){
    HAL_Delay(10);
    uint8_t readValue = MPURead(MPUREG_WHO_AM_I);
    testOutput[totalBytes++] = readValue;
    if (readValue == MPU_WHO_AM_I_VALUE) {
      gyroFound = true;
      break;
    }
  }
  if (!gyroFound) {
    testOutput[totalBytes++] = 0xee;
    send(COMPLETELY_DONE, testOutput, 8);
    return;
  }

  // Read gyro info so we can log it.
  testOutput[totalBytes++] = MPURead(MPUREG_PRODUCT_ID);

  testOutput[totalBytes++] = MPUWrite(MPUREG_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  HAL_Delay(2);

  // Clock Source PPL with Z axis gyro reference
  testOutput[totalBytes++] = MPUWrite(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  HAL_Delay(1);

  testOutput[totalBytes++] = MPUWrite(0x19, 0);
  HAL_Delay(1);

  // Disable Primary I2C Interface
  testOutput[totalBytes++] = MPUWrite(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  HAL_Delay(1);

  testOutput[totalBytes++] = MPUWrite(MPUREG_PWR_MGMT_2, 0x00);
  HAL_Delay(1);

  testOutput[totalBytes++] = MPUWrite(MPUREG_GYRO_CONFIG, FS_250DPS);
  HAL_Delay(1);

  testOutput[totalBytes++] = MPUWrite(MPUREG_ACCEL_CONFIG,  0x10);
  HAL_Delay(1);

  // Set DLPF
  testOutput[totalBytes++] = MPUWrite(0x1A, 0x00);
  HAL_Delay(100);

  float accelBaseline[3] = {0, 0, 0};
  float gyroBaseline[3] = {0, 0, 0};
  float accel[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};
  float accelFactoryTrim[3] = {0, 0, 0};
  float gyroFactoryTrim[3] = {0, 0, 0};

  uint8_t transmitBuffer[1 + 6 + 2 + 6];
  uint8_t receiverBuffer[1 + 6 + 2 + 6];
  transmitBuffer[0] = MPU_READ | MPUREG_ACC;
  for (uint8_t i = 0; i < MPU_TEST_SAMPLES; i++) {
    HAL_Delay(1);
    HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi, transmitBuffer, receiverBuffer, 1 + 6 + 2 + 6, 100) != HAL_OK) {
      testOutput[totalBytes++] = 0xee;
      return;
    }
    HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);

    accelBaseline[0] += int16_t_from_bytes(receiverBuffer + 1);
    accelBaseline[1] += int16_t_from_bytes(receiverBuffer + 3);
    accelBaseline[2] += int16_t_from_bytes(receiverBuffer + 5);
    // Skip temp sensor reading.
    gyroBaseline[0] += int16_t_from_bytes(receiverBuffer + 9);
    gyroBaseline[1] += int16_t_from_bytes(receiverBuffer + 11);
    gyroBaseline[2] += int16_t_from_bytes(receiverBuffer + 13);
  }
  testOutput[totalBytes++] = MPUWrite(MPUREG_GYRO_CONFIG, FS_250DPS | BITS_GYRO_ST_X | BITS_GYRO_ST_Y | BITS_GYRO_ST_Z);

  // accel 8g, self-test enabled all axes
  testOutput[totalBytes++] = MPUWrite(MPUREG_ACCEL_CONFIG, 0x10 | 0xE0);

  HAL_Delay(20);

  for (uint8_t i = 0; i < MPU_TEST_SAMPLES; i++) {
    HAL_Delay(1);
    HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi, transmitBuffer, receiverBuffer, 1 + 6 + 2 + 6, 100) != HAL_OK) {
      testOutput[totalBytes++] = 0xe2;
      send(COMPLETELY_DONE, testOutput, totalBytes);
      return;
    }
    HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);

    accel[0] += int16_t_from_bytes(receiverBuffer + 1);
    accel[1] += int16_t_from_bytes(receiverBuffer + 3);
    accel[2] += int16_t_from_bytes(receiverBuffer + 5);
    // Skip temp sensor reading.
    gyro[0] += int16_t_from_bytes(receiverBuffer + 9);
    gyro[1] += int16_t_from_bytes(receiverBuffer + 11);
    gyro[2] += int16_t_from_bytes(receiverBuffer + 13);
  }

  for (uint8_t i = 0; i < 3; i++) {
    accelBaseline[i] /= MPU_TEST_SAMPLES;
    gyroBaseline[i] /= MPU_TEST_SAMPLES;
    accel[i] /= MPU_TEST_SAMPLES;
    gyro[i] /= MPU_TEST_SAMPLES;
  }

  // extract factory trim values
  uint8_t trims[5];
  uint8_t readTrim[5] = {MPU_READ | MPUREG_TRIMS, 0, 0, 0, 0};
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi, readTrim, trims, 5, 10) != HAL_OK) {
    testOutput[totalBytes++] = 0xe1;
    send(COMPLETELY_DONE, testOutput, totalBytes);
    return;
  }
  HAL_GPIO_WritePin(MPU_GPIOx, MPU_NSS_Pin, GPIO_PIN_SET);
  uint8_t atrim[3];
  uint8_t gtrim[3];

  atrim[0] = ((trims[1] >> 3) & 0x1C) | ((trims[4] >> 4) & 0x03);
  atrim[1] = ((trims[2] >> 3) & 0x1C) | ((trims[4] >> 2) & 0x03);
  atrim[2] = ((trims[3] >> 3) & 0x1C) | ((trims[4] >> 0) & 0x03);
  gtrim[0] = trims[1] & 0x1F;
  gtrim[1] = trims[2] & 0x1F;
  gtrim[2] = trims[3] & 0x1F;

  // convert factory trims to right units
  for (uint8_t i = 0; i < 3; i++) {
    accelFactoryTrim[i] = 4096 * 0.34f * powf(0.92f / 0.34f, (atrim[i] - 1) / 30.0f);
    gyroFactoryTrim[i] = 25 * 131.0f * powf(1.046f, gtrim[i] - 1);
  }

  // Y gyro trim is negative
  gyroFactoryTrim[1] *= -1;

  testOutput[totalBytes++] = 0x09;
  for (uint8_t i = 0; i < 3; i++) {
    float diff = accel[i] - accelBaseline[i];
    float err = 100 * (diff - accelFactoryTrim[i]) / accelFactoryTrim[i];
    testOutput[totalBytes++] = (uint8_t) fabsf(err);
  }

  testOutput[totalBytes++] = 0x09;
  for (uint8_t i = 0; i < 3; i++) {
    float diff = gyro[i] - gyroBaseline[i];
    float err = 100 * (diff - gyroFactoryTrim[i]) / gyroFactoryTrim[i];
    testOutput[totalBytes++] = (uint8_t) fabsf(err);
  }
  testOutput[totalBytes++] = 0x09;
  send(COMPLETELY_DONE, testOutput, totalBytes);
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
      HAL_Delay(50);
      switch(receiveBuffer[0]) {
        case ECHO:
          // Don't respond immediately so the test jig can do a little
          // processing before listening more.
          send(COMPLETELY_DONE, receiveBuffer, UART_command_length);
          break;
        case TEST_DATA:
          testData();
          break;
        case TEST_INTERNAL_DATA:
          HAL_GPIO_TogglePin(PASS_LED_GPIOx, PASS_LED_Pin);
          testInternalData();
          break;
        case READ_UNIQUE_ID:
          //HAL_GPIO_TogglePin(PASS_LED_GPIOx, PASS_LED_Pin);
          readUniqueId();
          break;
        case TEST_GYRO:
          testGyro();
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
