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
#include "stm32f3xx_hal_i2c.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
extern const char *gitversion;
extern const char *build_date;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// This pin reads the value from the top.
static GPIO_TypeDef* TOP_READ_GPIOx = GPIOA;
static uint16_t TOP_READ_Pin = GPIO_PIN_13;

// This pin turns on the PASS LED.
static GPIO_TypeDef* PASS_LED_GPIOx = GPIOE;
static uint16_t PASS_LED_Pin = GPIO_PIN_11;

// This pin turns on the BUSY LED.
static GPIO_TypeDef* BUSY_LED_GPIOx = GPIOE;
static uint16_t BUSY_LED_Pin = GPIO_PIN_10;

// This pin turns on the FAIL LED.
static GPIO_TypeDef* FAIL_LED_GPIOx = GPIOE;
static uint16_t FAIL_LED_Pin = GPIO_PIN_13;

// This pin turns on the power to the expansion.
static GPIO_TypeDef* POWER_ENABLE_GPIOx = GPIOF;
static uint16_t POWER_ENABLE_Pin = GPIO_PIN_9;

// This pin detects power faults.
static GPIO_TypeDef* POWER_FAULT_GPIOx = GPIOE;
static uint16_t POWER_FAULT_Pin = GPIO_PIN_4;

// This pin is connected to the button and will hang the process until pressed.
static GPIO_TypeDef* GO_GPIOx = GPIOC;
static uint16_t GO_Pin = GPIO_PIN_3;

// Pins for the height. Affects the address of the EEPROM memory.
static GPIO_TypeDef* HEIGHT_4_GPIOx = GPIOC;
static uint16_t HEIGHT_4_Pin = GPIO_PIN_13;
static uint16_t HEIGHT_4_Address = 10;
static GPIO_TypeDef* HEIGHT_2_GPIOx = GPIOC;
static uint16_t HEIGHT_2_Pin = GPIO_PIN_14;
static uint16_t HEIGHT_2_Address = 11;
static GPIO_TypeDef* HEIGHT_1_GPIOx = GPIOC;
static uint16_t HEIGHT_1_Pin = GPIO_PIN_15;
static uint16_t HEIGHT_1_Address = 12;

// Pins for i2c to communicate with EEPROM.
static GPIO_TypeDef* i2c_SDA_GPIOx = GPIOB;
static uint16_t i2c_SDA_Pin = GPIO_PIN_7;
static GPIO_TypeDef* i2c_SCL_GPIOx = GPIOB;
static uint16_t i2c_SCL_Pin = GPIO_PIN_6;

// This pin is connected to the button and will hang the process until pressed.
static GPIO_TypeDef* UART_TX_GPIOx = GPIOD;
static uint16_t UART_TX_Pin = GPIO_PIN_8;
static GPIO_TypeDef* UART_RX_GPIOx = GPIOD;
static uint16_t UART_RX_Pin = GPIO_PIN_9;
static uint8_t UART_command_length = 8;

// This pin resets the board under test and if given enough time will reset to bootloader.
static GPIO_TypeDef* RESET_GPIOx = GPIOC;
static uint16_t RESET_Pin = GPIO_PIN_7;

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

const std::vector<std::pair<GPIO_TypeDef*, uint16_t>> powerPins = {
  {GPIOC, GPIO_PIN_0}, // PC0 - 3V3
  {GPIOF, GPIO_PIN_10}, // PF10 - 5V
  {GPIOC, GPIO_PIN_8} // PC8 - GND
};

const std::vector<std::pair<GPIO_TypeDef*, uint16_t>> addressPins = {
  {GPIOE, GPIO_PIN_12}, // PE12 - Blue
  {GPIOE, GPIO_PIN_11},  // PE11 - Green
  {GPIOE, GPIO_PIN_10},  // PE10 - Orange
  {GPIOE, GPIO_PIN_9},  // PE9  - Red
  {GPIOE, GPIO_PIN_8},  // PE8  - Blue
  {GPIOE, GPIO_PIN_15},  // PE15 - Green
  {GPIOE, GPIO_PIN_14}  // PE14 - Orange
};

const std::vector<std::pair<GPIO_TypeDef*, uint16_t>> dataPins = {
  {GPIOD, GPIO_PIN_5}, // TIM1
  {GPIOD, GPIO_PIN_7}, // TIM2
  {GPIOB, GPIO_PIN_4}, // TIM3
  {GPIOB, GPIO_PIN_5}, // TIM4
  {GPIOB, GPIO_PIN_8}, // GPIO1
  {GPIOB, GPIO_PIN_9}, // GPIO2
  {GPIOE, GPIO_PIN_0}, // GPIO3
  {GPIOE, GPIO_PIN_1}, // GPIO4
  {GPIOE, GPIO_PIN_3}, // GPIO5
  {GPIOE, GPIO_PIN_6}, // GPIO6
  //{GPIOB, GPIO_PIN_7}, // i2c lines that are pulled high by default.
  //{GPIOB, GPIO_PIN_6}
  {GPIOC, GPIO_PIN_13}, // HEIGHT_4
  {GPIOC, GPIO_PIN_14}, // HEIGHT_2
  {GPIOC, GPIO_PIN_15}, // HEIGHT_1
  //{GPIOC, GPIO_PIN_0}, // 13: 3V3 LL
  {GPIOC, GPIO_PIN_1}, // 3V3 E
  //{GPIOC, GPIO_PIN_2}, // +Batt
  //{GPIOF, GPIO_PIN_10}, // 16: 5V
  {GPIOC, GPIO_PIN_3}, // UART8_TX/Switch
  {GPIOF, GPIO_PIN_2}, // UART8_RX
  {GPIOA, GPIO_PIN_1}, // UART7_TX
  {GPIOA, GPIO_PIN_3}, // UART7_RX
  {GPIOA, GPIO_PIN_2}, // UART6_TX
  {GPIOF, GPIO_PIN_4}, // UART6_RX
  {GPIOA, GPIO_PIN_5}, // UART5_TX
  {GPIOA, GPIO_PIN_4}, // UART5_RX
  {GPIOA, GPIO_PIN_7}, // UART4_TX
  {GPIOA, GPIO_PIN_6}, // UART4_RX
  {GPIOC, GPIO_PIN_5}, // UART3_TX
  {GPIOC, GPIO_PIN_4}, // UART3_RX
  {GPIOB, GPIO_PIN_0}, // UART2_TX
  {GPIOB, GPIO_PIN_1}, // UART2_RX
  {GPIOB, GPIO_PIN_2}, // UART1_TX
  {GPIOE, GPIO_PIN_7}, // UART1_RX

  // Right side
  {GPIOD, GPIO_PIN_6},  // TIMG1_CH1
  {GPIOD, GPIO_PIN_3},  // TIMG1_CH2
  {GPIOD, GPIO_PIN_4},  // TIMG1_CH3
  {GPIOD, GPIO_PIN_1},  // TIMG1_CH4
  {GPIOD, GPIO_PIN_2},  // TIMG2_CH1
  {GPIOC, GPIO_PIN_12}, // TIMG2_CH2
  {GPIOD, GPIO_PIN_0},  // TIMG2_CH3
  {GPIOC, GPIO_PIN_10}, // TIMG2_CH4
  {GPIOC, GPIO_PIN_11},
  {GPIOA, GPIO_PIN_14},
  {GPIOA, GPIO_PIN_15},
  {GPIOF, GPIO_PIN_6},
  {GPIOA, GPIO_PIN_10},
  {GPIOA, GPIO_PIN_9},
  {GPIOA, GPIO_PIN_8},
  {GPIOC, GPIO_PIN_9},
  //{GPIOC, GPIO_PIN_8}, // GND
  {GPIOB, GPIO_PIN_3}, // BOOT0
  {GPIOA, GPIO_PIN_0}, // Reset
  {GPIOD, GPIO_PIN_15},
  {GPIOD, GPIO_PIN_14},
  {GPIOD, GPIO_PIN_13},
  {GPIOD, GPIO_PIN_12},
  {GPIOD, GPIO_PIN_11},
  {GPIOD, GPIO_PIN_10},
  {GPIOD, GPIO_PIN_9},
  {GPIOD, GPIO_PIN_8},
  {GPIOB, GPIO_PIN_15},
  {GPIOB, GPIO_PIN_14},
  {GPIOB, GPIO_PIN_13},
  {GPIOB, GPIO_PIN_12},
  {GPIOB, GPIO_PIN_11},
  {GPIOB, GPIO_PIN_10}
};

const std::vector<std::pair<GPIO_TypeDef*, uint16_t>> topDirect = {
  {GPIOE, GPIO_PIN_2}, // PE2 - TEST_SWCLK
  {GPIOE, GPIO_PIN_5},  // PE5 - TEST_SWDIO
  {GPIOC, GPIO_PIN_6},  // PC6 - BOOT0
  {GPIOC, GPIO_PIN_7},  // PC7 - RESET
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

void setAddress(uint address) {
  for (uint k = 0; k < addressPins.size(); ++k) {
    GPIO_PinState value;
    if (((address >> k) & 1) == 1) {
      value = GPIO_PIN_SET;
    } else {
      value = GPIO_PIN_RESET;
    }
    HAL_GPIO_WritePin(addressPins[k].first, addressPins[k].second, value);
  }
}

void topOn_SWDOff(void) {
  Input_Z(TOP_READ_GPIOx, TOP_READ_Pin);
  HAL_Delay(1);

  // Turn on the address pins.
  for (uint k = 0; k < addressPins.size(); ++k) {
    Output_High(addressPins[k].first, addressPins[k].second);
  }
}

void topOff_SWDOn(void) {
  Enable_SWD();

  for (uint k = 0; k < addressPins.size(); ++k) {
    Input_Z(addressPins[k].first, addressPins[k].second);
  }
}

void testTop(uint8_t startIndex, uint8_t endIndex) {
  // DO NOT set breakpoints between here and the note below. SWD is temporarily
  // disabled.
  topOn_SWDOff();

  // Check where we read high from the top.
  int numTop = 0;
  // Iterate through the top addresses.
  // TODO(tannewt): Adjust address range based on the number of top pins on the
  // current test top.
  for (uint j = startIndex; j < endIndex; ++j) {
    setAddress(j);
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(TOP_READ_GPIOx, TOP_READ_Pin) == GPIO_PIN_SET) {
      if (numTop == 0) {
        printf("%d", j);
      } else {
        printf(",%d", j);
      }
      numTop++;
    }
  }
  // Breakpoints after this are OK. SWD is enabled.
  topOff_SWDOn();

  // Read direct connections from the top.
  for (uint j = 0; j < topDirect.size(); j++) {
    Input_Z(topDirect[j].first, topDirect[j].second);
    if (HAL_GPIO_ReadPin(topDirect[j].first, topDirect[j].second) == GPIO_PIN_SET) {
      if (numTop == 0) {
        printf("%d", j + 1000);
      } else {
        printf(",%d", j + 1000);
      }
      numTop++;
    }
  }
}

// Do this only with no test board present.
void clearPins(void) {
  // DO NOT set breakpoints between here and the note below. SWD is temporarily
  // disabled.
  topOn_SWDOff();

  Output_High(TOP_READ_GPIOx, TOP_READ_Pin);
  HAL_GPIO_WritePin(TOP_READ_GPIOx, TOP_READ_Pin, GPIO_PIN_RESET);

  // Iterate through the top addresses.
  for (uint j = 0; j < 8 * 16; ++j) {
    setAddress(j);
    HAL_Delay(10);
  }
  Input_Z(TOP_READ_GPIOx, TOP_READ_Pin);

  // Breakpoints after this are OK. SWD is enabled.
  topOff_SWDOn();
}

void heightOn(void) {
  Output_High(HEIGHT_4_GPIOx, HEIGHT_4_Pin);
  Output_High(HEIGHT_2_GPIOx, HEIGHT_2_Pin);
  Output_High(HEIGHT_1_GPIOx, HEIGHT_1_Pin);

  HAL_GPIO_WritePin(HEIGHT_4_GPIOx, HEIGHT_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HEIGHT_2_GPIOx, HEIGHT_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HEIGHT_1_GPIOx, HEIGHT_1_Pin, GPIO_PIN_RESET);
}

void heightOff(void) {
  Input_Z(HEIGHT_4_GPIOx, HEIGHT_4_Pin);
  Input_Z(HEIGHT_2_GPIOx, HEIGHT_2_Pin);
  Input_Z(HEIGHT_1_GPIOx, HEIGHT_1_Pin);
}

void testHeight(void) {
  topOn_SWDOff();

  // Turn on the height pins.
  heightOn();

  bool pass = true;

  GPIO_PinState height4 = GPIO_PIN_RESET;
  GPIO_PinState height2 = GPIO_PIN_RESET;
  GPIO_PinState height1 = GPIO_PIN_RESET;
  for (uint height = 0; height < 7; ++height) {
    // Write out the height.
    HAL_GPIO_WritePin(HEIGHT_4_GPIOx, HEIGHT_4_Pin, height4);
    HAL_GPIO_WritePin(HEIGHT_2_GPIOx, HEIGHT_2_Pin, height2);
    HAL_GPIO_WritePin(HEIGHT_1_GPIOx, HEIGHT_1_Pin, height1);
    HAL_Delay(1);
    uint next_height = height + 1;
    if ((next_height & 0x4) == 0) {
      height4 = GPIO_PIN_RESET;
    } else {
      height4 = GPIO_PIN_SET;
    }
    if ((next_height & 0x2) == 0) {
      height2 = GPIO_PIN_RESET;
    } else {
      height2 = GPIO_PIN_SET;
    }
    if ((next_height & 0x1) == 0) {
      height1 = GPIO_PIN_RESET;
    } else {
      height1 = GPIO_PIN_SET;
    }

    setAddress(HEIGHT_4_Address);
    HAL_Delay(10);
    GPIO_PinState read_height4 = HAL_GPIO_ReadPin(TOP_READ_GPIOx, TOP_READ_Pin);
    setAddress(HEIGHT_2_Address);
    HAL_Delay(10);
    GPIO_PinState read_height2 = HAL_GPIO_ReadPin(TOP_READ_GPIOx, TOP_READ_Pin);
    setAddress(HEIGHT_1_Address);
    HAL_Delay(10);
    GPIO_PinState read_height1 = HAL_GPIO_ReadPin(TOP_READ_GPIOx, TOP_READ_Pin);
    uint read_height = 0;
    if (read_height4 == GPIO_PIN_SET) {
      read_height += 0x4;
    }
    if (read_height2 == GPIO_PIN_SET) {
      read_height += 0x2;
    }
    if (read_height1 == GPIO_PIN_SET) {
      read_height += 0x1;
    }

    if (read_height != next_height) {
      printf("height failed at %d. read %d and wanted %d\n", height, read_height, next_height);
      HAL_Delay(1);
      pass = false;
    }
  }

  topOff_SWDOn();

  heightOff();

  if (pass) {
    printf("heightPass\n");
  } else {
    printf("heightFail\n");
  }
  HAL_Delay(1);
}

void testAddress(void) {
  for (uint i = 0; i < addressPins.size(); ++i) {
    Input_Z(addressPins[i].first, addressPins[i].second);
  }
  HAL_Delay(1);
  for (uint i = 0; i < addressPins.size(); ++i) {
    Output_High(addressPins[i].first, addressPins[i].second);
    HAL_Delay(1);
    printf("a%d ", i);
    int numShorted = 0;
    for (uint j = i + 1; j < addressPins.size(); ++j) {
      if (HAL_GPIO_ReadPin(addressPins[j].first, addressPins[j].second) == GPIO_PIN_SET) {
        if (numShorted == 0) {
          printf("%p_%x", addressPins[j].first, addressPins[j].second);
        } else {
          printf(",%p_%x", addressPins[j].first, addressPins[j].second);
        }
        numShorted++;
      }
    }
    printf("\n");
    Input_Z(addressPins[i].first, addressPins[i].second);
    HAL_Delay(1);
  }
}

void testPower(void) {
  //printf("debug testPower entered\n");
  //HAL_Delay(1);
  // Set everything to inputs.
  for (uint i = 0; i < powerPins.size(); ++i) {
    Input_Z(powerPins[i].first, powerPins[i].second);
  }
  HAL_Delay(1);

  for (uint i = 0; i < powerPins.size(); ++i) {
    Output_High(powerPins[i].first, powerPins[i].second);
    HAL_Delay(1);
    //printf("debug %p_%x started \n", powerPins[i].first, powerPins[i].second);
    printf("%p_%x ", powerPins[i].first, powerPins[i].second);
    int numShorted = 0;
    for (uint j = 0; j < powerPins.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (HAL_GPIO_ReadPin(powerPins[j].first, powerPins[j].second) == GPIO_PIN_SET) {
        if (numShorted == 0) {
          printf("%p_%x", powerPins[j].first, powerPins[j].second);
        } else {
          printf(",%p_%x", powerPins[j].first, powerPins[j].second);
        }
        numShorted++;
      }
    }
    printf(" ");
    //testTop();
    printf("\n");
    Input_Z(powerPins[i].first, powerPins[i].second);
    HAL_Delay(1);
  }
}

void testData(void) {
  // Set everything to input.
  for (uint i = 0; i < dataPins.size(); ++i) {
    Input_Z(dataPins[i].first, dataPins[i].second);
  }
  HAL_Delay(100);

  for (uint i = 0; i < dataPins.size(); ++i) {
    printf("%p_%x ", dataPins[i].first, dataPins[i].second);
    Output_High(dataPins[i].first, dataPins[i].second);
    HAL_Delay(2);

    // Check where we read high from the bottom.
    int numShorted = 0;
    // Iterate through the bottom pins.
    for (uint j = i + 1; j < dataPins.size(); ++j) {
      if (HAL_GPIO_ReadPin(dataPins[j].first, dataPins[j].second) == GPIO_PIN_SET) {
        if (numShorted == 0) {
          printf("%p_%x", dataPins[j].first, dataPins[j].second);
        } else {
          printf(",%p_%x", dataPins[j].first, dataPins[j].second);
        }
        numShorted++;
      }
    }
    printf(" ");
    if (i < 30) {
      testTop(0, 31);
    } else {
      testTop(31, 64);
    }
    printf("\n");
    Input_Z(dataPins[i].first, dataPins[i].second);
    HAL_Delay(2);
  }

  // Deinit all of the data pins so they return to their default state. This
  // includes the SWD pins.
  for (uint i = 0; i < dataPins.size(); ++i) {
    HAL_GPIO_DeInit(dataPins[i].first, dataPins[i].second);
  }
}

void printStatus(HAL_StatusTypeDef status) {
  if (status == HAL_OK) {
    printf("ok\n");
  } else if (status == HAL_ERROR) {
    printf("error\n");
  } else if (status == HAL_BUSY) {
    printf("busy\n");
  } else if (status == HAL_TIMEOUT) {
    printf("timeout\n");
  }
  HAL_Delay(1);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = i2c_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(i2c_SCL_GPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = i2c_SDA_Pin;
    HAL_GPIO_Init(i2c_SDA_GPIOx, &GPIO_InitStruct);

  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_DISABLE();
  }
}

I2C_HandleTypeDef hi2c;

void i2cOn(void) {
  hi2c.Instance = I2C1;
  hi2c.Init.Timing = 0x10808DD3;
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_StatusTypeDef status = HAL_I2C_Init(&hi2c);
  printStatus(status);

    /**Configure Analogue filter
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c, I2C_ANALOGFILTER_ENABLED);
}

void i2cReady(char* line) {
  char command[16];
  uint16_t address;
  //HAL_Delay(10);
  int scan_status = sscanf(line, "%s %" SCNu16 "\n", &command, &address);
  //printf("debug %d %hu address\n", hi2c.State, address);
  //Output_High(BUSY_LED_GPIOx, BUSY_LED_Pin);
  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c, address, 40, 1000);
  //printf("debug state after %x %x\n", hi2c.State, hi2c.ErrorCode);
  //HAL_Delay(1);
  printStatus(status);
  //Input_Z(BUSY_LED_GPIOx, BUSY_LED_Pin);
}

void i2cRead(char* line) {
  static uint8_t readBuffer[32];
  char command[16];
  uint16_t deviceAddress;
  uint16_t memoryAddress;
  uint16_t bytesToRead;
  int scan_status = sscanf(line, "%s %" SCNu16 " %" SCNu16 " %" SCNu16 "\n", &command, &deviceAddress, &memoryAddress, &bytesToRead);

  //printf("debug state %x dAddress %d\n", hi2c.State, deviceAddress);

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c, deviceAddress, memoryAddress, I2C_MEMADD_SIZE_16BIT, readBuffer, bytesToRead, 3000);
  //printf("debug state after %x\n", hi2c.State);
  //HAL_Delay(1);
  printStatus(status);
  //HAL_Delay(1);
  if (status == HAL_OK) {
    for (int i = 0; i < bytesToRead; ++i) {
      printf("%02x ", readBuffer[i]);
      HAL_Delay(1);
    }
    printf("\n");
    HAL_Delay(1);
  }
}

void i2cWrite(char* line) {
  static uint8_t writeBuffer[32];
  char command[16];
  uint16_t deviceAddress;
  uint16_t memoryAddress;
  uint16_t totalBytesToWrite;
  int scan_status = sscanf(line, "%s %" SCNu16 " %" SCNu16 " %" SCNu16 "\n", &command, &deviceAddress, &memoryAddress, &totalBytesToWrite);

  // Tell the computer we're ready for the data to write.
  printf("ok\n");
  HAL_Delay(1);

  uint8_t offset = 0;
  while (offset < totalBytesToWrite) {
    char line[128];
    char* line_status = fgets(line, 128, stdin);
    if (line_status == nullptr) {
      continue;
    }
    int scan_status = sscanf(line, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
      writeBuffer,
      writeBuffer + 1,
      writeBuffer + 2,
      writeBuffer + 3,
      writeBuffer + 4,
      writeBuffer + 5,
      writeBuffer + 6,
      writeBuffer + 7,
      writeBuffer + 8,
      writeBuffer + 9,
      writeBuffer + 10,
      writeBuffer + 11,
      writeBuffer + 12,
      writeBuffer + 13,
      writeBuffer + 14,
      writeBuffer + 15,
      writeBuffer + 16,
      writeBuffer + 17,
      writeBuffer + 18,
      writeBuffer + 19,
      writeBuffer + 20,
      writeBuffer + 21,
      writeBuffer + 22,
      writeBuffer + 23,
      writeBuffer + 24,
      writeBuffer + 25,
      writeBuffer + 26,
      writeBuffer + 27,
      writeBuffer + 28,
      writeBuffer + 29,
      writeBuffer + 30,
      writeBuffer + 31);

    uint16_t bytesToWrite = std::min(32, totalBytesToWrite - offset);
    if (scan_status < bytesToWrite) {
      printf("scanFail %d\n", scan_status);
      HAL_Delay(1);
      return;
    } else {
      //printf("debug writing %02x %02x %02x\n", writeBuffer[0], writeBuffer[1], writeBuffer[2]);
      //HAL_Delay(1);
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c, deviceAddress, memoryAddress + offset, I2C_MEMADD_SIZE_16BIT, writeBuffer, bytesToWrite, 3000);
    printStatus(status);

    if (status != HAL_OK) {
      return;
    }

    status = HAL_BUSY;

    // Wait for the memory to start responding again.
    while (status != HAL_OK) {
      //printf("debug ");
      //printStatus(status);
      status = HAL_I2C_IsDeviceReady(&hi2c, deviceAddress, 40, 1000);
      HAL_Delay(1);
    }

    offset += bytesToWrite;
  }
}

void i2cOff(void) {
  HAL_I2C_DeInit(&hi2c);

  Input_Z(i2c_SCL_GPIOx, i2c_SCL_Pin);
  Input_Z(i2c_SDA_GPIOx, i2c_SDA_Pin);
}

void powerOff(void) {
  for (uint i = 0; i < powerPins.size(); ++i) {
    Input_Z(powerPins[i].first, powerPins[i].second);
  }
  Input_Z(POWER_ENABLE_GPIOx, POWER_ENABLE_Pin);
}

void powerOn(void) {
  for (uint i = 0; i < powerPins.size(); ++i) {
    Input_Z(powerPins[i].first, powerPins[i].second);
  }

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = POWER_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(POWER_FAULT_GPIOx, &GPIO_InitStruct);
  HAL_Delay(10);

  if (HAL_GPIO_ReadPin(POWER_FAULT_GPIOx, POWER_FAULT_Pin) == GPIO_PIN_RESET) {
    printf("powerFaultPrecheckFail\n");
    HAL_Delay(1);
    return;
  }
  Output_High(POWER_ENABLE_GPIOx, POWER_ENABLE_Pin);
  HAL_Delay(200);
  if (HAL_GPIO_ReadPin(POWER_FAULT_GPIOx, POWER_FAULT_Pin) == GPIO_PIN_RESET) {
    Input_Z(POWER_ENABLE_GPIOx, POWER_ENABLE_Pin);
    printf("powerFaultPostcheckFail\n");
  } else {
    printf("powerFaultPostcheckOK\n");
  }
  HAL_Delay(1);
  Input_Z(POWER_FAULT_GPIOx, POWER_FAULT_Pin);
}

void waitForButton(void) {
  Input_Z(GO_GPIOx, GO_Pin);
  while (true) {
    if (HAL_GPIO_ReadPin(GO_GPIOx, GO_Pin) == GPIO_PIN_SET) {
      break;
    }
    HAL_Delay(10);
  }
  while (true) {
    if (HAL_GPIO_ReadPin(GO_GPIOx, GO_Pin) == GPIO_PIN_RESET) {
      break;
    }
    HAL_Delay(10);
  }
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

static UART_HandleTypeDef hUART;
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

uint8_t cBoardReceive(uint8_t* outputBuffer, uint8_t outputBufferSize) {
  uint8_t receiveBuffer[UART_command_length + 1];
  receiveBuffer[0] = MORE_DATA;
  uint8_t totalCopied = 0;
  while (receiveBuffer[0] == MORE_DATA) {
    HAL_StatusTypeDef status = HAL_UART_Receive(&hUART, (unsigned char*) receiveBuffer, UART_command_length + 1, 2000);
    if (status != HAL_OK) {
      printf("uart receive status %d\n", status);
      HAL_Delay(1);
      printStatus(status);
      HAL_Delay(1);
      return COMPLETELY_DONE;
    }
    printf("%02x ", receiveBuffer[0]);
    for (int i = 0; i < UART_command_length; ++i) {
      printf("%02x ", receiveBuffer[i + 1]);
      if (totalCopied < outputBufferSize) {
        outputBuffer[totalCopied++] = receiveBuffer[i + 1];
      }
      HAL_Delay(1);
    }
    printf("\n");
  }
  HAL_Delay(1);
  return receiveBuffer[0];
}

void cBoardTestData(void) {
  uint8_t transmitBuffer[UART_command_length] = {TEST_DATA, 0, 0, 0, 0, 0, 0, 0};
  HAL_StatusTypeDef status = HAL_UART_Transmit(&hUART, (unsigned char*) transmitBuffer, UART_command_length, 100);
  if (status != HAL_OK) {
    printStatus(status);
    return;
  }

  uint8_t cBoardStatus = TEST_TOP_MUX;
  uint8_t readIndex;
  while (cBoardStatus != COMPLETELY_DONE) {
    // Save the first byte because its the index of the top pin that should be high.
    cBoardStatus = cBoardReceive(&readIndex, 1);
    if (cBoardStatus == TEST_TOP_MUX) {
      // DO NOT set breakpoints between here and the note below. SWD is temporarily
      // disabled.
      topOn_SWDOff();

      setAddress(readIndex);
      HAL_Delay(10);
      if (HAL_GPIO_ReadPin(TOP_READ_GPIOx, TOP_READ_Pin) == GPIO_PIN_SET) {
        printf("topPass\n");
      } else {
        printf("topFail ");
        testTop(0, 63);
        printf("\n");
      }
      HAL_Delay(1);
      // Breakpoints after this are OK. SWD is enabled.
      topOff_SWDOn();
    } else if (cBoardStatus == TEST_TOP_DIRECT) {
      Input_Z(topDirect[readIndex].first, topDirect[readIndex].second);
      if (HAL_GPIO_ReadPin(topDirect[readIndex].first, topDirect[readIndex].second) == GPIO_PIN_SET) {
        printf("directPass\n");
      } else {
        printf("directFail\n");
      }
      HAL_Delay(1);
    }
    HAL_Delay(100);
    if (cBoardStatus != COMPLETELY_DONE) {
      // Dummy transmit to tell the board under test to continue.
      transmitBuffer[0] = CONTINUE;
      HAL_UART_Transmit(&hUART, (unsigned char*) transmitBuffer, UART_command_length, 1000);
    }
  }
}

void cBoardCommand(char* line) {
  uint8_t transmitBuffer[UART_command_length] = {0, 0, 0, 0, 0, 0, 0, 0};
  char command[16];
  int scan_status = sscanf(line, "%s %x %x %x %x %x %x %x %x",
    &command,
    transmitBuffer,
    transmitBuffer + 1,
    transmitBuffer + 2,
    transmitBuffer + 3,
    transmitBuffer + 4,
    transmitBuffer + 5,
    transmitBuffer + 6,
    transmitBuffer + 7);
  HAL_StatusTypeDef status = HAL_UART_Transmit(&hUART, (unsigned char*) transmitBuffer, UART_command_length, 100);

  uint8_t receive = cBoardReceive(nullptr, 0);
  while (receive != COMPLETELY_DONE) {
    receive = cBoardReceive(nullptr, 0);
  }
}

void testDeviceId(void) {
  printf("%08lx ", U_ID_0);
  printf("%08lx ", U_ID_1);
  printf("%08lx\n", U_ID_2);
  HAL_Delay(1);
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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(10);

  Input_Z(POWER_ENABLE_GPIOx, POWER_ENABLE_Pin);

  Input_Z(BUSY_LED_GPIOx, BUSY_LED_Pin);
  Input_Z(FAIL_LED_GPIOx, FAIL_LED_Pin);
  Input_Z(PASS_LED_GPIOx, PASS_LED_Pin);

  // Set all data pins as pulled down inputs so they can drain any rogue charge.
  // for (uint i = 0; i < dataPins.size(); ++i) {
  //   Input_Z(dataPins[i].first, dataPins[i].second);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char line[128];
    char command[16];
    char* line_status = fgets(line, 128, stdin);
    if (line_status == nullptr) {
      break;
    }
    int status = sscanf(line, "%16s", command);
    if (status == 1) {
      //printf("debug %s\n", command);
      //HAL_Delay(1);
      if (strcmp(command, "noled") == 0) {
        Input_Z(FAIL_LED_GPIOx, FAIL_LED_Pin);
        Input_Z(PASS_LED_GPIOx, PASS_LED_Pin);
      } else if (strcmp(command, "pass") == 0) {
        Output_High(PASS_LED_GPIOx, PASS_LED_Pin);
        Input_Z(FAIL_LED_GPIOx, FAIL_LED_Pin);
      } else if (strcmp(command, "fail") == 0) {
        Output_High(FAIL_LED_GPIOx, FAIL_LED_Pin);
        Input_Z(PASS_LED_GPIOx, PASS_LED_Pin);
      } else if (strcmp(command, "testAddress") == 0) {
        testAddress();
      } else if (strcmp(command, "testData") == 0) {
        testData();
      } else if (strcmp(command, "testPower") == 0) {
        testPower();
      } else if (strcmp(command, "powerOn") == 0) {
        powerOn();
      } else if (strcmp(command, "powerOff") == 0) {
        powerOff();
      } else if (strcmp(command, "testHeight") == 0) {
        testHeight();
      } else if (strcmp(command, "heightOn") == 0) {
        heightOn();
      } else if (strcmp(command, "heightOff") == 0) {
        heightOff();
      } else if (strcmp(command, "i2cOn") == 0) {
        i2cOn();
      } else if (strcmp(command, "i2cReady") == 0) {
        Output_High(FAIL_LED_GPIOx, FAIL_LED_Pin);
        i2cReady(line);
        Input_Z(FAIL_LED_GPIOx, FAIL_LED_Pin);
      } else if (strcmp(command, "i2cRead") == 0) {
        i2cRead(line);
      } else if (strcmp(command, "i2cWrite") == 0) {
        i2cWrite(line);
      } else if (strcmp(command, "i2cOff") == 0) {
        i2cOff();
      } else if (strcmp(command, "clearPins") == 0) {
        clearPins();
      } else if (strcmp(command, "waitForButton") == 0) {
        waitForButton();
      } else if (strcmp(command, "cBoardCommand") == 0) {
        cBoardCommand(line);
      } else if (strcmp(command, "cBoardCommsOn") == 0) {
        cBoardCommsOn();
      } else if (strcmp(command, "cBoardCommsOff") == 0) {
        cBoardCommsOff();
      } else if (strcmp(command, "cBoardTestData") == 0) {
        cBoardTestData();
      } else if (strcmp(command, "testDeviceId") == 0) {
        testDeviceId();
      } else if (strcmp(command, "readTop") == 0) {
        testTop(0, 63);
        printf("\n");
      } else if (strcmp(command, "resetToBL") == 0) {
        Output_High(RESET_GPIOx, RESET_Pin);
        HAL_GPIO_WritePin(RESET_GPIOx, RESET_Pin, GPIO_PIN_RESET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(RESET_GPIOx, RESET_Pin, GPIO_PIN_SET);
        Input_Z(RESET_GPIOx, RESET_Pin);
      } else if (strcmp(command, "testBuildInfo") == 0) {
        printf("%s %s\n", gitversion, build_date);
      }
      HAL_Delay(5);
      printf("%s done\n", command);
      HAL_Delay(1);
    }
  }

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
  if (GPIOx != RESET_GPIOx || GPIO_Pin != RESET_Pin) {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  } else {
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  }
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
  GPIO_InitStruct.Pin = TOP_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
  HAL_GPIO_Init(TOP_READ_GPIOx, &GPIO_InitStruct);

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
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
