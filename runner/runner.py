from __future__ import print_function

import math
import serial
import sys
import time

import subprocess

# i2c addresses for the EEPROM
MEMORY_ADDRESS = 0b10100001
SERIAL_ADDRESS = 0b10110001

PORT = {"0x48000000": "PA",
        "0x48000400": "PB",
        "0x48000800": "PC",
        "0x48000c00": "PD",
        "0x48001000": "PE",
        "0x48001400": "PF"}

TOP_TO_FS = {
  0: "TIM2",
  1: "TIM3",
  2: "TIM4",
  3: "GPIO2",
  4: "GPIO3",
  5: "GPIO4",
  6: "GPIO5",
  7: "GPIO6",
  8: "i2c_SDA",
  9: "i2c_SCL",
  10: "HEIGHT_4",
  11: "HEIGHT_2",
  12: "HEIGHT_1",
  13: "3V3_0.3A_LL",
  14: "3V3_0.3A_E",
  15: "+BATT",
  16: "5V",
  17: "UART8_TX",
  18: "UART7_RX",
  19: "UART7_TX",
  20: "UART7_RX",
  21: "UART6_TX",
  22: "UART6_RX",
  23: "UART5_TX",
  24: "UART5_RX",
  25: "UART4_TX",
  26: "UART4_RX",
  27: "UART3_TX",
  28: "UART3_RX",
  29: "UART2_TX",
  30: "UART2_RX",
  31: "TIMG1_CH1",
  32: "TIMG1_CH2",
  33: "TIMG1_CH3",
  34: "TIMG1_CH4",
  35: "TIMG2_CH1",
  36: "TIMG2_CH2",
  37: "TIMG2_CH3",
  38: "TIMG2_CH4",
  39: "ADC1",
  40: "ADC2",
  41: "SDMMC1_D0",
  42: "SDMMC1_D1",
  43: "SDMMC1_D2",
  44: "SDMMC1_D3",
  45: "SDMMC1_CK",
  46: "SDMMC1_CMD",
  47: "GND",
  48: "CAN_HI",
  49: "CAN_LO",
  50: "SPI3_NSS",
  51: "SPI3_SCK",
  52: "SPI3_MISO",
  53: "SPI3_MOSI",
  54: "SPI2_NSS",
  55: "SPI2_SCK",
  56: "SPI2_MISO",
  57: "SPI2_MOSI",
  58: "SPI1_NSS",
  59: "SPI1_SCK",
  60: "SPI1_MISO",
  61: "SPI1_MOSI"
}

PIN_TO_FS = {
  "PD5": "TIM1",
  "PD7": "TIM2",
  "PB4": "TIM3",
  "PB5": "TIM4",
  "PB8": "GPIO1",
  "PB9": "GPIO2",
  "PE6": "GPIO3",
  "PC13": "GPIO4",
  "PC14": "GPIO5",
  "PC15": "GPIO6",
  "PB7": "i2c_SDA",
  "PB6": "i2c_SCL",
  "PF0": "HEIGHT_4",
  "PF1": "HEIGHT_2",
  "PF9": "HEIGHT_1",
  "PC0": "3V3_0.3A_LL",
  "PC1": "3V3_0.3A_E",
  "PC2": "+BATT",
  "PF10": "5V",
  "PC3": "UART8_TX",
  "PF2": "UART8_RX",
  "PA1": "UART7_TX",
  "PA3": "UART7_RX",
  "PA2": "UART6_TX",
  "PF4": "UART6_RX",
  "PA5": "UART5_TX",
  "PA4": "UART5_RX",
  "PA7": "UART4_TX",
  "PA6": "UART4_RX",
  "PC5": "UART3_TX",
  "PC4": "UART3_RX",
  "PB0": "UART2_TX",
  "PB1": "UART2_RX",
  "PB2": "UART1_TX",
  "PE7": "UART1_RX",

  "PD6": "TIMG1_CH1",
  "PD3": "TIMG1_CH2",
  "PD4": "TIMG1_CH3",
  "PD1": "TIMG1_CH4",
  "PD2": "TIMG2_CH1",
  "PC12": "TIMG2_CH2",
  "PD0": "TIMG2_CH3",
  "PC10": "TIMG2_CH4",
  "PC11": "ADC1",
  "PA14": "ADC2",
  "PA15": "SDMMC1_D0",
  "PF6": "SDMMC1_D1",
  "PA10": "SDMMC1_D2",
  "PA9": "SDMMC1_D3",
  "PA8": "SDMMC1_CK",
  "PC9": "SDMMC1_CMD",
  "PC8": "GND",
  "PC6": "BOOT0",
  "PC7": "RESET",
  "PD15": "CAN_HI",
  "PD14": "CAN_LO",
  "PD13": "SPI3_NSS",
  "PD12": "SPI3_SCK",
  "PD11": "SPI3_MISO",
  "PD10": "SPI3_MOSI",
  "PD9": "SPI2_NSS",
  "PD8": "SPI2_SCK",
  "PB15": "SPI2_MISO",
  "PB14": "SPI2_MOSI",
  "PB13": "SPI1_NSS",
  "PB12": "SPI1_SCK",
  "PB11": "SPI1_MISO",
  "PB10": "SPI1_MOSI"
}

TOP_IGNORE = ["i2c_SDA", "i2c_SCL", "HEIGHT_2", "HEIGHT_1", "3V3_0.3A_LL", "5V"]

def addresses_to_pin(addresses):
  if not addresses:
    return ""
  port, pin = addresses.split("_")
  return PORT[port] + str(int(math.log(int("0x"+pin, 0), 2)))

def top_to_fs(top):
  if int(top) in TOP_TO_FS:
    return TOP_TO_FS[int(top)]
  return top

def pin_to_fs(pin):
  if pin in PIN_TO_FS:
    return PIN_TO_FS[pin]
  return pin

ser = serial.Serial(sys.argv[1], 115200, timeout=5)
print(ser.name)

def runCommand(command):
  print(command)
  ser.write(bytes(command) + b'\n')
  command = command.split(" ")[0]
  while True:
    response = ser.readline().strip("\n")
    print(response)
    if response == command + " done":
      break
  print()

def i2cRead(deviceAddress, memoryAddress, bytesToRead):
  runCommand("i2cRead " + str(deviceAddress) + " " + str(memoryAddress) + " " + str(bytesToRead))

runCommand("noled")

if sys.argv[2] not in ["F3FC", "F4FC"]:
  ser.write(b'testData\n')
  response = None
  while True:
    response = ser.readline().strip("\n")
    if response == "testData done":
      break
    if response.startswith("debug"):
      print(response)
      continue
    if response.count(" ") > 2:
      print(response)
      continue
    pin, shorts, top = response.split(" ")
    if shorts == "":
      shorts = []
    else:
      shorts = shorts.split(",")
    if top == "":
      top = []
    else:
      top = top.split(",")
    top = map(top_to_fs, top)
    top = [x for x in top if x not in TOP_IGNORE]
    print(pin_to_fs(addresses_to_pin(pin)), map(pin_to_fs, map(addresses_to_pin, shorts)), top)

  print()

OK_SHORTS = {"3V3_0.3A_LL": ["5V", "GND"], "5V": ["3V3_0.3A_LL", "GND"], "GND": ["3V3_0.3A_LL", "5V"]}
okToPower = True
print("testPower")
ser.write(b'testPower\n')
response = None
while True:
  response = ser.readline().strip("\n")
  if response == "testPower done":
    break
  if response.startswith("debug"):
    print(response)
    continue
  if response == "":
    print("empty")
    continue
  if response.count(" ") > 2:
    print(response)
    continue
  pin, shorts, top = response.split(" ")
  pin = pin_to_fs(addresses_to_pin(pin))
  if shorts == "":
    shorts = []
  else:
    shorts = shorts.split(",")
  shorts = map(pin_to_fs, map(addresses_to_pin, shorts))
  if top == "":
    top = []
  else:
    top = top.split(",")
  top = map(top_to_fs, top)
  top = [x for x in top if x not in TOP_IGNORE]
  print(pin, shorts, top)
  if pin not in OK_SHORTS or any([short not in OK_SHORTS[pin] for short in shorts]):
    okToPower = False

if okToPower:
  runCommand("powerOn")

  time.sleep(1);

  #runCommand("testHeight")

  runCommand("heightOn")

  runCommand("i2cOn")

  runCommand("i2cReady " + str(MEMORY_ADDRESS))

  i2cRead(MEMORY_ADDRESS, 0, 16)

  runCommand("i2cOff")

  subprocess.call(["st-util"], stdout=subprocess.PIPE)

  runCommand("powerOff")

if okToPower:
  ser.write(b'pass\n')
else:
  ser.write(b'fail\n')
response = ser.readline()
print(response)
ser.close()
