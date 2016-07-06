from __future__ import print_function

import base64
import binascii
import colorama
from colorama import Fore as fore, Style as style
import datetime
import math
import os
import os.path
import serial
import serial.tools.list_ports
import struct
import sys
import time

# Init colorama so it adapts the output for Windows.
colorama.init()

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "Chickadee-tech-Board History-2c3e589ba27e.json"

from gcloud import datastore

import polystack_pb2
from google.protobuf import text_format
from google.protobuf.internal import encoder

import subprocess

import platform

# i2c addresses for the EEPROM
MEMORY_ADDRESS = 0b10100000
SERIAL_ADDRESS = 0b10110000

SERIAL_PAGE = 0b0000100000000000
MEMORY_PAGE = 0b0000000000000000

SHIPPING_BINARIES = {"F3FC": "builds/betaflight_batch1_CKD_F3FC_V9.bin",
               "F4FC": "builds/betaflight_batch1_CKD_F4FC_V5.bin"}

DIRTY_WHITELIST = ["a6652e2-dirty Wed Jun 29 14:05:13 CDT 2016"]

PRINT_LOG = False

PORT = {"F3FC": {"0x48000000": "PA",
            "0x48000400": "PB",
            "0x48000800": "PC",
            "0x48000c00": "PD",
            "0x48001000": "PE",
            "0x48001400": "PF"},
      "F4FC": {"0x40020000": "PA",
            "0x40020400": "PB",
            "0x40020800": "PC",
            "0x40020c00": "PD",
            "0x40021000": "PE",
            "0x40021400": "PF",
            "0x40021800": "PG",
            "0x40021c00": "PH"}}

# Constants for talking with the code on the test control board.
ECHO = 0
TEST_DATA = 1
CONTINUE = 2
TEST_INTERNAL_DATA = 3
READ_UNIQUE_ID = 4
TEST_GYRO = 5

TOP_TO_FS = {
  0: "TIM1",
  1: "TIM2",
  2: "TIM3",
  3: "TIM4",
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
  18: "UART8_RX",
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
  61: "SPI1_MOSI",
  1000: "GPIO2",
  1001: "GPIO1",
  1002: "BOOT0",
  1003: "RESET"
}

PIN_TO_FS = {
  "PD5": "TIM1",
  "PD7": "TIM2",
  "PB4": "TIM3",
  "PB5": "TIM4",
  "PB8": "GPIO1",
  "PB9": "GPIO2",
  "PE0": "GPIO3",
  "PE1": "GPIO4",
  "PE3": "GPIO5",
  "PE6": "GPIO6",
  "PB7": "i2c_SDA",
  "PB6": "i2c_SCL",
  "PC13": "HEIGHT_4",
  "PC14": "HEIGHT_2",
  "PC15": "HEIGHT_1",
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
  "PB3": "BOOT0",
  "PA0": "RESET",
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

BROKEN_PINS = {"ABoAJ0YzVxEgOTEy": ["UART5_TX"]}

EXPECTED_INTERNAL_HIGH = {"F3FC": ["PA12", "PC13", "PC14", "PC15"], "F4FC": ["PC13", "PC14", "PC15"]}
# TODO(tannewt): Add in 8 and 9 to the expected for the F4FC because they are i2c and should read high.
DEFAULT_TOP_STATE = {"F3FC": "8,9,12,13,16,1001,1003", "F4FC": "8,9,12,13,16,1001,1003"}

GYRO_TEST_EXPECTED_09 = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 17, 21]

def fatalError(message):
  print(message)
  if platform.system() == "Windows":
    print("CTRL-C to close window.")
    while True:
      pass
  sys.exit(1)

ACTIVE_LOG = []
def logWarning(msg):
  msg = msg.replace("\n", "\nW: ")
  if PRINT_LOG:
    print("W: " + msg)
  ACTIVE_LOG.append("W: " + msg)

def logError(msg):
  msg = msg.replace("\n", "\nE: ")
  if PRINT_LOG:
    print("E: " + msg)
  ACTIVE_LOG.append("E: " + msg)

def logInfo(msg):
  msg = msg.replace("\n", "\nI: ")
  if PRINT_LOG:
    print("I: " + msg)
  ACTIVE_LOG.append("I: " + msg)

def clearLog():
  global ACTIVE_LOG
  ACTIVE_LOG = []

def getLog():
  return ACTIVE_LOG

def addresses_to_pin(board, addresses):
  if not addresses:
    return ""
  port, pin = addresses.split("_")
  if board not in PORT:
    board = "F3FC"
  return PORT[board][port] + str(int(math.log(int("0x"+pin, 0), 2)))

def top_to_fs(top):
  if int(top) in TOP_TO_FS:
    return TOP_TO_FS[int(top)]
  return top

def pin_to_fs(pin):
  if pin in PIN_TO_FS:
    return PIN_TO_FS[pin]
  return pin

print("Available COM ports:")
comports = serial.tools.list_ports.comports()
if len(comports) == 0:
  fatalError("No COM ports found! Is the jig plugged in?")

for i, port in enumerate(comports):
  print(str(i) + ":", port.device, port.description)
index = raw_input("Enter the number next to the desired COM port: ")
print()
test_device_com_device = comports[int(index)].device
ser = serial.Serial(test_device_com_device, 115200, timeout=5)
print("Connected to:", ser.name)

def runCommand(command):
  logInfo(command)
  ser.write(bytes(command) + b'\n')
  command = command.split(" ")[0]
  responses = []
  while True:
    response = ser.readline().strip("\n")
    if response == "":
      continue
    if response == command + " done":
      break
    #logInfo(response)
    responses.append(response)
  logInfo("")
  return responses

def runTest(command, passResponse = None, msg = None):
  logInfo(command)
  ser.write(bytes(command) + b'\n')
  command = command.split(" ")[0]
  ok = passResponse == None
  while True:
    response = ser.readline().strip("\n")
    if response == "":
      continue
    if response == command + " done":
      break
    ok = ok or response == passResponse
    logInfo(response)
  if not ok and msg:
    logError(msg)
  logInfo("")
  return ok

def i2cRead(deviceAddress, memoryAddress, bytesToRead):
  response = runCommand("i2cRead " + str(deviceAddress) + " " + str(memoryAddress) + " " + str(bytesToRead))
  return response[-1]

def i2cWrite(deviceAddress, memoryAddress, bytesToWrite):
  command = "i2cWrite " + str(deviceAddress) + " " + str(memoryAddress) + " " + str(len(bytesToWrite))
  logInfo(command)
  ser.write(bytes(command) + b'\n')
  command = command.split(" ")[0]
  offset = 0
  while True:
    response = ser.readline().strip("\n")
    if response == "":
      continue
    if response.startswith("debug"):
      logInfo(response)
      continue
    if response == "i2cWrite done":
      break
    if response == "ok":
      bytes_to_this_page = min(32, len(bytesToWrite) - offset)
      if bytes_to_this_page == 0:
        continue

      fullHex = binascii.hexlify(bytesToWrite[offset:offset+bytes_to_this_page])
      spaceSeparatedHex = []
      for i, c in enumerate(fullHex):
        spaceSeparatedHex.append(c)
        if i % 2 == 1:
          spaceSeparatedHex.append(" ")
      spaceSeparatedHex = "".join(spaceSeparatedHex)

      logInfo("writing " + spaceSeparatedHex)

      data = bytes(spaceSeparatedHex) + b'\n'

      #print(data)
      ser.write(data)
      offset += bytes_to_this_page
    else:
      logError("Unknown response: " + response)
      logInfo("")
      return False
  logInfo("")
  return True

PORT_NAME_TO_ATTR = {"TIM": "single_timer_config",
               "GPIO": "gpio_config",
               "UART": "serial_config",
               "TIMG": "timer_group_config",
               "ADC": "adc_config",
               "SPI": "spi_config"}
def dataPinOk(test_jig_id, board_name, board_info, pin_name, shorts, top):
  if test_jig_id in BROKEN_PINS and pin_name in BROKEN_PINS[test_jig_id]:
    logWarning("Skipping test of " + pin_name + " pin.")
    return True

  if (pin_name in ["HEIGHT_1", "HEIGHT_2", "HEIGHT_4"] and not shorts and not top):
    return True

  if len(shorts) > 0 and not pin_name.startswith(("TIM", "GPIO", "UART", "TIMG", "ADC", "SPI")):
    logError(pin_name + " is shorted to " + str(shorts))
    return False
  if pin_name.startswith(("TIM", "GPIO", "UART", "TIMG", "ADC", "SPI")):
    suffix = ""
    prefix = pin_name
    if "_" in pin_name:
      prefix, suffix = pin_name.split("_")
      suffix = "_" + suffix
    input_index = int(prefix[-1:])
    prefix = prefix[:-1]
    shift = len(getattr(board_info, PORT_NAME_TO_ATTR[prefix]))
    if shift >= input_index and not top:
      return True
    output_name = prefix + str(input_index - shift) + suffix

    if board_name == "Receiver Breakout" and pin_name == "TIM1" and shorts == ["UART2_TX", "UART1_TX", "UART1_RX", "TIMG1_CH1", "TIMG2_CH2", "TIMG1_CH3", "TIMG1_CH4", "TIMG2_CH1", "TIMG2_CH2"]:
      return True
    if ((output_name == "UART1_TX" and shorts == ["SPI2_NSS"] and not top) or
       (output_name == "UART1_RX" and shorts == ["SPI2_SCK"] and not top)):
      return True
    if top != [output_name] or shorts:
      print(top, output_name, shorts)
      logError("Bottom " + pin_name + " is connected to incorrect top pin(s): " + str(top))
      return False

  elif top != [pin_name]:
    logError("Bottom " + pin_name + " is connected to incorrect top pin(s): " + str(top))
    return False
  return True

def testDataPins(test_jig_id, board, board_info):
  logInfo("testData")
  ser.write(b'testData\n')
  response = None
  ok = True
  while True:
    response = ser.readline().strip("\n")
    if response == "testData done":
      break
    if response.startswith("debug"):
      logInfo(response)
      continue
    if response.count(" ") != 2:
      logWarning(response)
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
    pin_name = pin_to_fs(addresses_to_pin(board, pin))
    shorts = map(pin_to_fs, map(lambda x: addresses_to_pin(board, x), shorts))
    ok = dataPinOk(test_jig_id, board, board_info, pin_name, shorts, top) and ok
  logInfo("")
  return ok

def cBoardTestData(board):
  logInfo("cBoardTestData")
  command = "cBoardTestData"
  ser.write(bytes(command) + b'\n')
  command = command.split(" ")[0]
  ok = True
  spare_bytes = []
  first_line = True
  pin = None
  high_pins = []
  previous_response = None
  while True:
    response = ser.readline().strip("\n")
    if response == "":
      continue
    if response == command + " done":
      break
    if response.count(" ") == 9:
      b = response.strip().split()
      if first_line:
        pin = addresses_to_pin(board, "0x" + "".join(b[1+1:1+1+4]) + "_" + "".join(b[1+1+4:1+1+4+2]))
        #print(b[1])
        spare_bytes.extend(b[1+4+2+1:])
      else:
        spare_bytes.extend(b[1:])
        while len(spare_bytes) >= 6:
          high_pins.append(addresses_to_pin(board, "0x" + "".join(spare_bytes[:4]) + "_" + "".join(spare_bytes[4:4+2])))
          spare_bytes = spare_bytes[6:]
      if b[0] == "00":
        first_line = False
    elif response == "timeout":
      logError("Serial failed to board under test.")
      ok = False
    else:
      if high_pins:
        logError(pin + " is shorted to " + str(high_pins))
        ok = False
      if response not in ["topPass", "directPass"]:
        logError(pin + " isn't connect to the top: " + response)
        ok = False
      spare_bytes = []
      high_pins = []
      first_line = True

  logInfo("")
  return ok

def cBoardTestInternalData(board):
  lines = runCommand("cBoardCommand " + str(TEST_INTERNAL_DATA) + " 00 00 00 00 00 00 00")

  ok = True
  spare_bytes = []
  first_line = True
  very_first = True
  pin = None
  high_pins = []
  for line in lines:
    b = line.strip().split()
    if first_line:
      pin = addresses_to_pin(board, "0x" + "".join(b[1:4+1]) + "_" + "".join(b[1+4:1+4+2]))
      #print(b[1])
      spare_bytes.extend(b[1+4+2:])
    else:
      spare_bytes.extend(b[1:])
      while len(spare_bytes) >= 6:
        port = "".join(spare_bytes[:4])
        high_pin = "".join(spare_bytes[4:4+2])
        spare_bytes = spare_bytes[6:]
        if port == "00000000":
          continue
        high_pin = addresses_to_pin(board, "0x" + port + "_" + high_pin)
        if very_first or high_pin not in EXPECTED_INTERNAL_HIGH[board]:
          high_pins.append(high_pin)
    if b[0] == "00":
      first_line = False
    else:
      if very_first:
        for p in EXPECTED_INTERNAL_HIGH[board]:
          if p not in high_pins:
            logError("Expected " + p + " to be high.")
            ok = False
          else:
            high_pins.remove(p)
        very_first = False
      if high_pins:
        logError(pin + " shorted to: " + ", ".join(high_pins))
        ok = False
      pin = None
      high_pins = []
      spare_bytes = []
      first_line = True
  return ok


def cBoardTestGyro():
  lines = runCommand("cBoardCommand " + str(TEST_GYRO) + " 00 00 00 00 00 00 00")
  receivedBytes = []
  ok = True
  for line in lines:
    receivedBytes.extend(line.strip().split()[1:])

  for pos in GYRO_TEST_EXPECTED_09:
    if receivedBytes[pos] != "09":
      ok = False
      logError("Position " + str(pos) + " not 09 as expected to indicate successful write to gyro.")
  who_am_i = receivedBytes[1]
  if who_am_i != "68":
    ok = False
    logError("Wrong WHO_AM_I. Gyro probably not inited right.")
  if ok:
    product_id = receivedBytes[2]
    logInfo("Product id: " + product_id)
    for axis in xrange(0,3):
      error = int(receivedBytes[14 + axis], 16)
      logInfo("Accel axis " + str(axis) + " error is " + str(error) + "%")
      if error >= 14:
        logError("Accel axis " + str(axis) + " error is too high!")
        ok = False
    for axis in xrange(0,3):
      error = int(receivedBytes[18 + axis], 16)
      logInfo("Gyro axis " + str(axis) + " error is " + str(error) + "%")
      if error >= 14:
        logError("Gyro axis " + str(axis) + " error is too high!")
        ok = False

  if not ok:
    logInfo("Gyro response: " + str(receivedBytes))
  logInfo("")
  return ok

def flash(filename):
  logInfo("Flashing " + filename)
  st_flash = ["st-flash", "write", filename, " 0x8000000"]
  if platform.system() == "Windows":
    st_flash = ["ST-LINK_CLI.exe", "-c", "SWD", "UR", "LPM", "-Q", "-ME", "-P", filename, " 0x8000000", "-V" , "after_programming", "-HardRst"]

  p = subprocess.Popen(st_flash, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  out, err = p.communicate()
  logInfo(out)
  if err:
    logError(err)
  returncode = p.poll()
  logInfo("Return code: " + str(returncode))
  return returncode

all_boards = []
board_filenames = os.listdir("board_info")
for i, board in enumerate(board_filenames):
  if i < len(board_filenames) - 1 and board.split("-v")[0] == board_filenames[i+1].split("-v")[0]:
    continue
  board_info_fn = os.path.join("board_info", board)
  #print(board_info_fn)
  board_info = polystack_pb2.PolystackMod()
  if os.path.isfile(board_info_fn):
    with open(board_info_fn) as f:
      text_format.Merge(f.read(), board_info)
  all_boards.append(board_info)

for i, board in enumerate(all_boards):
  print(str(i) + ":", board.mod_info.mod_url)
index = raw_input("Enter the number next to the URL that matches the URL on the boards to test: ")
board_info = all_boards[int(index)]
board = board_info.mod_info.mod_name

if board not in ["F3FC", "F4FC"] and board_info.mod_info.manufacturer_id == 0:
  print("manufacturer_id must not be zero for manufactured boards.")
  sys.exit(-1)

print()
print("Set up to test", board_info.mod_info.mod_name, "version", board_info.mod_info.mod_version, "with url", board_info.mod_info.mod_url)

build_info = runCommand("testBuildInfo")[-1]
print("Test jig build:", build_info)

if build_info not in DIRTY_WHITELIST and "dirty" in build_info and "--test" not in sys.argv:
  fatalError("Test jig contains experimental code(" + build_info + "). Please contact scott@chickadee.tech for instructions on how to update it to production code. Thanks!")

testDeviceId = runCommand("testDeviceId")[-1]
testDeviceIdB64 = base64.urlsafe_b64encode(binascii.unhexlify(testDeviceId.replace(" ", "")))
print("Test device id:", testDeviceIdB64)

for part in map(binascii.unhexlify, testDeviceId.split()):
  board_info.manufacturing_info.test_device_id.append(struct.unpack("<L", part)[0])

google_client = datastore.Client("chickadee-tech-board-history")

testing = True
while testing:
  print()
  print("Press GO to test next board. (Ctrl-C to exit.)")
  runCommand("waitForButton")
  print("Testing in progress")

  clearLog()
  ok = True
  noFatal = True

  startTime = time.time()
  serial_number = "unknown"
  runCommand("noled")

  if board not in ["F3FC", "F4FC"]:
    ok = testDataPins(testDeviceIdB64, board, board_info) and ok

  powerOk = runTest("powerOn", "powerFaultPostcheckOK", "Failed to turn on power.")
  ok = ok and powerOk

  if powerOk:
    if board not in ["F3FC", "F4FC"]:
      ok = runTest("testHeight", "heightPass") and ok

      runCommand("heightOn")

      runCommand("i2cOn")

      ok = runTest("i2cReady " + str(SERIAL_ADDRESS), "ok", "Serial number not ready. I2C lines probably hosed.") and ok
      ok = runTest("i2cReady " + str(MEMORY_ADDRESS), "ok", "Memory address not ready. I2C lines probably hosed.") and ok

      serial_number = i2cRead(SERIAL_ADDRESS, SERIAL_PAGE, 16)
      if serial_number not in ["error", "timeout", "busy"]:
        serial_number = base64.urlsafe_b64encode(binascii.unhexlify(serial_number.replace(" ", ""))).strip("=")

        # Set manufacturing info into the board info.
        board_info.manufacturing_info.test_time = long(startTime)
        serialized_board_info = board_info.SerializeToString()
        delimiter = encoder._VarintBytes(len(serialized_board_info))

        ok = i2cWrite(MEMORY_ADDRESS + 1, MEMORY_PAGE, delimiter + serialized_board_info) and ok
      else:
        logError("Unable to read EEPROM serial number!")
        ok = False

      runCommand("i2cOff")

      runCommand("heightOff")
    else:
      # Wait for bootup.
      time.sleep(0.5)
      testBin = "builds/" + board + "/test.bin"
      returncode = flash(testBin)
      if returncode != 0:
        logWarning("Initial flash failed. Has this board been tested before?")
        runCommand("resetToBL")
        returncode = flash(testBin)
        if returncode != 0:
          logError("Second flash attempt failed. Bad board.")
          noFatal = False

      # Cold restart
      runCommand("powerOff")

      time.sleep(0.5)

      topResponse = None
      if noFatal:
        runCommand("powerOn")
        time.sleep(0.5)

        # Read the top pins. These should always be high: ['i2c_SDA', 'i2c_SCL', 'HEIGHT_1', '3V3_0.3A_LL', '5V', 'GPIO1', 'RESET']. GPIO1 is in SWD state by default.
        topResponse = runCommand("readTop")
        if len(topResponse) == 0:
          ok = False
          noFatal = False
          logError("No response from control board.")

      # Read the unique device id.
      if noFatal:
        runCommand("cBoardCommsOn")
        responses = runCommand("cBoardCommand " + str(READ_UNIQUE_ID) + " 00 00 00 00 00 00 00")
        if len(responses) == 0:
          ok = False
          noFatal = False
          logError("No serial number response from board under test.")

        serial_number = responses[0][3:] + responses[1][3:14]
        serial_number = base64.urlsafe_b64encode(binascii.unhexlify(serial_number.replace(" ", ""))).strip("=")

      if topResponse and topResponse[0] != DEFAULT_TOP_STATE[board]:
        ok = False
        logInfo(", ".join([top_to_fs(x) + " (" + str(x) +")" for x in topResponse[0].split(",")]))
        logError("Default pin state not as expected. Is the 3.3v regulator working?")

      if noFatal:
        # Test the local pins for shorts.
        time.sleep(0.1)
        ok = cBoardTestInternalData(board) and ok

        # TODO(tannewt): Test the current and batt pins.

        # Test the gyro.
        time.sleep(0.1)
        ok = cBoardTestGyro() and ok

        # Test the connections to the polystack connector.
        time.sleep(0.1)
        ok = cBoardTestData(board) and ok
        #time.sleep(4)
        time.sleep(0.1)
        #runCommand("cBoardCommand " + str(ECHO) + " ff 02 fe 03 fd 04 fc")
        #runCommand("cBoardCommand " + str(ECHO) + " ff 03 fe 03 fd 04 fc")
        #runCommand("cBoardCommand " + str(ECHO) + " ff 04 fe 03 fd 04 fc")
        runCommand("cBoardCommsOff")

      # Flash shipping firmware.
      if ok and noFatal:
        returncode = flash(SHIPPING_BINARIES[board])
        if returncode != 0:
          logError("Final flash failed. Not sure why.")
          ok = False
      else:
        logInfo("Firmware flash skipped because board failed.")

      if ok and noFatal:
        runCommand("powerOff")
        print("Please remove the control board from the jig and plug it into the computer directly.")
        controlDevice = None
        for tryNum in xrange(180):
          comports = serial.tools.list_ports.comports()
          if len(comports) > 1:
            for comport in comports:
              if comport.device == test_device_com_device:
                continue
              if comport.pid == 22336 and comport.vid == 1155:
                controlDevice = comport.device
                break
          if controlDevice:
            break
          time.sleep(0.1)

        # Validate that MSP CLI works and record the version info.
        if controlDevice:
          time.sleep(0.1)
          s = serial.Serial(controlDevice, 115200, timeout=5)
          s.write("#")
          if s.readline() != "\r\n":
            logError("Invalid response after #")
            ok = False
          elif not s.readline().startswith("Entering"):
            logError("Failed to enter CLI")
            ok = False
          elif s.readline() != "\r\n":
            logError("No newline after entering")
            ok = False
          elif s.read(2) != "# ":
              logError("No prompt")
              ok = False
          s.write("version\r\n")
          if s.readline() != "version\r\n":
            logError("version not echoed")
            ok = False
          else:
            versionString = s.readline()
            logInfo(versionString)
            if "BetaFlight/CKD" not in versionString:
              logError("Flashed incorrect firmware")
              ok = False
          s.close()
        else:
          logError("Unable to find control board when connected directly to the computer.")
          ok = False

    runCommand("powerOff")

  if ok and noFatal:
    runCommand("pass")
    print(fore.GREEN +
"""########     ###     ######   ######
##     ##   ## ##   ##    ## ##    ##
##     ##  ##   ##  ##       ##
########  ##     ##  ######   ######
##        #########       ##       ##
##        ##     ## ##    ## ##    ##
##        ##     ##  ######   ######  """ + style.RESET_ALL)
  else:
    runCommand("fail")
    print(fore.RED + """########    ###    #### ##
##         ## ##    ##  ##
##        ##   ##   ##  ##
######   ##     ##  ##  ##
##       #########  ##  ##
##       ##     ##  ##  ##
##       ##     ## #### ######## """ + style.RESET_ALL)
  print()


  testDuration = time.time() - startTime
  logInfo(str(testDuration) + " second duration")
  print(str(testDuration) + " second test duration")

  print("Saving test results to cloud.")
  saveStartTime = time.time()
  parent_key = google_client.key("Board", unicode(serial_number))
  boardInfo = datastore.Entity(key=parent_key)
  boardInfo.update({"board_id": unicode(serial_number)})
  google_client.put(boardInfo)

  key_name = unicode(str(long(startTime)) + "/" + testDeviceIdB64)
  key = google_client.key("TestRun", key_name, parent=parent_key)
  runInfo = datastore.Entity(key=key, exclude_from_indexes=['log'])
  runInfo.update(
    {
     "test_device_id": unicode(testDeviceIdB64),
     "test_time": datetime.datetime.fromtimestamp(startTime),
     "test_duration_sec": testDuration,
     "test_jig_build_info": unicode(build_info),
     "pass": ok and noFatal,
     "board_name": unicode(board),
     "log": unicode("\n".join(getLog()))})

  google_client.put(runInfo)
  print("Cloud save took",time.time() - saveStartTime,"seconds.")
  print(key_name)

  # getLog()

response = ser.readline()
print(response)
ser.close()
