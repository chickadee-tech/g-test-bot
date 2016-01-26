EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ckd_sandwich
LIBS:ti-gate
LIBS:generic-bottom-test-jig-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 7050 7000 0    197  ~ 0
generic-bottom-test-jig
$Comp
L CKD_SANDWICH PORT_OUT1
U 1 1 56A16422
P 8900 1750
F 0 "PORT_OUT1" H 8900 -2400 60  0000 C CNN
F 1 "CKD_SANDWICH" H 8900 -2300 60  0000 C CNN
F 2 "hirose-df40:DF40-3-4mm-80pin-Receptacle" H 8900 -2450 60  0001 C CNN
F 3 "" H 8900 -2450 60  0000 C CNN
	1    8900 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3600 8200 4100
Connection ~ 8200 4000
Connection ~ 8200 3900
Connection ~ 8200 3800
Connection ~ 8200 3700
Wire Wire Line
	9600 3400 9600 4100
Connection ~ 9600 3500
Connection ~ 9600 3600
Connection ~ 9600 3700
Connection ~ 9600 3800
Connection ~ 9600 3900
Connection ~ 9600 4000
$Comp
L CONN_02X25 P2
U 1 1 56A164BD
P 1950 4250
F 0 "P2" H 1950 5550 50  0000 C CNN
F 1 "CONN_02X25" V 1950 4250 50  0000 C CNN
F 2 "" H 1950 3500 50  0000 C CNN
F 3 "" H 1950 3500 50  0000 C CNN
	1    1950 4250
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X25 P4
U 1 1 56A164F9
P 4250 4250
F 0 "P4" H 4250 5550 50  0000 C CNN
F 1 "CONN_02X25" V 4250 4250 50  0000 C CNN
F 2 "" H 4250 3500 50  0000 C CNN
F 3 "" H 4250 3500 50  0000 C CNN
	1    4250 4250
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X10 P6
U 1 1 56A1651D
P 6200 3800
F 0 "P6" H 6200 4350 50  0000 C CNN
F 1 "CONN_02X10" V 6200 3800 50  0000 C CNN
F 2 "" H 6200 2600 50  0000 C CNN
F 3 "" H 6200 2600 50  0000 C CNN
	1    6200 3800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P1
U 1 1 56A16543
P 1700 1700
F 0 "P1" H 1700 2050 50  0000 C CNN
F 1 "CONN_01X06" V 1800 1700 50  0000 C CNN
F 2 "" H 1700 1700 50  0000 C CNN
F 3 "" H 1700 1700 50  0000 C CNN
	1    1700 1700
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P3
U 1 1 56A16571
P 3850 1650
F 0 "P3" H 3850 1900 50  0000 C CNN
F 1 "CONN_01X04" V 3950 1650 50  0000 C CNN
F 2 "" H 3850 1650 50  0000 C CNN
F 3 "" H 3850 1650 50  0000 C CNN
	1    3850 1650
	1    0    0    -1  
$EndComp
NoConn ~ 2200 3150
$Comp
L GNDPWR #PWR03
U 1 1 56A166ED
P 4250 5600
F 0 "#PWR03" H 4250 5400 50  0001 C CNN
F 1 "GNDPWR" H 4250 5470 50  0000 C CNN
F 2 "" H 4250 5550 50  0000 C CNN
F 3 "" H 4250 5550 50  0000 C CNN
	1    4250 5600
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR04
U 1 1 56A16713
P 1950 5600
F 0 "#PWR04" H 1950 5400 50  0001 C CNN
F 1 "GNDPWR" H 1950 5470 50  0000 C CNN
F 2 "" H 1950 5550 50  0000 C CNN
F 3 "" H 1950 5550 50  0000 C CNN
	1    1950 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 5450 4000 5600
Wire Wire Line
	4000 5600 4500 5600
Wire Wire Line
	4500 5600 4500 5450
Connection ~ 4250 5600
Wire Wire Line
	1700 5450 1700 5600
Wire Wire Line
	1700 5600 2200 5600
Wire Wire Line
	2200 5600 2200 5450
Connection ~ 1950 5600
Text Label 2200 4250 0    60   ~ 0
PE8_LD4_BLUE
Text Label 2200 4350 0    60   ~ 0
PE10_LD5_ORANGE
Text Label 2200 4450 0    60   ~ 0
PE12_LD9_BLUE
Text Label 2200 4550 0    60   ~ 0
PE14_LD8_ORANGE
NoConn ~ 4000 3950
NoConn ~ 2200 3550
Text Label 2200 3650 0    60   ~ 0
PA2
Text Label 2200 3750 0    60   ~ 0
PA4
Text Label 4500 5150 0    60   ~ 0
PA9
NoConn ~ 4500 5050
NoConn ~ 4000 5050
Text Label 4500 4950 0    60   ~ 0
PA13_SWDIO
Text Label 4000 4850 2    60   ~ 0
PA14_SWCLK
Text Label 4500 4850 0    60   ~ 0
PA15
Text Label 2200 4050 0    60   ~ 0
PB0
Text Label 1700 4050 2    60   ~ 0
PB1
Text Label 2200 4150 0    60   ~ 0
PB2
Text Label 4500 4250 0    60   ~ 0
PB3_SWO
Text Label 4000 4150 2    60   ~ 0
PB4
Text Label 4500 4150 0    60   ~ 0
PB5
Text Label 4000 4050 2    60   ~ 0
PB6_SCL
Text Label 4500 4050 0    60   ~ 0
PB7_SDA
Text Label 4000 3850 2    60   ~ 0
PB8
Text Label 4500 3850 0    60   ~ 0
PB9
Text Label 2200 4650 0    60   ~ 0
PB10
Text Label 1700 4650 2    60   ~ 0
PB11
Text Label 2200 4750 0    60   ~ 0
PB12
Text Label 1700 4750 2    60   ~ 0
PB13
Text Label 2200 4850 0    60   ~ 0
PB14
Text Label 1700 4850 2    60   ~ 0
PB15
Text Label 2200 3250 0    60   ~ 0
PC0_3V3
Text Label 1700 3250 2    60   ~ 0
PC1_3V3
Text Label 2200 3350 0    60   ~ 0
PC2_BATT
Text Label 1700 3350 2    60   ~ 0
PC3
Text Label 2200 3950 0    60   ~ 0
PC4
Text Label 1700 3950 2    60   ~ 0
PC5
Text Label 1700 5350 2    60   ~ 0
PC6_BOOT0
Text Label 2200 5350 0    60   ~ 0
PC7_RESET
Text Label 4000 5350 2    60   ~ 0
PC8_GND
Text Label 4500 5250 0    60   ~ 0
PC9
Text Label 4000 4750 2    60   ~ 0
PC10
Text Label 4500 4750 0    60   ~ 0
PC11
Text Label 4000 4650 2    60   ~ 0
PC12
Text Label 4500 3450 0    60   ~ 0
PC13
Text Label 4500 4650 0    60   ~ 0
PD0
Text Label 4000 4550 2    60   ~ 0
PD1
Text Label 4500 4550 0    60   ~ 0
PD2
Text Label 4000 4450 2    60   ~ 0
PD3
Text Label 4500 4450 0    60   ~ 0
PD4
Text Label 4000 4350 2    60   ~ 0
PD5
Text Label 4500 4350 0    60   ~ 0
PD6
Text Label 4000 4250 2    60   ~ 0
PD7
Text Label 2200 4950 0    60   ~ 0
PD8_TX
Text Label 1700 4950 2    60   ~ 0
PD9_RX
Text Label 2200 5050 0    60   ~ 0
PD10
Text Label 1700 5050 2    60   ~ 0
PD11
Text Label 2200 5150 0    60   ~ 0
PD12
Text Label 1700 5150 2    60   ~ 0
PD13
Text Label 2200 5250 0    60   ~ 0
PD14
Text Label 1700 5250 2    60   ~ 0
PD15
NoConn ~ 4000 3750
NoConn ~ 4500 3750
NoConn ~ 4000 3650
NoConn ~ 4500 3650
NoConn ~ 4000 3550
NoConn ~ 4500 3550
Text Label 4000 3450 2    60   ~ 0
PE6
Text Label 1700 4150 2    60   ~ 0
PE7
Text Label 1700 4250 2    60   ~ 0
PE9_LD3_RED
Text Label 1700 4350 2    60   ~ 0
PE11_LD7_GREEN
Text Label 1700 4450 2    60   ~ 0
PE13_LD10_RED
Text Label 1700 4550 2    60   ~ 0
PE15_LD6_GREEN
Text Label 2200 3450 0    60   ~ 0
PF2
Text Label 1700 3650 2    60   ~ 0
PF4
Text Label 4000 4950 2    60   ~ 0
PF6
Text Label 4000 3150 2    60   ~ 0
PF9
Text Label 4500 3150 0    60   ~ 0
PF10_5V
NoConn ~ 4500 5350
$Comp
L GNDPWR #PWR05
U 1 1 56A16EAC
P 1250 3150
F 0 "#PWR05" H 1250 2950 50  0001 C CNN
F 1 "GNDPWR" H 1250 3020 50  0000 C CNN
F 2 "" H 1250 3100 50  0000 C CNN
F 3 "" H 1250 3100 50  0000 C CNN
	1    1250 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3150 1700 3150
Text Label 1700 3450 2    60   ~ 0
PA1
Text Label 1700 3550 2    60   ~ 0
PA3
Text Label 2200 3850 0    60   ~ 0
PA6_SDO
Text Label 1700 3850 2    60   ~ 0
PA7_SDA
Text Label 1700 3750 2    60   ~ 0
PA5_SCL
Text Label 4000 5250 2    60   ~ 0
PA8
Text Label 4000 5150 2    60   ~ 0
PA10
Text Label 9600 5700 0    60   ~ 0
PB10
Text Label 9600 5600 0    60   ~ 0
PB11
Text Label 9600 5500 0    60   ~ 0
PB12
Text Label 9600 5400 0    60   ~ 0
PB13
Text Label 9600 5300 0    60   ~ 0
PB14
Text Label 9600 5200 0    60   ~ 0
PB15
Text Label 9600 5100 0    60   ~ 0
PD8_TX
Text Label 9600 5000 0    60   ~ 0
PD9_RX
Text Label 9600 4900 0    60   ~ 0
PD10
Text Label 9600 4800 0    60   ~ 0
PD11
Text Label 9600 4700 0    60   ~ 0
PD12
Text Label 9600 4600 0    60   ~ 0
PD13
Text Label 9600 4500 0    60   ~ 0
PD14
Text Label 9600 4400 0    60   ~ 0
PD15
Text Label 9600 4300 0    60   ~ 0
PC7_RESET
Text Label 9600 4200 0    60   ~ 0
PC6_BOOT0
Text Label 8200 5700 2    60   ~ 0
PE7
Text Label 8200 5600 2    60   ~ 0
PB2
Text Label 8200 5500 2    60   ~ 0
PB1
Text Label 8200 5400 2    60   ~ 0
PB0
Text Label 8200 5300 2    60   ~ 0
PC4
Text Label 8200 5200 2    60   ~ 0
PC5
Text Label 8200 4900 2    60   ~ 0
PA4
Text Label 8200 4700 2    60   ~ 0
PF4
Text Label 8200 4600 2    60   ~ 0
PA2
Text Label 8200 4500 2    60   ~ 0
PA3
Text Label 8200 4300 2    60   ~ 0
PF2
Text Label 8200 4400 2    60   ~ 0
PA1
Text Label 8200 4200 2    60   ~ 0
PC3
Text Label 8200 3500 2    60   ~ 0
PC2_BATT
Text Label 8200 3400 2    60   ~ 0
PC1_3V3_E
Text Label 8200 3300 2    60   ~ 0
PC0_3V3
Text Label 8000 3850 2    60   ~ 0
PF10_5V
Text Label 8200 3200 2    60   ~ 0
PF9
Text Label 8200 2500 2    60   ~ 0
PC13
Text Label 8200 2400 2    60   ~ 0
PE6
Text Label 8200 2300 2    60   ~ 0
PB9
Text Label 8200 2900 2    60   ~ 0
PB6_SCL
Text Label 8200 2800 2    60   ~ 0
PB7_SDA
Text Label 8200 2200 2    60   ~ 0
PB8
Text Label 9800 3800 0    60   ~ 0
PC8_GND
Text Label 9600 3300 0    60   ~ 0
PC9
Text Label 9600 3200 0    60   ~ 0
PA8
Text Label 9600 3100 0    60   ~ 0
PA9
Text Label 9600 3000 0    60   ~ 0
PA10
Text Label 9600 2900 0    60   ~ 0
PF6
Text Label 9600 2800 0    60   ~ 0
PA15
Text Label 9600 2600 0    60   ~ 0
PC11
Text Label 9600 2500 0    60   ~ 0
PC10
Text Label 9600 2400 0    60   ~ 0
PD0
Text Label 9600 2300 0    60   ~ 0
PC12
Text Label 9600 2200 0    60   ~ 0
PD2
Text Label 9600 2100 0    60   ~ 0
PD1
Text Label 9600 2000 0    60   ~ 0
PD4
Text Label 9600 1900 0    60   ~ 0
PD3
Text Label 9600 1800 0    60   ~ 0
PD6
Text Label 8200 2100 2    60   ~ 0
PB5
Text Label 8200 2000 2    60   ~ 0
PB4
Text Label 8200 1800 2    60   ~ 0
PD5
Text Label 8200 1900 2    60   ~ 0
PD7
NoConn ~ 4500 3950
$Comp
L CONN_01X16 P5
U 1 1 56A175BB
P 6050 1800
F 0 "P5" H 6050 2650 50  0000 C CNN
F 1 "CONN_01X16" V 6150 1800 50  0000 C CNN
F 2 "" H 6050 1800 50  0000 C CNN
F 3 "" H 6050 1800 50  0000 C CNN
	1    6050 1800
	1    0    0    -1  
$EndComp
Text Label 4000 3350 2    60   ~ 0
PC14
Text Label 4500 3350 0    60   ~ 0
PC15
Text Label 8200 2700 2    60   ~ 0
PC15
Text Label 8200 2600 2    60   ~ 0
PC14
Text Label 4500 3250 0    60   ~ 0
PF1
Text Label 4000 3250 2    60   ~ 0
PF0
Text Label 8200 3100 2    60   ~ 0
PF1
Text Label 8200 3000 2    60   ~ 0
PF0
Text Label 8200 5100 2    60   ~ 0
PA6_SDO
Text Label 8200 5000 2    60   ~ 0
PA7_SDA
Text Label 8200 4800 2    60   ~ 0
PA5_SCL
Wire Wire Line
	8000 3850 8200 3850
Connection ~ 8200 3850
Text Label 9600 2700 0    60   ~ 0
PA14_SWCLK
Wire Wire Line
	9600 3800 9800 3800
Text Label 6650 3750 0    60   ~ 0
PC8_GND
Text Label 5800 3750 2    60   ~ 0
PF10_5V
Text Label 5650 4250 2    60   ~ 0
PC2_BATT
Wire Wire Line
	6450 4250 6450 3350
Connection ~ 6450 4150
Connection ~ 6450 4050
Connection ~ 6450 3950
Connection ~ 6450 3850
Connection ~ 6450 3750
Connection ~ 6450 3650
Connection ~ 6450 3550
Connection ~ 6450 3450
Wire Wire Line
	6450 3750 6650 3750
Connection ~ 5950 3550
Connection ~ 5950 3650
Connection ~ 5950 3750
Connection ~ 5950 3850
Connection ~ 5950 3950
Connection ~ 5950 4050
Wire Wire Line
	5950 3750 5800 3750
Wire Wire Line
	5950 3350 5950 4150
Connection ~ 5950 3450
Wire Wire Line
	5950 4250 5650 4250
$Comp
L +5V #PWR06
U 1 1 56A18D6B
P 5450 1050
F 0 "#PWR06" H 5450 900 50  0001 C CNN
F 1 "+5V" H 5450 1190 50  0000 C CNN
F 2 "" H 5450 1050 50  0000 C CNN
F 3 "" H 5450 1050 50  0000 C CNN
	1    5450 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 1050 5850 1050
$Comp
L GNDPWR #PWR07
U 1 1 56A18E42
P 5550 2550
F 0 "#PWR07" H 5550 2350 50  0001 C CNN
F 1 "GNDPWR" H 5550 2420 50  0000 C CNN
F 2 "" H 5550 2500 50  0000 C CNN
F 3 "" H 5550 2500 50  0000 C CNN
	1    5550 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2550 5850 2550
Text Label 5850 1850 2    60   ~ 0
PA13_SWDIO
Text Label 5850 1750 2    60   ~ 0
PE14_LD8_ORANGE
Text Label 5850 1650 2    60   ~ 0
PE15_LD6_GREEN
Text Label 5850 1550 2    60   ~ 0
PE8_LD4_BLUE
Text Label 5850 1450 2    60   ~ 0
PE9_LD3_RED
Text Label 5850 1350 2    60   ~ 0
PE10_LD5_ORANGE
Text Label 5850 1250 2    60   ~ 0
PE11_LD7_GREEN
Text Label 5850 1150 2    60   ~ 0
PE12_LD9_BLUE
Text Label 5850 2350 2    60   ~ 0
PD8_TX
Text Label 5850 2450 2    60   ~ 0
PD9_RX
Text Label 5850 1950 2    60   ~ 0
PC6_BOOT0
Text Label 5850 2050 2    60   ~ 0
PC7_RESET
Text Label 1500 1550 2    60   ~ 0
SWCLK
Text Label 1500 1750 2    60   ~ 0
SWDIO
NoConn ~ 1500 1450
NoConn ~ 1500 1950
$Comp
L GNDPWR #PWR08
U 1 1 56A192FC
P 800 1650
F 0 "#PWR08" H 800 1450 50  0001 C CNN
F 1 "GNDPWR" H 800 1520 50  0000 C CNN
F 2 "" H 800 1600 50  0000 C CNN
F 3 "" H 800 1600 50  0000 C CNN
	1    800  1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  1650 1500 1650
$Comp
L Switch_SPDT_x2 SW1
U 1 1 56A19414
P 3100 1600
F 0 "SW1" H 2900 1750 50  0000 C CNN
F 1 "Switch_SPDT_x2" H 2850 1450 50  0000 C CNN
F 2 "" H 3100 1600 50  0000 C CNN
F 3 "" H 3100 1600 50  0000 C CNN
	1    3100 1600
	1    0    0    -1  
$EndComp
$Comp
L Switch_SPDT_x2 SW1
U 2 1 56A19441
P 3100 2000
F 0 "SW1" H 2900 2150 50  0000 C CNN
F 1 "Switch_SPDT_x2" H 2850 1850 50  0000 C CNN
F 2 "" H 3100 2000 50  0000 C CNN
F 3 "" H 3100 2000 50  0000 C CNN
	2    3100 2000
	1    0    0    -1  
$EndComp
NoConn ~ 3400 1700
NoConn ~ 3400 2100
Wire Wire Line
	3650 1600 3500 1600
Wire Wire Line
	3500 1600 3500 1800
Wire Wire Line
	3500 1800 2800 1800
Wire Wire Line
	2800 1800 2800 1600
Wire Wire Line
	3400 1500 3650 1500
Wire Wire Line
	3650 1700 3550 1700
Wire Wire Line
	3550 1700 3550 1900
Wire Wire Line
	3550 1900 3400 1900
Wire Wire Line
	3650 1800 3650 2200
Wire Wire Line
	3650 2200 2800 2200
Wire Wire Line
	2800 2200 2800 2000
Text Label 1500 1850 2    60   ~ 0
PC7_RESET
$Comp
L TLV70233DBV U?
U 1 1 56A5DFB9
P 4250 6550
F 0 "U?" H 4000 6750 50  0000 C CNN
F 1 "TLV70233DBV" H 4350 6750 50  0000 C CNN
F 2 "SOT-23-5" H 4250 6650 50  0000 C CIN
F 3 "" H 4250 6550 50  0000 C CNN
	1    4250 6550
	1    0    0    -1  
$EndComp
Text Label 4700 6500 0    60   ~ 0
PC0_3V3
$Comp
L GNDPWR #PWR?
U 1 1 56A5E0FC
P 4250 6850
F 0 "#PWR?" H 4250 6650 50  0001 C CNN
F 1 "GNDPWR" H 4250 6720 50  0000 C CNN
F 2 "" H 4250 6800 50  0000 C CNN
F 3 "" H 4250 6800 50  0000 C CNN
	1    4250 6850
	1    0    0    -1  
$EndComp
Text Label 3800 6650 2    60   ~ 0
PE9_LD3_RED
NoConn ~ 1700 3050
NoConn ~ 2200 3050
NoConn ~ 4000 3050
NoConn ~ 4500 3050
$Comp
L +5V #PWR?
U 1 1 56A5E512
P 3700 6300
F 0 "#PWR?" H 3700 6150 50  0001 C CNN
F 1 "+5V" H 3700 6440 50  0000 C CNN
F 2 "" H 3700 6300 50  0000 C CNN
F 3 "" H 3700 6300 50  0000 C CNN
	1    3700 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 6300 3700 6500
Wire Wire Line
	3700 6500 3800 6500
$Comp
L Q_NMOS_GSD Q?
U 1 1 56A5E6C0
P 6200 5450
F 0 "Q?" H 6500 5500 50  0000 R CNN
F 1 "Q_NMOS_GSD" H 6850 5400 50  0000 R CNN
F 2 "" H 6400 5550 50  0000 C CNN
F 3 "" H 6200 5450 50  0000 C CNN
	1    6200 5450
	1    0    0    -1  
$EndComp
Text Label 6000 5450 2    60   ~ 0
PE9_LD3_RED
Text Label 6300 5000 0    60   ~ 0
PC8_GND
$Comp
L GNDPWR #PWR?
U 1 1 56A5E760
P 6300 5800
F 0 "#PWR?" H 6300 5600 50  0001 C CNN
F 1 "GNDPWR" H 6300 5670 50  0000 C CNN
F 2 "" H 6300 5750 50  0000 C CNN
F 3 "" H 6300 5750 50  0000 C CNN
	1    6300 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 5000 6300 5250
Wire Wire Line
	6300 5650 6300 5800
$EndSCHEMATC
