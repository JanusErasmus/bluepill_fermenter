EESchema Schematic File Version 4
LIBS:bluepill_fermenter-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title "Bluepill UPS"
Date "2018-12-14"
Rev "324234"
Comp "Janus"
Comment1 "Overview of the UPS"
Comment2 "tw"
Comment3 "3"
Comment4 "4"
$EndDescr
$Sheet
S 2750 2550 1600 1850
U 5C14BAF9
F0 "Bluepill Node" 50
F1 "node.sch" 50
F2 "ADC_CH0" I R 4350 3500 50 
F3 "ADC_CH1" I R 4350 3600 50 
F4 "UART1_RX" I R 4350 2650 50 
F5 "UART1_TX" O R 4350 2750 50 
F6 "I2C1_SCL" B R 4350 4150 50 
F7 "I2C1_SDA" B R 4350 4250 50 
F8 "PB8" O R 4350 3200 50 
F9 "PB9" O R 4350 3300 50 
$EndSheet
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5C14C190
P 1700 2850
F 0 "J1" H 1620 2525 50  0000 C CNN
F 1 "Conn_01x02" H 1620 2616 50  0000 C CNN
F 2 "SIP2" H 1700 2850 50  0001 C CNN
F 3 "~" H 1700 2850 50  0001 C CNN
	1    1700 2850
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0109
U 1 1 5C14C23B
P 2150 2400
F 0 "#PWR0109" H 2150 2250 50  0001 C CNN
F 1 "+5V" H 2165 2573 50  0000 C CNN
F 2 "" H 2150 2400 50  0001 C CNN
F 3 "" H 2150 2400 50  0001 C CNN
	1    2150 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2400 2150 2850
$Comp
L power:GNDREF #PWR0110
U 1 1 5C14C26C
P 2150 3100
F 0 "#PWR0110" H 2150 2850 50  0001 C CNN
F 1 "GNDREF" H 2155 2927 50  0001 C CNN
F 2 "" H 2150 3100 50  0001 C CNN
F 3 "" H 2150 3100 50  0001 C CNN
	1    2150 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2950 2150 3100
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5C14C2D0
P 2300 2800
F 0 "#FLG0101" H 2300 2875 50  0001 C CNN
F 1 "PWR_FLAG" H 2300 2974 50  0001 C CNN
F 2 "" H 2300 2800 50  0001 C CNN
F 3 "~" H 2300 2800 50  0001 C CNN
	1    2300 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2800 2300 2850
Wire Wire Line
	2300 2850 2150 2850
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5C14C309
P 2300 3100
F 0 "#FLG0102" H 2300 3175 50  0001 C CNN
F 1 "PWR_FLAG" H 2300 3274 50  0001 C CNN
F 2 "" H 2300 3100 50  0001 C CNN
F 3 "~" H 2300 3100 50  0001 C CNN
	1    2300 3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 3100 2300 2950
Wire Wire Line
	2300 2950 2150 2950
Wire Wire Line
	1900 2850 2150 2850
Connection ~ 2150 2850
Wire Wire Line
	2150 2950 1900 2950
Connection ~ 2150 2950
Text Label 5300 3200 2    50   ~ 0
COOLER_CONTROL
Text Label 5300 3300 2    50   ~ 0
HEATER_CONTROL
Text Label 4900 3500 2    50   ~ 0
TEMP0
Text Label 4900 3600 2    50   ~ 0
TEMP1
$Comp
L Device:R R7
U 1 1 5C14A963
P 7300 4150
F 0 "R7" V 7200 4150 50  0000 C CNN
F 1 "10K" V 7400 4150 50  0000 C CNN
F 2 "" V 7230 4150 50  0001 C CNN
F 3 "~" H 7300 4150 50  0001 C CNN
	1    7300 4150
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5C14AA7C
P 6650 2500
F 0 "R1" V 6443 2500 50  0000 C CNN
F 1 "1K" V 6534 2500 50  0000 C CNN
F 2 "" V 6580 2500 50  0001 C CNN
F 3 "~" H 6650 2500 50  0001 C CNN
	1    6650 2500
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:2N7000 Q1
U 1 1 5C14AAFD
P 7300 2500
F 0 "Q1" H 7505 2546 50  0000 L CNN
F 1 "2N7000" H 7505 2455 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7500 2425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7300 2500 50  0001 L CNN
	1    7300 2500
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q2
U 1 1 5C14AB5F
P 7300 3300
F 0 "Q2" H 7505 3346 50  0000 L CNN
F 1 "2N7000" H 7505 3255 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7500 3225 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 7300 3300 50  0001 L CNN
	1    7300 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 2700 7400 2900
Wire Wire Line
	7400 3500 7400 3700
Wire Wire Line
	7100 2500 6950 2500
$Comp
L Device:R R2
U 1 1 5C14ADB8
P 6650 3300
F 0 "R2" V 6443 3300 50  0000 C CNN
F 1 "1K" V 6534 3300 50  0000 C CNN
F 2 "" V 6580 3300 50  0001 C CNN
F 3 "~" H 6650 3300 50  0001 C CNN
	1    6650 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 3300 6950 3300
Wire Wire Line
	4350 3300 6500 3300
Wire Wire Line
	6350 3200 6350 2500
Wire Wire Line
	6350 2500 6500 2500
Wire Wire Line
	4350 3200 6350 3200
$Comp
L power:GNDPWR #PWR0101
U 1 1 5C14BE97
P 7400 2900
F 0 "#PWR0101" H 7400 2700 50  0001 C CNN
F 1 "GNDPWR" H 7404 2746 50  0001 C CNN
F 2 "" H 7400 2850 50  0001 C CNN
F 3 "" H 7400 2850 50  0001 C CNN
	1    7400 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0103
U 1 1 5C14BED7
P 7400 3700
F 0 "#PWR0103" H 7400 3500 50  0001 C CNN
F 1 "GNDPWR" H 7404 3546 50  0001 C CNN
F 2 "" H 7400 3650 50  0001 C CNN
F 3 "" H 7400 3650 50  0001 C CNN
	1    7400 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5C14C196
P 6950 3500
F 0 "R4" H 6880 3454 50  0000 R CNN
F 1 "10K" H 6880 3545 50  0000 R CNN
F 2 "" V 6880 3500 50  0001 C CNN
F 3 "~" H 6950 3500 50  0001 C CNN
	1    6950 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5C14C1D6
P 6950 2700
F 0 "R3" H 6880 2654 50  0000 R CNN
F 1 "10K" H 6880 2745 50  0000 R CNN
F 2 "" V 6880 2700 50  0001 C CNN
F 3 "~" H 6950 2700 50  0001 C CNN
	1    6950 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GNDPWR #PWR0104
U 1 1 5C14C212
P 6950 3700
F 0 "#PWR0104" H 6950 3500 50  0001 C CNN
F 1 "GNDPWR" H 6954 3546 50  0001 C CNN
F 2 "" H 6950 3650 50  0001 C CNN
F 3 "" H 6950 3650 50  0001 C CNN
	1    6950 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0105
U 1 1 5C14C22D
P 6950 2900
F 0 "#PWR0105" H 6950 2700 50  0001 C CNN
F 1 "GNDPWR" H 6954 2746 50  0001 C CNN
F 2 "" H 6950 2850 50  0001 C CNN
F 3 "" H 6950 2850 50  0001 C CNN
	1    6950 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2850 6950 2900
Wire Wire Line
	6950 3650 6950 3700
Wire Wire Line
	6950 2550 6950 2500
Connection ~ 6950 2500
Wire Wire Line
	6950 2500 6800 2500
Wire Wire Line
	6950 3350 6950 3300
Connection ~ 6950 3300
Wire Wire Line
	6950 3300 7100 3300
Wire Wire Line
	7400 3100 7400 3050
Wire Wire Line
	7400 3050 7750 3050
Wire Wire Line
	7400 2300 7400 2250
Wire Wire Line
	7400 2250 7750 2250
$Comp
L Device:R R5
U 1 1 5C14E157
P 6950 4350
F 0 "R5" H 6880 4304 50  0000 R CNN
F 1 "10K" H 6880 4395 50  0000 R CNN
F 2 "" V 6880 4350 50  0001 C CNN
F 3 "~" H 6950 4350 50  0001 C CNN
	1    6950 4350
	-1   0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 5C14E257
P 7300 4750
F 0 "R8" V 7200 4750 50  0000 C CNN
F 1 "10K" V 7400 4750 50  0000 C CNN
F 2 "" V 7230 4750 50  0001 C CNN
F 3 "~" H 7300 4750 50  0001 C CNN
	1    7300 4750
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5C14E25E
P 6950 4950
F 0 "R6" H 6880 4904 50  0000 R CNN
F 1 "10K" H 6880 4995 50  0000 R CNN
F 2 "" V 6880 4950 50  0001 C CNN
F 3 "~" H 6950 4950 50  0001 C CNN
	1    6950 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 4150 8100 4150
$Comp
L power:GNDREF #PWR0111
U 1 1 5C14ED3D
P 6950 5150
F 0 "#PWR0111" H 6950 4900 50  0001 C CNN
F 1 "GNDREF" H 6955 4977 50  0001 C CNN
F 2 "" H 6950 5150 50  0001 C CNN
F 3 "" H 6950 5150 50  0001 C CNN
	1    6950 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0112
U 1 1 5C14ED5E
P 6950 4550
F 0 "#PWR0112" H 6950 4300 50  0001 C CNN
F 1 "GNDREF" H 6955 4377 50  0001 C CNN
F 2 "" H 6950 4550 50  0001 C CNN
F 3 "" H 6950 4550 50  0001 C CNN
	1    6950 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4150 6950 4150
Wire Wire Line
	6950 4150 6950 4200
Wire Wire Line
	6950 4150 6500 4150
Wire Wire Line
	6350 4150 6350 3500
Wire Wire Line
	4350 3500 6350 3500
Connection ~ 6950 4150
Wire Wire Line
	6250 3600 6250 4750
Wire Wire Line
	6250 4750 6500 4750
Wire Wire Line
	4350 3600 6250 3600
Wire Wire Line
	6950 4800 6950 4750
Connection ~ 6950 4750
Wire Wire Line
	6950 4750 7150 4750
Wire Wire Line
	6950 5100 6950 5150
Wire Wire Line
	6950 4550 6950 4500
$Comp
L Device:R R12
U 1 1 5C151145
P 8150 2850
F 0 "R12" V 7943 2850 50  0000 C CNN
F 1 "470R" V 8034 2850 50  0000 C CNN
F 2 "" V 8080 2850 50  0001 C CNN
F 3 "~" H 8150 2850 50  0001 C CNN
	1    8150 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5C1511A3
P 8250 1850
F 0 "R11" V 8043 1850 50  0000 C CNN
F 1 "470R" V 8134 1850 50  0000 C CNN
F 2 "" V 8180 1850 50  0001 C CNN
F 3 "~" H 8250 1850 50  0001 C CNN
	1    8250 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 1850 7750 1850
Wire Wire Line
	7750 1850 7750 2250
Connection ~ 7750 2250
Wire Wire Line
	8000 2850 7750 2850
Wire Wire Line
	7750 2850 7750 3050
Connection ~ 7750 3050
Text Label 8750 2250 0    50   ~ 0
COOLER_OUTPUT
Text Label 8750 3050 0    50   ~ 0
HEATER_OUTPUT
Text Label 9600 1950 0    50   ~ 0
HEATER_LED
Text Label 9600 1850 0    50   ~ 0
COOLER_LED
Text Label 8750 4150 0    50   ~ 0
COOLER_INPUT
Text Label 8750 4750 0    50   ~ 0
HEATER_INPUT
$Comp
L Device:R R10
U 1 1 5C15382B
P 8100 3850
F 0 "R10" H 8030 3804 50  0000 R CNN
F 1 "10K" H 8030 3895 50  0000 R CNN
F 2 "" V 8030 3850 50  0001 C CNN
F 3 "~" H 8100 3850 50  0001 C CNN
	1    8100 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8100 4000 8100 4150
Connection ~ 8100 4150
$Comp
L power:+5V #PWR0113
U 1 1 5C1540C1
P 8100 3600
F 0 "#PWR0113" H 8100 3450 50  0001 C CNN
F 1 "+5V" H 8115 3773 50  0000 C CNN
F 2 "" H 8100 3600 50  0001 C CNN
F 3 "" H 8100 3600 50  0001 C CNN
	1    8100 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3600 8100 3700
$Comp
L power:+5V #PWR0114
U 1 1 5C1549C6
P 9650 1500
F 0 "#PWR0114" H 9650 1350 50  0001 C CNN
F 1 "+5V" H 9665 1673 50  0000 C CNN
F 2 "" H 9650 1500 50  0001 C CNN
F 3 "" H 9650 1500 50  0001 C CNN
	1    9650 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 1500 9650 1750
Text Label 9700 1750 0    50   ~ 0
LED_VCC
$Comp
L Device:R R9
U 1 1 5C155D82
P 7800 5600
F 0 "R9" V 7593 5600 50  0000 C CNN
F 1 "50R" V 7684 5600 50  0000 C CNN
F 2 "" V 7730 5600 50  0001 C CNN
F 3 "~" H 7800 5600 50  0001 C CNN
	1    7800 5600
	0    1    1    0   
$EndComp
$Comp
L power:GNDPWR #PWR0115
U 1 1 5C155DF6
P 8100 6000
F 0 "#PWR0115" H 8100 5800 50  0001 C CNN
F 1 "GNDPWR" H 8104 5846 50  0001 C CNN
F 2 "" H 8100 5950 50  0001 C CNN
F 3 "" H 8100 5950 50  0001 C CNN
	1    8100 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0116
U 1 1 5C155E1F
P 7550 6000
F 0 "#PWR0116" H 7550 5750 50  0001 C CNN
F 1 "GNDREF" H 7555 5827 50  0001 C CNN
F 2 "" H 7550 6000 50  0001 C CNN
F 3 "" H 7550 6000 50  0001 C CNN
	1    7550 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5C157407
P 8350 5800
F 0 "C4" H 8465 5846 50  0000 L CNN
F 1 "100nF" H 8465 5755 50  0000 L CNN
F 2 "" H 8388 5650 50  0001 C CNN
F 3 "~" H 8350 5800 50  0001 C CNN
	1    8350 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5C1574E9
P 7100 5750
F 0 "C3" H 7215 5796 50  0000 L CNN
F 1 "100nF" H 7215 5705 50  0000 L CNN
F 2 "" H 7138 5600 50  0001 C CNN
F 3 "~" H 7100 5750 50  0001 C CNN
	1    7100 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5600 8100 6000
Wire Wire Line
	7550 5600 7550 6000
$Comp
L power:GNDREF #PWR0117
U 1 1 5C15946D
P 7100 6000
F 0 "#PWR0117" H 7100 5750 50  0001 C CNN
F 1 "GNDREF" H 7105 5827 50  0001 C CNN
F 2 "" H 7100 6000 50  0001 C CNN
F 3 "" H 7100 6000 50  0001 C CNN
	1    7100 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0118
U 1 1 5C1594AB
P 8350 6000
F 0 "#PWR0118" H 8350 5800 50  0001 C CNN
F 1 "GNDPWR" H 8354 5846 50  0001 C CNN
F 2 "" H 8350 5950 50  0001 C CNN
F 3 "" H 8350 5950 50  0001 C CNN
	1    8350 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 5950 8350 6000
Wire Wire Line
	8350 5650 8350 5600
Wire Wire Line
	8350 5600 8100 5600
Wire Wire Line
	8100 5600 7950 5600
Connection ~ 8100 5600
Wire Wire Line
	7650 5600 7550 5600
Wire Wire Line
	7100 5600 7550 5600
Connection ~ 7550 5600
Wire Wire Line
	7100 5900 7100 6000
$Comp
L Device:C C1
U 1 1 5C15F5D6
P 6500 4350
F 0 "C1" H 6615 4396 50  0000 L CNN
F 1 "100nF" H 6615 4305 50  0000 L CNN
F 2 "" H 6538 4200 50  0001 C CNN
F 3 "~" H 6500 4350 50  0001 C CNN
	1    6500 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5C15F64E
P 6500 4950
F 0 "C2" H 6615 4996 50  0000 L CNN
F 1 "100nF" H 6615 4905 50  0000 L CNN
F 2 "" H 6538 4800 50  0001 C CNN
F 3 "~" H 6500 4950 50  0001 C CNN
	1    6500 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0119
U 1 1 5C160478
P 6500 4550
F 0 "#PWR0119" H 6500 4300 50  0001 C CNN
F 1 "GNDREF" H 6505 4377 50  0001 C CNN
F 2 "" H 6500 4550 50  0001 C CNN
F 3 "" H 6500 4550 50  0001 C CNN
	1    6500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0120
U 1 1 5C1604A9
P 6500 5150
F 0 "#PWR0120" H 6500 4900 50  0001 C CNN
F 1 "GNDREF" H 6505 4977 50  0001 C CNN
F 2 "" H 6500 5150 50  0001 C CNN
F 3 "" H 6500 5150 50  0001 C CNN
	1    6500 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5150 6500 5100
Wire Wire Line
	6500 4550 6500 4500
Wire Wire Line
	6500 4200 6500 4150
Connection ~ 6500 4150
Wire Wire Line
	6500 4150 6350 4150
Wire Wire Line
	9650 3100 10050 3100
Wire Wire Line
	9500 2250 9500 3200
Wire Wire Line
	9500 3200 10050 3200
Wire Wire Line
	7750 2250 9500 2250
Wire Wire Line
	9350 3050 9350 3300
Wire Wire Line
	9350 3300 10050 3300
Wire Wire Line
	7750 3050 9350 3050
Wire Wire Line
	10050 3400 9150 3400
$Comp
L power:GNDPWR #PWR0121
U 1 1 5C167874
P 9000 3400
F 0 "#PWR0121" H 9000 3200 50  0001 C CNN
F 1 "GNDPWR" H 9004 3246 50  0001 C CNN
F 2 "" H 9000 3350 50  0001 C CNN
F 3 "" H 9000 3350 50  0001 C CNN
	1    9000 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	10050 3500 9350 3500
Wire Wire Line
	9350 3500 9350 4150
Wire Wire Line
	8100 4150 9350 4150
Wire Wire Line
	10050 3600 9500 3600
Wire Wire Line
	9500 3600 9500 4750
Wire Wire Line
	7450 4750 9500 4750
$Comp
L Connector_Generic:Conn_01x07 J2
U 1 1 5C16A29D
P 10250 3400
F 0 "J2" H 10330 3442 50  0000 L CNN
F 1 "Conn_01x07" H 10330 3351 50  0000 L CNN
F 2 "SIP7" H 10250 3400 50  0001 C CNN
F 3 "~" H 10250 3400 50  0001 C CNN
	1    10250 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR0122
U 1 1 5C16A333
P 9900 3900
F 0 "#PWR0122" H 9900 3650 50  0001 C CNN
F 1 "GNDREF" H 9905 3727 50  0001 C CNN
F 2 "" H 9900 3900 50  0001 C CNN
F 3 "" H 9900 3900 50  0001 C CNN
	1    9900 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 3700 9900 3700
Wire Wire Line
	9900 3700 9900 3900
Wire Wire Line
	6500 4750 6500 4800
Connection ~ 6500 4750
Wire Wire Line
	6500 4750 6950 4750
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5C16E17F
P 9150 3250
F 0 "#FLG0103" H 9150 3325 50  0001 C CNN
F 1 "PWR_FLAG" H 9150 3424 50  0001 C CNN
F 2 "" H 9150 3250 50  0001 C CNN
F 3 "~" H 9150 3250 50  0001 C CNN
	1    9150 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3250 9150 3400
Connection ~ 9150 3400
Wire Wire Line
	9150 3400 9000 3400
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5C171E62
P 10300 1850
F 0 "J3" H 10380 1892 50  0000 L CNN
F 1 "Conn_01x03" H 10380 1801 50  0000 L CNN
F 2 "" H 10300 1850 50  0001 C CNN
F 3 "~" H 10300 1850 50  0001 C CNN
	1    10300 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 2850 8450 1950
Wire Wire Line
	8450 1950 10100 1950
Wire Wire Line
	8300 2850 8450 2850
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5C177171
P 5350 2550
F 0 "J4" H 5430 2542 50  0000 L CNN
F 1 "Conn_01x04" H 5430 2451 50  0000 L CNN
F 2 "SIP4" H 5350 2550 50  0001 C CNN
F 3 "~" H 5350 2550 50  0001 C CNN
	1    5350 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2750 4350 2750
Wire Wire Line
	5150 2650 4350 2650
$Comp
L power:GNDREF #PWR01
U 1 1 5C17ABCC
P 4850 2550
F 0 "#PWR01" H 4850 2300 50  0001 C CNN
F 1 "GNDREF" H 4855 2377 50  0001 C CNN
F 2 "" H 4850 2550 50  0001 C CNN
F 3 "" H 4850 2550 50  0001 C CNN
	1    4850 2550
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR02
U 1 1 5C182515
P 5000 2250
F 0 "#PWR02" H 5000 2100 50  0001 C CNN
F 1 "+3V3" H 5015 2423 50  0000 C CNN
F 2 "" H 5000 2250 50  0001 C CNN
F 3 "" H 5000 2250 50  0001 C CNN
	1    5000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2250 5000 2450
Wire Wire Line
	5000 2450 5150 2450
Wire Wire Line
	5150 2550 4850 2550
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 5C1867AB
P 5300 4050
F 0 "J5" H 5380 4042 50  0000 L CNN
F 1 "Conn_01x04" H 5380 3951 50  0000 L CNN
F 2 "SIP4" H 5300 4050 50  0001 C CNN
F 3 "~" H 5300 4050 50  0001 C CNN
	1    5300 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR03
U 1 1 5C18A966
P 4850 4050
F 0 "#PWR03" H 4850 3800 50  0001 C CNN
F 1 "GNDREF" H 4855 3877 50  0001 C CNN
F 2 "" H 4850 4050 50  0001 C CNN
F 3 "" H 4850 4050 50  0001 C CNN
	1    4850 4050
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR04
U 1 1 5C18A9FA
P 5000 3850
F 0 "#PWR04" H 5000 3700 50  0001 C CNN
F 1 "+3V3" H 5015 4023 50  0000 C CNN
F 2 "" H 5000 3850 50  0001 C CNN
F 3 "" H 5000 3850 50  0001 C CNN
	1    5000 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 3850 5000 3950
Wire Wire Line
	5000 3950 5100 3950
Wire Wire Line
	5100 4050 4850 4050
Wire Wire Line
	4350 4150 5100 4150
Wire Wire Line
	4350 4250 5100 4250
Wire Wire Line
	9650 1750 10100 1750
$Comp
L power:+5V #PWR?
U 1 1 5C1965D5
P 9650 2850
F 0 "#PWR?" H 9650 2700 50  0001 C CNN
F 1 "+5V" H 9665 3023 50  0000 C CNN
F 2 "" H 9650 2850 50  0001 C CNN
F 3 "" H 9650 2850 50  0001 C CNN
	1    9650 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 2850 9650 3100
Wire Wire Line
	8400 1850 10100 1850
$EndSCHEMATC
