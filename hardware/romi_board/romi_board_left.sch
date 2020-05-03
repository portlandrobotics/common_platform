EESchema Schematic File Version 4
EELAYER 30 0
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
$Comp
L Connector_Generic:Conn_01x06 ENCODER_LEFT1
U 1 1 5EAE1434
P 5700 2750
F 0 "ENCODER_LEFT1" H 5618 3167 50  0000 C CNN
F 1 "Conn_01x06" H 5618 3076 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 5700 2750 50  0001 C CNN
F 3 "~" H 5700 2750 50  0001 C CNN
	1    5700 2750
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 C_LEFTx1
U 1 1 5EB31141
P 6450 2750
F 0 "C_LEFTx1" H 6300 2200 50  0000 L CNN
F 1 "Conn_01x07" H 6200 2300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 6450 2750 50  0001 C CNN
F 3 "~" H 6450 2750 50  0001 C CNN
	1    6450 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B2P1
U 1 1 5EB35877
P 5800 4000
F 0 "B2P1" H 5950 4050 50  0000 C CNN
F 1 "BTAB" H 5950 3950 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4000 50  0001 C CNN
F 3 "~" H 5800 4000 50  0001 C CNN
	1    5800 4000
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B4P1
U 1 1 5EB3A588
P 5800 4400
F 0 "B4P1" H 5950 4450 50  0000 C CNN
F 1 "BTAB" H 5950 4350 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4400 50  0001 C CNN
F 3 "~" H 5800 4400 50  0001 C CNN
	1    5800 4400
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B4M1
U 1 1 5EB3A889
P 5800 4600
F 0 "B4M1" H 5950 4650 50  0000 C CNN
F 1 "BTAB" H 5950 4550 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4600 50  0001 C CNN
F 3 "~" H 5800 4600 50  0001 C CNN
	1    5800 4600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6000 4400 6100 4400
Wire Wire Line
	6100 4400 6100 4200
Wire Wire Line
	6100 4200 6000 4200
$Comp
L power:GND #PWR0105
U 1 1 5EB3BDA5
P 6100 4650
F 0 "#PWR0105" H 6100 4400 50  0001 C CNN
F 1 "GND" H 6105 4477 50  0000 C CNN
F 2 "" H 6100 4650 50  0001 C CNN
F 3 "" H 6100 4650 50  0001 C CNN
	1    6100 4650
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0106
U 1 1 5EB3C864
P 6100 3900
F 0 "#PWR0106" H 6100 3750 50  0001 C CNN
F 1 "+BATT" H 6115 4073 50  0000 C CNN
F 2 "" H 6100 3900 50  0001 C CNN
F 3 "" H 6100 3900 50  0001 C CNN
	1    6100 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4000 6100 4000
Wire Wire Line
	6100 4000 6100 3900
Wire Wire Line
	6000 4600 6100 4600
Wire Wire Line
	6100 4600 6100 4650
$Comp
L Connector_Generic:Conn_01x01 B2M1
U 1 1 5EB39EE4
P 5800 4200
F 0 "B2M1" H 5950 4250 50  0000 C CNN
F 1 "BTAB" H 5950 4150 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4200 50  0001 C CNN
F 3 "~" H 5800 4200 50  0001 C CNN
	1    5800 4200
	-1   0    0    -1  
$EndComp
Text Label 6100 4300 0    50   ~ 0
VBAT_MID
$Comp
L power:GND #PWR0107
U 1 1 5EB4E5B4
P 5950 3100
F 0 "#PWR0107" H 5950 2850 50  0001 C CNN
F 1 "GND" H 5955 2927 50  0000 C CNN
F 2 "" H 5950 3100 50  0001 C CNN
F 3 "" H 5950 3100 50  0001 C CNN
	1    5950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3100 5950 3050
Wire Wire Line
	5950 3050 5900 3050
Wire Wire Line
	5900 2550 6250 2550
Wire Wire Line
	5900 2650 6250 2650
Wire Wire Line
	5900 2750 6250 2750
Wire Wire Line
	5900 2850 6250 2850
Wire Wire Line
	5900 2950 6250 2950
Wire Wire Line
	5950 3050 6250 3050
Connection ~ 5950 3050
Wire Wire Line
	6200 4000 6100 4000
Connection ~ 6100 4000
Text Label 6150 2550 2    50   ~ 0
LM1x
Text Label 6150 2650 2    50   ~ 0
LM2x
Text Label 6150 2750 2    50   ~ 0
L5V
Text Label 6200 2850 2    50   ~ 0
LENCAx
Text Label 6200 2950 2    50   ~ 0
LENCBx
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5EB33870
P 6200 3900
F 0 "#FLG0101" H 6200 3975 50  0001 C CNN
F 1 "PWR_FLAG" V 6200 4028 50  0000 L CNN
F 2 "" H 6200 3900 50  0001 C CNN
F 3 "~" H 6200 3900 50  0001 C CNN
	1    6200 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 3900 6200 4000
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5EB34331
P 6200 4600
F 0 "#FLG0102" H 6200 4675 50  0001 C CNN
F 1 "PWR_FLAG" V 6200 4728 50  0000 L CNN
F 2 "" H 6200 4600 50  0001 C CNN
F 3 "~" H 6200 4600 50  0001 C CNN
	1    6200 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 4600 6200 4600
Connection ~ 6100 4600
Wire Wire Line
	7250 4500 7250 4600
$Comp
L power:GND #PWR02
U 1 1 5EAE4356
P 7250 4600
F 0 "#PWR02" H 7250 4350 50  0001 C CNN
F 1 "GND" H 7255 4427 50  0000 C CNN
F 2 "" H 7250 4600 50  0001 C CNN
F 3 "" H 7250 4600 50  0001 C CNN
	1    7250 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR01
U 1 1 5EAE5D27
P 7500 4100
F 0 "#PWR01" H 7500 3950 50  0001 C CNN
F 1 "+BATT" H 7515 4273 50  0000 C CNN
F 2 "" H 7500 4100 50  0001 C CNN
F 3 "" H 7500 4100 50  0001 C CNN
	1    7500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 4200 6100 4200
Connection ~ 6100 4200
Wire Wire Line
	7250 4500 7650 4500
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5EAED124
P 7850 4300
F 0 "J1" H 7930 4292 50  0000 L CNN
F 1 "Conn_01x04" H 7930 4201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7850 4300 50  0001 C CNN
F 3 "~" H 7850 4300 50  0001 C CNN
	1    7850 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 4400 6850 4400
Wire Wire Line
	6850 4400 6850 4200
Wire Wire Line
	7650 4300 7500 4300
Wire Wire Line
	7500 4300 7500 4200
Wire Wire Line
	7650 4200 7500 4200
Connection ~ 7500 4200
Wire Wire Line
	7500 4200 7500 4100
$EndSCHEMATC