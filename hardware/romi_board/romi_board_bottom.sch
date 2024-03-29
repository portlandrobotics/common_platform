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
L Connector_Generic:Conn_01x06 C_IBC_T1
U 1 1 5EB31141
P 9700 2800
F 0 "C_IBC_T1" H 9800 2700 50  0000 L CNN
F 1 "Conn_01x06" H 9800 2800 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9700 2800 50  0001 C CNN
F 3 "~" H 9700 2800 50  0001 C CNN
	1    9700 2800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B2P1
U 1 1 5EB35877
P 5800 4400
F 0 "B2P1" H 5950 4450 50  0000 C CNN
F 1 "BTAB" H 5950 4350 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4400 50  0001 C CNN
F 3 "~" H 5800 4400 50  0001 C CNN
	1    5800 4400
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B4P1
U 1 1 5EB3A588
P 5800 4000
F 0 "B4P1" H 5950 4050 50  0000 C CNN
F 1 "BTAB" H 5950 3950 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4000 50  0001 C CNN
F 3 "~" H 5800 4000 50  0001 C CNN
	1    5800 4000
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 B4M1
U 1 1 5EB3A889
P 5800 4200
F 0 "B4M1" H 5950 4250 50  0000 C CNN
F 1 "BTAB" H 5950 4150 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4200 50  0001 C CNN
F 3 "~" H 5800 4200 50  0001 C CNN
	1    5800 4200
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
	6100 4600 6100 4650
Text Label 6100 4300 0    50   ~ 0
VBAT_MID
$Comp
L power:GND #PWR0107
U 1 1 5EB4E5B4
P 6600 3150
F 0 "#PWR0107" H 6600 2900 50  0001 C CNN
F 1 "GND" H 6605 2977 50  0000 C CNN
F 2 "" H 6600 3150 50  0001 C CNN
F 3 "" H 6600 3150 50  0001 C CNN
	1    6600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2550 6250 2550
Wire Wire Line
	5900 2650 6250 2650
Wire Wire Line
	5900 2950 6250 2950
Wire Wire Line
	6200 4000 6100 4000
Connection ~ 6100 4000
Text Label 6250 2550 2    50   ~ 0
LM1
Text Label 6250 2650 2    50   ~ 0
LM2
Text Label 6250 2850 2    50   ~ 0
LENCA
Text Label 6250 2950 2    50   ~ 0
LENCB
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
$Comp
L Connector_Generic:Conn_01x06 ENCODER_RIGHT1
U 1 1 5EBDCED4
P 7400 2750
F 0 "ENCODER_RIGHT1" H 7318 3167 50  0000 C CNN
F 1 "Conn_01x06" H 7318 3076 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 7400 2750 50  0001 C CNN
F 3 "~" H 7400 2750 50  0001 C CNN
	1    7400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2550 7200 2550
Wire Wire Line
	6850 2650 7200 2650
Wire Wire Line
	6850 2850 7200 2850
Wire Wire Line
	6850 2950 7200 2950
Text Label 6850 2550 0    50   ~ 0
RM1
Text Label 6850 2650 0    50   ~ 0
RM2
Text Label 6850 2850 0    50   ~ 0
RENCA
Text Label 6850 2950 0    50   ~ 0
RENCB
$Comp
L power:+3.3V #PWR0101
U 1 1 5EBF4B82
P 6600 2400
F 0 "#PWR0101" H 6600 2250 50  0001 C CNN
F 1 "+3.3V" H 6615 2573 50  0000 C CNN
F 2 "" H 6600 2400 50  0001 C CNN
F 3 "" H 6600 2400 50  0001 C CNN
	1    6600 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 3050 6600 3050
Wire Wire Line
	6600 3150 6600 3050
Connection ~ 6600 3050
Wire Wire Line
	6600 3050 7200 3050
Wire Wire Line
	5900 2750 6600 2750
Wire Wire Line
	6600 2400 6600 2750
Connection ~ 6600 2750
Wire Wire Line
	6600 2750 7200 2750
Wire Wire Line
	5900 2850 6250 2850
$Comp
L Connector_Generic:Conn_01x06 C_IBC_L1
U 1 1 5EDD1CED
P 9700 3750
F 0 "C_IBC_L1" H 9800 3700 50  0000 L CNN
F 1 "Conn_01x06" H 9800 3850 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9700 3750 50  0001 C CNN
F 3 "~" H 9700 3750 50  0001 C CNN
	1    9700 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 C_IBC_R1
U 1 1 5EDD2A1B
P 9700 4500
F 0 "C_IBC_R1" H 9780 4492 50  0000 L CNN
F 1 "Conn_01x02" H 9780 4401 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9700 4500 50  0001 C CNN
F 3 "~" H 9700 4500 50  0001 C CNN
	1    9700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 4500 9300 4500
Wire Wire Line
	9300 4600 9500 4600
Text Label 9300 4600 2    50   ~ 0
LENCB
Text Label 9300 4500 2    50   ~ 0
LENCA
$Comp
L power:GND #PWR0102
U 1 1 5EDDA95A
P 9350 4100
F 0 "#PWR0102" H 9350 3850 50  0001 C CNN
F 1 "GND" H 9355 3927 50  0000 C CNN
F 2 "" H 9350 4100 50  0001 C CNN
F 3 "" H 9350 4100 50  0001 C CNN
	1    9350 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3950 9350 3950
Wire Wire Line
	9350 3950 9350 4050
Wire Wire Line
	9500 4050 9350 4050
Connection ~ 9350 4050
Wire Wire Line
	9350 4050 9350 4100
Text Label 9500 3850 2    50   ~ 0
RENCB
Text Label 9500 3750 2    50   ~ 0
RENCA
$Comp
L power:+BATT #PWR0103
U 1 1 5EDDC3C0
P 9350 3500
F 0 "#PWR0103" H 9350 3350 50  0001 C CNN
F 1 "+BATT" H 9365 3673 50  0000 C CNN
F 2 "" H 9350 3500 50  0001 C CNN
F 3 "" H 9350 3500 50  0001 C CNN
	1    9350 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 3500 9350 3550
Wire Wire Line
	9350 3650 9500 3650
Wire Wire Line
	9500 3550 9350 3550
Connection ~ 9350 3550
Wire Wire Line
	9350 3550 9350 3650
Text Label 9500 2600 2    50   ~ 0
LM1
Text Label 9500 2700 2    50   ~ 0
LM2
Text Label 9500 3000 2    50   ~ 0
RM1
Text Label 9500 3100 2    50   ~ 0
RM2
$Comp
L power:+3.3V #PWR0104
U 1 1 5EDE2B3A
P 9050 2700
F 0 "#PWR0104" H 9050 2550 50  0001 C CNN
F 1 "+3.3V" H 9065 2873 50  0000 C CNN
F 2 "" H 9050 2700 50  0001 C CNN
F 3 "" H 9050 2700 50  0001 C CNN
	1    9050 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5EDE35C7
P 9050 3000
F 0 "#PWR0108" H 9050 2750 50  0001 C CNN
F 1 "GND" H 9055 2827 50  0000 C CNN
F 2 "" H 9050 3000 50  0001 C CNN
F 3 "" H 9050 3000 50  0001 C CNN
	1    9050 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 2800 9050 2700
Wire Wire Line
	9050 3000 9050 2900
Wire Wire Line
	9050 2900 9500 2900
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5EDE5143
P 9050 2800
F 0 "#FLG0103" H 9050 2875 50  0001 C CNN
F 1 "PWR_FLAG" V 9050 2928 50  0000 L CNN
F 2 "" H 9050 2800 50  0001 C CNN
F 3 "~" H 9050 2800 50  0001 C CNN
	1    9050 2800
	0    -1   -1   0   
$EndComp
Connection ~ 9050 2800
Wire Wire Line
	9050 2800 9500 2800
$Comp
L Mechanical:MountingHole H1
U 1 1 5EDF363D
P 4150 6700
F 0 "H1" H 4250 6746 50  0000 L CNN
F 1 "MountingHole" H 4250 6655 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4150 6700 50  0001 C CNN
F 3 "~" H 4150 6700 50  0001 C CNN
	1    4150 6700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5EDF59D9
P 4150 6900
F 0 "H2" H 4250 6946 50  0000 L CNN
F 1 "MountingHole" H 4250 6855 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4150 6900 50  0001 C CNN
F 3 "~" H 4150 6900 50  0001 C CNN
	1    4150 6900
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5EDF61AD
P 4150 7100
F 0 "H3" H 4250 7146 50  0000 L CNN
F 1 "MountingHole" H 4250 7055 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4150 7100 50  0001 C CNN
F 3 "~" H 4150 7100 50  0001 C CNN
	1    4150 7100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5EDF63E3
P 4150 7300
F 0 "H4" H 4250 7346 50  0000 L CNN
F 1 "MountingHole" H 4250 7255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4150 7300 50  0001 C CNN
F 3 "~" H 4150 7300 50  0001 C CNN
	1    4150 7300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 5EDF67DA
P 4150 7500
F 0 "H5" H 4250 7546 50  0000 L CNN
F 1 "MountingHole" H 4250 7455 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4150 7500 50  0001 C CNN
F 3 "~" H 4150 7500 50  0001 C CNN
	1    4150 7500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 5EDF6984
P 4950 6700
F 0 "H6" H 5050 6746 50  0000 L CNN
F 1 "MountingHole" H 5050 6655 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.5mm" H 4950 6700 50  0001 C CNN
F 3 "~" H 4950 6700 50  0001 C CNN
	1    4950 6700
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H7
U 1 1 5EDF6E7E
P 4950 6900
F 0 "H7" H 5050 6946 50  0000 L CNN
F 1 "MountingHole" H 5050 6855 50  0000 L CNN
F 2 "MountingHole:MountingHole_6mm" H 4950 6900 50  0001 C CNN
F 3 "~" H 4950 6900 50  0001 C CNN
	1    4950 6900
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H8
U 1 1 5EDF7262
P 4950 7100
F 0 "H8" H 5050 7146 50  0000 L CNN
F 1 "MountingHole" H 5050 7055 50  0000 L CNN
F 2 "MountingHole:MountingHole_6mm" H 4950 7100 50  0001 C CNN
F 3 "~" H 4950 7100 50  0001 C CNN
	1    4950 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4600 6100 4600
$Comp
L Connector_Generic:Conn_01x01 B2M1
U 1 1 5EB39EE4
P 5800 4600
F 0 "B2M1" H 5950 4650 50  0000 C CNN
F 1 "BTAB" H 5950 4550 50  0000 C CNN
F 2 "teensy:romi_battery_tab" H 5800 4600 50  0001 C CNN
F 3 "~" H 5800 4600 50  0001 C CNN
	1    5800 4600
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
