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
L Connector_Generic:Conn_01x06 ENCODER_RIGHT1
U 1 1 5EAE1434
P 6050 3900
F 0 "ENCODER_RIGHT1" H 5968 4317 50  0000 C CNN
F 1 "Conn_01x06" H 5968 4226 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 6050 3900 50  0001 C CNN
F 3 "~" H 6050 3900 50  0001 C CNN
	1    6050 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 C_RIGHTx1
U 1 1 5EB31141
P 5300 3900
F 0 "C_RIGHTx1" H 5150 3350 50  0000 L CNN
F 1 "Conn_01x07" H 5050 3450 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 5300 3900 50  0001 C CNN
F 3 "~" H 5300 3900 50  0001 C CNN
	1    5300 3900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5EB4E5B4
P 5800 4250
F 0 "#PWR0107" H 5800 4000 50  0001 C CNN
F 1 "GND" H 5805 4077 50  0000 C CNN
F 2 "" H 5800 4250 50  0001 C CNN
F 3 "" H 5800 4250 50  0001 C CNN
	1    5800 4250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5800 4250 5800 4200
Wire Wire Line
	5800 4200 5850 4200
Wire Wire Line
	5850 3700 5500 3700
Wire Wire Line
	5850 3800 5500 3800
Wire Wire Line
	5850 3900 5500 3900
Wire Wire Line
	5850 4000 5500 4000
Wire Wire Line
	5850 4100 5500 4100
Wire Wire Line
	5800 4200 5600 4200
Connection ~ 5800 4200
Text Label 5600 3700 0    50   ~ 0
RM1x
Text Label 5600 3800 0    50   ~ 0
RM2x
Text Label 5600 3900 0    50   ~ 0
R5V
Text Label 5550 4000 0    50   ~ 0
RENCAx
Text Label 5550 4100 0    50   ~ 0
RENCBx
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5EAE732F
P 5600 4550
F 0 "#FLG0101" H 5600 4625 50  0001 C CNN
F 1 "PWR_FLAG" H 5600 4723 50  0000 C CNN
F 2 "" H 5600 4550 50  0001 C CNN
F 3 "~" H 5600 4550 50  0001 C CNN
	1    5600 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	5600 4550 5600 4200
Connection ~ 5600 4200
Wire Wire Line
	5600 4200 5500 4200
$EndSCHEMATC
