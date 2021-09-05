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
L MCU_Microchip_ATtiny:ATtiny1604-SS U2
U 1 1 612492EC
P 5850 3500
F 0 "U2" H 5450 4150 50  0000 C CNN
F 1 "ATtiny1604-SS" H 6150 4200 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5850 3500 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny804_1604-Data-Sheet-40002028A.pdf" H 5850 3500 50  0001 C CNN
	1    5850 3500
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:MCP1703A-5002_SOT23 U1
U 1 1 6124A4B7
P 4200 2700
F 0 "U1" H 4200 2942 50  0000 C CNN
F 1 "MCP1703A-5002_SOT23" H 4200 2851 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4200 2900 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005122B.pdf" H 4200 2650 50  0001 C CNN
	1    4200 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6124B506
P 4200 3100
F 0 "#PWR05" H 4200 2850 50  0001 C CNN
F 1 "GND" H 4205 2927 50  0000 C CNN
F 2 "" H 4200 3100 50  0001 C CNN
F 3 "" H 4200 3100 50  0001 C CNN
	1    4200 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 6124C0D9
P 5850 4300
F 0 "#PWR09" H 5850 4050 50  0001 C CNN
F 1 "GND" H 5855 4127 50  0000 C CNN
F 2 "" H 5850 4300 50  0001 C CNN
F 3 "" H 5850 4300 50  0001 C CNN
	1    5850 4300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR08
U 1 1 6124CA78
P 5150 2500
F 0 "#PWR08" H 5150 2350 50  0001 C CNN
F 1 "+3V3" H 5165 2673 50  0000 C CNN
F 2 "" H 5150 2500 50  0001 C CNN
F 3 "" H 5150 2500 50  0001 C CNN
	1    5150 2500
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR02
U 1 1 6124D5B1
P 3250 2500
F 0 "#PWR02" H 3250 2350 50  0001 C CNN
F 1 "+BATT" H 3265 2673 50  0000 C CNN
F 2 "" H 3250 2500 50  0001 C CNN
F 3 "" H 3250 2500 50  0001 C CNN
	1    3250 2500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 6124EE86
P 2600 2750
F 0 "J1" H 2518 3067 50  0000 C CNN
F 1 "Conn_01x03" H 2518 2976 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2600 2750 50  0001 C CNN
F 3 "~" H 2600 2750 50  0001 C CNN
	1    2600 2750
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 6124F9D8
P 2600 3400
F 0 "J2" H 2518 3717 50  0000 C CNN
F 1 "Conn_01x03" H 2518 3626 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2600 3400 50  0001 C CNN
F 3 "~" H 2600 3400 50  0001 C CNN
	1    2600 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2800 2650 3050 2650
Wire Wire Line
	3250 2650 3250 2500
Wire Wire Line
	3250 2650 3250 2700
Wire Wire Line
	3250 2700 3650 2700
Connection ~ 3250 2650
Wire Wire Line
	2800 3300 3250 3300
Wire Wire Line
	3250 3300 3250 2700
Connection ~ 3250 2700
Wire Wire Line
	2800 2850 2900 2850
Wire Wire Line
	2900 2850 2900 3050
Wire Wire Line
	2900 3500 2800 3500
Wire Wire Line
	2900 3500 2900 3600
Connection ~ 2900 3500
$Comp
L power:GND #PWR01
U 1 1 61250763
P 2900 3600
F 0 "#PWR01" H 2900 3350 50  0001 C CNN
F 1 "GND" H 2905 3427 50  0000 C CNN
F 2 "" H 2900 3600 50  0001 C CNN
F 3 "" H 2900 3600 50  0001 C CNN
	1    2900 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3100 4200 3050
Wire Wire Line
	4500 2700 4650 2700
Wire Wire Line
	5850 2700 5850 2800
Wire Wire Line
	5150 2500 5150 2700
Connection ~ 5150 2700
Wire Wire Line
	5150 2700 5850 2700
$Comp
L Device:C C1
U 1 1 61251284
P 3650 2900
F 0 "C1" H 3765 2946 50  0000 L CNN
F 1 "2.2u" H 3765 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3688 2750 50  0001 C CNN
F 3 "~" H 3650 2900 50  0001 C CNN
	1    3650 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 612517FC
P 4650 2900
F 0 "C3" H 4765 2946 50  0000 L CNN
F 1 "2.2u" H 4765 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4688 2750 50  0001 C CNN
F 3 "~" H 4650 2900 50  0001 C CNN
	1    4650 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 61251C09
P 4000 4400
F 0 "C2" H 4115 4446 50  0000 L CNN
F 1 "100n" H 4115 4355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4038 4250 50  0001 C CNN
F 3 "~" H 4000 4400 50  0001 C CNN
	1    4000 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 612524CE
P 3750 4350
F 0 "R1" H 3600 4400 50  0000 L CNN
F 1 "330k" H 3500 4300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3750 4350 50  0001 C CNN
F 3 "~" H 3750 4350 50  0001 C CNN
	1    3750 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 61252B77
P 3900 3900
F 0 "R2" H 3968 3946 50  0000 L CNN
F 1 "1M" H 3968 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3900 3900 50  0001 C CNN
F 3 "~" H 3900 3900 50  0001 C CNN
	1    3900 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R8
U 1 1 6125308F
P 7800 4000
F 0 "R8" H 7732 3954 50  0000 R CNN
F 1 "10k" H 7732 4045 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7800 4000 50  0001 C CNN
F 3 "~" H 7800 4000 50  0001 C CNN
	1    7800 4000
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small_US R9
U 1 1 61253D36
P 7850 1850
F 0 "R9" H 7918 1896 50  0000 L CNN
F 1 "10k" H 7918 1805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7850 1850 50  0001 C CNN
F 3 "~" H 7850 1850 50  0001 C CNN
	1    7850 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R11
U 1 1 6125409D
P 9000 1200
F 0 "R11" H 9068 1246 50  0000 L CNN
F 1 "10k" H 9068 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 9000 1200 50  0001 C CNN
F 3 "~" H 9000 1200 50  0001 C CNN
	1    9000 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J6
U 1 1 612544DE
P 9600 1800
F 0 "J6" H 9680 1792 50  0000 L CNN
F 1 "Conn_01x06" H 9680 1701 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6_Handsoldering" H 9600 1800 50  0001 C CNN
F 3 "~" H 9600 1800 50  0001 C CNN
	1    9600 1800
	1    0    0    -1  
$EndComp
Text Notes 9150 1450 0    50   ~ 0
NX138BKS S1 G1 D2 S2 G2 D1
$Comp
L power:GND #PWR015
U 1 1 612560DF
P 9250 2200
F 0 "#PWR015" H 9250 1950 50  0001 C CNN
F 1 "GND" H 9255 2027 50  0000 C CNN
F 2 "" H 9250 2200 50  0001 C CNN
F 3 "" H 9250 2200 50  0001 C CNN
	1    9250 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2200 9250 1900
Wire Wire Line
	9250 1600 9400 1600
Wire Wire Line
	9400 1900 9250 1900
Connection ~ 9250 1900
Wire Wire Line
	9250 1900 9250 1600
Wire Wire Line
	9400 1800 9000 1800
Wire Wire Line
	9000 1800 9000 1300
$Comp
L Device:R_Small_US R10
U 1 1 6125824E
P 8650 1200
F 0 "R10" H 8718 1246 50  0000 L CNN
F 1 "10k" H 8718 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 8650 1200 50  0001 C CNN
F 3 "~" H 8650 1200 50  0001 C CNN
	1    8650 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2100 8650 2100
Wire Wire Line
	8650 2100 8650 1300
Wire Wire Line
	9000 1100 9000 950 
Wire Wire Line
	9000 950  8800 950 
Wire Wire Line
	8650 950  8650 1100
$Comp
L power:+BATT #PWR014
U 1 1 6125A12B
P 8800 950
F 0 "#PWR014" H 8800 800 50  0001 C CNN
F 1 "+BATT" H 8815 1123 50  0000 C CNN
F 2 "" H 8800 950 50  0001 C CNN
F 3 "" H 8800 950 50  0001 C CNN
	1    8800 950 
	1    0    0    -1  
$EndComp
Connection ~ 8800 950 
Wire Wire Line
	8800 950  8650 950 
$Comp
L Device:R_Small_US R7
U 1 1 6125EA78
P 7650 2500
F 0 "R7" H 7718 2546 50  0000 L CNN
F 1 "10k" H 7718 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7650 2500 50  0001 C CNN
F 3 "~" H 7650 2500 50  0001 C CNN
	1    7650 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 6125ED26
P 7650 2650
F 0 "#PWR010" H 7650 2400 50  0001 C CNN
F 1 "GND" H 7655 2477 50  0000 C CNN
F 2 "" H 7650 2650 50  0001 C CNN
F 3 "" H 7650 2650 50  0001 C CNN
	1    7650 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 6125EF9B
P 7850 2000
F 0 "#PWR013" H 7850 1750 50  0001 C CNN
F 1 "GND" H 7855 1827 50  0000 C CNN
F 2 "" H 7850 2000 50  0001 C CNN
F 3 "" H 7850 2000 50  0001 C CNN
	1    7850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 2600 7650 2650
Wire Wire Line
	7650 2350 7650 2400
Wire Wire Line
	7650 2350 9100 2350
Wire Wire Line
	9100 2350 9100 2000
Wire Wire Line
	9100 2000 9400 2000
Connection ~ 7650 2350
Wire Wire Line
	9400 1700 7950 1700
Wire Wire Line
	7950 1700 7950 1650
Wire Wire Line
	7950 1650 7850 1650
Wire Wire Line
	7850 1750 7850 1650
Connection ~ 7850 1650
Wire Wire Line
	7850 2000 7850 1950
$Comp
L Switch:SW_Push SW1
U 1 1 612628EB
P 7800 4400
F 0 "SW1" V 7754 4548 50  0000 L CNN
F 1 "SW_Push" V 7845 4548 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3305A" H 7800 4600 50  0001 C CNN
F 3 "~" H 7800 4600 50  0001 C CNN
	1    7800 4400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 612641FE
P 7800 4700
F 0 "#PWR012" H 7800 4450 50  0001 C CNN
F 1 "GND" H 7805 4527 50  0000 C CNN
F 2 "" H 7800 4700 50  0001 C CNN
F 3 "" H 7800 4700 50  0001 C CNN
	1    7800 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR011
U 1 1 61264612
P 7800 3800
F 0 "#PWR011" H 7800 3650 50  0001 C CNN
F 1 "+3V3" H 7815 3973 50  0000 C CNN
F 2 "" H 7800 3800 50  0001 C CNN
F 3 "" H 7800 3800 50  0001 C CNN
	1    7800 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3800 7800 3900
Wire Wire Line
	7800 4100 7800 4150
Wire Wire Line
	7800 4600 7800 4650
Wire Wire Line
	7800 4150 7600 4150
Wire Wire Line
	7250 4150 7250 3800
Connection ~ 7800 4150
Wire Wire Line
	7800 4150 7800 4200
Wire Wire Line
	7150 2350 7150 3300
Wire Wire Line
	7050 1650 7050 3200
$Comp
L power:GND #PWR03
U 1 1 61268D74
P 3850 4750
F 0 "#PWR03" H 3850 4500 50  0001 C CNN
F 1 "GND" H 3855 4577 50  0000 C CNN
F 2 "" H 3850 4750 50  0001 C CNN
F 3 "" H 3850 4750 50  0001 C CNN
	1    3850 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR04
U 1 1 612692F6
P 3900 3650
F 0 "#PWR04" H 3900 3500 50  0001 C CNN
F 1 "+BATT" H 3915 3823 50  0000 C CNN
F 2 "" H 3900 3650 50  0001 C CNN
F 3 "" H 3900 3650 50  0001 C CNN
	1    3900 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3650 3900 3800
Wire Wire Line
	3750 4250 3750 4150
Wire Wire Line
	3750 4150 3900 4150
Wire Wire Line
	4000 4150 4000 4250
Wire Wire Line
	3900 4150 3900 4000
Connection ~ 3900 4150
Wire Wire Line
	3900 4150 4000 4150
Wire Wire Line
	4000 4550 4000 4600
Wire Wire Line
	4000 4600 3850 4600
Wire Wire Line
	3750 4600 3750 4450
Wire Wire Line
	3850 4750 3850 4600
Connection ~ 3850 4600
Wire Wire Line
	3850 4600 3750 4600
Wire Wire Line
	4000 4150 5100 4150
Connection ~ 4000 4150
$Comp
L Connector_Generic:Conn_01x06 J4
U 1 1 612709A9
P 8250 5650
F 0 "J4" H 8330 5642 50  0000 L CNN
F 1 "Conn_01x06" H 8330 5551 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8250 5650 50  0001 C CNN
F 3 "~" H 8250 5650 50  0001 C CNN
	1    8250 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5450 8050 5450
Wire Wire Line
	8050 5550 7900 5550
Wire Wire Line
	8050 5650 7900 5650
Wire Wire Line
	8050 5750 7900 5750
Wire Wire Line
	8050 5850 7900 5850
Wire Wire Line
	8050 5950 7900 5950
Text Label 7900 5450 2    50   ~ 0
+3V3
Text Label 7900 5550 2    50   ~ 0
GND
Text Label 7900 5650 2    50   ~ 0
TX
Text Label 7900 5750 2    50   ~ 0
RX
Text Label 7900 5850 2    50   ~ 0
SCL
Text Label 7900 5950 2    50   ~ 0
SDA
Text Label 5250 3300 2    50   ~ 0
TX
Text Label 5250 3400 2    50   ~ 0
RX
Text Label 5250 3100 2    50   ~ 0
SCL
Text Label 5250 3200 2    50   ~ 0
SDA
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 6127D77E
P 7000 5750
F 0 "J3" H 7080 5742 50  0000 L CNN
F 1 "Conn_01x04" H 7080 5651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 7000 5750 50  0001 C CNN
F 3 "~" H 7000 5750 50  0001 C CNN
	1    7000 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5650 6550 5650
Wire Wire Line
	6800 5950 6650 5950
Text Label 6550 5650 0    50   ~ 0
+3V3
Text Label 6650 5950 0    50   ~ 0
GND
$Comp
L Device:R_Small_US R3
U 1 1 61283593
P 6400 5850
F 0 "R3" V 6450 5700 50  0000 C CNN
F 1 "4.7k" V 6500 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 6400 5850 50  0001 C CNN
F 3 "~" H 6400 5850 50  0001 C CNN
	1    6400 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 5850 6500 5850
Wire Wire Line
	6300 5850 6000 5850
Wire Wire Line
	6000 5850 6000 5800
Wire Wire Line
	6000 5750 6800 5750
Wire Wire Line
	6000 5800 5700 5800
Connection ~ 6000 5800
Wire Wire Line
	6000 5800 6000 5750
Text Label 5700 5800 0    50   ~ 0
UPDI
Text Label 6450 3100 0    50   ~ 0
UPDI
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 6128A715
P 8700 4300
F 0 "J5" H 8780 4292 50  0000 L CNN
F 1 "Conn_01x02" H 8780 4201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8700 4300 50  0001 C CNN
F 3 "~" H 8700 4300 50  0001 C CNN
	1    8700 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4150 8350 4150
Wire Wire Line
	8350 4150 8350 4300
Wire Wire Line
	8350 4300 8500 4300
Wire Wire Line
	8350 4400 8350 4650
Wire Wire Line
	8350 4650 7800 4650
Wire Wire Line
	8350 4400 8500 4400
Connection ~ 7800 4650
Wire Wire Line
	7800 4650 7800 4700
$Comp
L Device:R_Small_US R5
U 1 1 6128F625
P 7500 4150
F 0 "R5" V 7705 4150 50  0000 C CNN
F 1 "1k" V 7614 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7500 4150 50  0001 C CNN
F 3 "~" H 7500 4150 50  0001 C CNN
	1    7500 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 4150 7250 4150
Wire Wire Line
	3650 3050 4200 3050
Connection ~ 4200 3050
Wire Wire Line
	4200 3050 4200 3000
Wire Wire Line
	4200 3050 4650 3050
Wire Wire Line
	4650 2750 4650 2700
Connection ~ 4650 2700
Wire Wire Line
	4650 2700 5150 2700
Wire Wire Line
	3650 2750 3650 2700
Connection ~ 3650 2700
Wire Wire Line
	3650 2700 3900 2700
Text Label 8650 1450 0    50   ~ 0
PSS
Text Label 9000 1600 0    50   ~ 0
PSM
Text Label 2800 2750 0    50   ~ 0
PSS
Text Label 2800 3400 0    50   ~ 0
PSM
Wire Wire Line
	5850 4300 5850 4200
$Comp
L Device:C C4
U 1 1 6129D4DB
P 5150 1700
F 0 "C4" H 5265 1746 50  0000 L CNN
F 1 "2.2u" H 5265 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5188 1550 50  0001 C CNN
F 3 "~" H 5150 1700 50  0001 C CNN
	1    5150 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR06
U 1 1 6129D94C
P 5150 1450
F 0 "#PWR06" H 5150 1300 50  0001 C CNN
F 1 "+3V3" H 5165 1623 50  0000 C CNN
F 2 "" H 5150 1450 50  0001 C CNN
F 3 "" H 5150 1450 50  0001 C CNN
	1    5150 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 6129DE2D
P 5150 1900
F 0 "#PWR07" H 5150 1650 50  0001 C CNN
F 1 "GND" H 5155 1727 50  0000 C CNN
F 2 "" H 5150 1900 50  0001 C CNN
F 3 "" H 5150 1900 50  0001 C CNN
	1    5150 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 6129E1DB
P 5500 1700
F 0 "C5" H 5615 1746 50  0000 L CNN
F 1 "100n" H 5615 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5538 1550 50  0001 C CNN
F 3 "~" H 5500 1700 50  0001 C CNN
	1    5500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 1550 5150 1550
Wire Wire Line
	5150 1550 5150 1450
Connection ~ 5150 1550
Wire Wire Line
	5500 1850 5150 1850
Wire Wire Line
	5150 1850 5150 1900
Connection ~ 5150 1850
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 612A4C5D
P 3050 2650
F 0 "#FLG0101" H 3050 2725 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 2823 50  0000 C CNN
F 2 "" H 3050 2650 50  0001 C CNN
F 3 "~" H 3050 2650 50  0001 C CNN
	1    3050 2650
	1    0    0    -1  
$EndComp
Connection ~ 3050 2650
Wire Wire Line
	3050 2650 3250 2650
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 612A51BD
P 2900 3050
F 0 "#FLG0102" H 2900 3125 50  0001 C CNN
F 1 "PWR_FLAG" V 2900 3178 50  0000 L CNN
F 2 "" H 2900 3050 50  0001 C CNN
F 3 "~" H 2900 3050 50  0001 C CNN
	1    2900 3050
	0    1    1    0   
$EndComp
Connection ~ 2900 3050
Wire Wire Line
	2900 3050 2900 3500
Wire Wire Line
	7050 1650 7850 1650
Wire Wire Line
	7150 2350 7650 2350
Wire Wire Line
	6450 3200 7050 3200
Wire Wire Line
	6450 3300 7150 3300
Wire Wire Line
	6450 3800 7250 3800
Wire Wire Line
	6450 3500 6650 3500
Wire Wire Line
	6650 3500 6650 4700
Wire Wire Line
	6650 4700 5100 4700
Wire Wire Line
	5100 4700 5100 4150
NoConn ~ 6450 3400
NoConn ~ 6450 3600
NoConn ~ 6450 3700
Text Label 7500 1650 0    50   ~ 0
PSYS_CTRL
Text Label 7550 2350 0    50   ~ 0
PMTR_CTRL
Text Label 4550 4150 0    50   ~ 0
VBAT_DIV
Text Label 6900 3800 0    50   ~ 0
PWR_BUTTON
Text Label 8350 4150 0    50   ~ 0
PBTN
Text Label 6550 5850 0    50   ~ 0
UPDI_TX
$EndSCHEMATC
