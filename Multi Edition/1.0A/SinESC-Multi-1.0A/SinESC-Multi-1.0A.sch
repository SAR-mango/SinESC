EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SinESC Multi Edition"
Date "2020-08-31"
Rev "1.0A"
Comp "© SinESC 2020"
Comment1 "Drawn by SAR_mango"
Comment2 "* All resistors are 1% 0201; current shunts are 1% 1206."
Comment3 "* Capacitor voltage ratings & packages vary. See BOM."
Comment4 "* Despite BOM consolidation, E192 values are required."
$EndDescr
$Comp
L Amplifier_Operational:MAX4239AUT U2
U 1 1 5F5FD7B9
P 2000 6800
F 0 "U2" H 1950 7050 50  0000 L CNN
F 1 "MAX4239AUT" H 1950 6950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2000 6800 50  0001 C CNN
F 3 "" H 2150 6950 50  0001 C CNN
F 4 "MAX4239AUT+T" H 2000 6800 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 2000 6800 50  0001 C CNN "Voltage Rating"
	1    2000 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 5F605CDA
P 1300 6500
F 0 "R4" H 1368 6546 50  0000 L CNN
F 1 "5.1kΩ" H 1368 6455 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 1340 6490 50  0001 C CNN
F 3 "" H 1300 6500 50  0001 C CNN
F 4 "ERJ-1GNF5101C" H 1300 6500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1300 6500 50  0001 C CNN "Voltage Rating"
	1    1300 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 5F6081DF
P 1300 6900
F 0 "R5" H 1368 6946 50  0000 L CNN
F 1 "976Ω" H 1368 6855 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 1340 6890 50  0001 C CNN
F 3 "" H 1300 6900 50  0001 C CNN
F 4 "ERJ-1GNF9760C" H 1300 6900 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1300 6900 50  0001 C CNN "Voltage Rating"
	1    1300 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 5F609957
P 1100 6700
F 0 "R1" V 1000 6700 50  0000 C CNN
F 1 "86.6Ω" V 1200 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 1140 6690 50  0001 C CNN
F 3 "" H 1100 6700 50  0001 C CNN
F 4 "ERJ-1GNF86R6C" H 1100 6700 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1100 6700 50  0001 C CNN "Voltage Rating"
	1    1100 6700
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5F6398AC
P 1600 7350
F 0 "C7" H 1600 7450 50  0000 L CNN
F 1 "6.8nF" H 1600 7250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1638 7200 50  0001 C CNN
F 3 "" H 1600 7350 50  0001 C CNN
F 4 "25V" H 1600 7350 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 1600 7350 50  0001 C CNN "Manufacturer Part #"
	1    1600 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 6700 1300 6700
Wire Wire Line
	1300 6650 1300 6700
Connection ~ 1300 6700
Wire Wire Line
	1300 6700 1300 6750
Text GLabel 950  6700 0    50   Input ~ 0
SHUNT_U
$Comp
L Device:R_US R9
U 1 1 5F65E4E4
P 2550 7400
F 0 "R9" H 2618 7446 50  0000 L CNN
F 1 "470Ω" H 2618 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 2590 7390 50  0001 C CNN
F 3 "" H 2550 7400 50  0001 C CNN
F 4 "ERJ-1GNF4700C" H 2550 7400 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 2550 7400 50  0001 C CNN "Voltage Rating"
	1    2550 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R8
U 1 1 5F67E33F
P 2550 7000
F 0 "R8" H 2618 7046 50  0000 L CNN
F 1 "15kΩ" H 2618 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 2590 6990 50  0001 C CNN
F 3 "" H 2550 7000 50  0001 C CNN
F 4 "ERJ-1GNF1502C" H 2550 7000 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 2550 7000 50  0001 C CNN "Voltage Rating"
	1    2550 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 7050 1300 7550
Wire Wire Line
	1900 7100 1900 7550
Connection ~ 1900 7550
Wire Wire Line
	1300 7550 1600 7550
Wire Wire Line
	1300 6700 1600 6700
Wire Wire Line
	1600 7500 1600 7550
Connection ~ 1600 7550
Wire Wire Line
	1600 7550 1900 7550
Wire Wire Line
	1600 7200 1600 6700
Connection ~ 1600 6700
Wire Wire Line
	1600 6700 1700 6700
Wire Wire Line
	1700 6900 1700 7200
Text GLabel 2600 6800 2    50   Output ~ 0
CUR_U
Wire Wire Line
	2000 7100 2450 7100
Wire Wire Line
	2550 7200 2550 7150
Wire Wire Line
	1700 7200 2550 7200
Wire Wire Line
	2550 7200 2550 7250
Connection ~ 2550 7200
Wire Wire Line
	1900 7550 2550 7550
Wire Wire Line
	2550 6850 2550 6800
Wire Wire Line
	2550 6800 2600 6800
Wire Wire Line
	2550 6800 2300 6800
Connection ~ 2550 6800
Wire Wire Line
	2450 6350 2450 7100
$Comp
L power:+3.3V #PWR06
U 1 1 5F74BE00
P 1900 6300
F 0 "#PWR06" H 1900 6150 50  0001 C CNN
F 1 "+3.3V" H 1900 6450 50  0000 C CNN
F 2 "" H 1900 6300 50  0001 C CNN
F 3 "" H 1900 6300 50  0001 C CNN
	1    1900 6300
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR07
U 1 1 5F7550FA
P 1900 7600
F 0 "#PWR07" H 1900 7350 50  0001 C CNN
F 1 "Earth" H 1900 7450 50  0001 C CNN
F 2 "" H 1900 7600 50  0001 C CNN
F 3 "~" H 1900 7600 50  0001 C CNN
	1    1900 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 7600 1900 7550
$Comp
L Amplifier_Operational:MAX4239AUT U3
U 1 1 5F786954
P 3900 6800
F 0 "U3" H 3850 7050 50  0000 L CNN
F 1 "MAX4239AUT" H 3850 6950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 3900 6800 50  0001 C CNN
F 3 "" H 4050 6950 50  0001 C CNN
F 4 "MAX4239AUT+T" H 3900 6800 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3900 6800 50  0001 C CNN "Voltage Rating"
	1    3900 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R13
U 1 1 5F78695A
P 3200 6500
F 0 "R13" H 3268 6546 50  0000 L CNN
F 1 "5.1kΩ" H 3268 6455 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 3240 6490 50  0001 C CNN
F 3 "" H 3200 6500 50  0001 C CNN
F 4 "ERJ-1GNF5101C" H 3200 6500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3200 6500 50  0001 C CNN "Voltage Rating"
	1    3200 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R14
U 1 1 5F786960
P 3200 6900
F 0 "R14" H 3268 6946 50  0000 L CNN
F 1 "976Ω" H 3268 6855 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 3240 6890 50  0001 C CNN
F 3 "" H 3200 6900 50  0001 C CNN
F 4 "ERJ-1GNF9760C" H 3200 6900 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3200 6900 50  0001 C CNN "Voltage Rating"
	1    3200 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R10
U 1 1 5F786966
P 3000 6700
F 0 "R10" V 2900 6700 50  0000 C CNN
F 1 "86.6Ω" V 3100 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 3040 6690 50  0001 C CNN
F 3 "" H 3000 6700 50  0001 C CNN
F 4 "ERJ-1GNF86R6C" H 3000 6700 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3000 6700 50  0001 C CNN "Voltage Rating"
	1    3000 6700
	0    1    1    0   
$EndComp
$Comp
L Device:C C14
U 1 1 5F78696C
P 3500 7350
F 0 "C14" H 3500 7450 50  0000 L CNN
F 1 "6.8nF" H 3500 7250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 3538 7200 50  0001 C CNN
F 3 "" H 3500 7350 50  0001 C CNN
F 4 "25V" H 3500 7350 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 3500 7350 50  0001 C CNN "Manufacturer Part #"
	1    3500 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 6700 3200 6700
Wire Wire Line
	3200 6650 3200 6700
Connection ~ 3200 6700
Wire Wire Line
	3200 6700 3200 6750
Text GLabel 2850 6700 0    50   Input ~ 0
SHUNT_V
$Comp
L Device:R_US R18
U 1 1 5F786977
P 4450 7400
F 0 "R18" H 4518 7446 50  0000 L CNN
F 1 "470Ω" H 4518 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 4490 7390 50  0001 C CNN
F 3 "" H 4450 7400 50  0001 C CNN
F 4 "ERJ-1GNF4700C" H 4450 7400 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4450 7400 50  0001 C CNN "Voltage Rating"
	1    4450 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R17
U 1 1 5F78697D
P 4450 7000
F 0 "R17" H 4518 7046 50  0000 L CNN
F 1 "15kΩ" H 4518 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 4490 6990 50  0001 C CNN
F 3 "" H 4450 7000 50  0001 C CNN
F 4 "ERJ-1GNF1502C" H 4450 7000 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4450 7000 50  0001 C CNN "Voltage Rating"
	1    4450 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 7050 3200 7550
Wire Wire Line
	3800 7100 3800 7550
Connection ~ 3800 7550
Wire Wire Line
	3200 7550 3500 7550
Wire Wire Line
	3200 6700 3500 6700
Wire Wire Line
	3500 7500 3500 7550
Connection ~ 3500 7550
Wire Wire Line
	3500 7550 3800 7550
Wire Wire Line
	3500 7200 3500 6700
Connection ~ 3500 6700
Wire Wire Line
	3500 6700 3600 6700
Wire Wire Line
	3600 6900 3600 7200
Text GLabel 4500 6800 2    50   Output ~ 0
CUR_V
Wire Wire Line
	3900 7100 4350 7100
Wire Wire Line
	4450 7200 4450 7150
Wire Wire Line
	3600 7200 4450 7200
Wire Wire Line
	4450 7200 4450 7250
Connection ~ 4450 7200
Wire Wire Line
	3800 7550 4450 7550
Wire Wire Line
	4450 6850 4450 6800
Wire Wire Line
	4450 6800 4500 6800
Wire Wire Line
	4450 6800 4200 6800
Connection ~ 4450 6800
Wire Wire Line
	4350 6350 4350 7100
$Comp
L power:+3.3V #PWR011
U 1 1 5F78699F
P 3800 6300
F 0 "#PWR011" H 3800 6150 50  0001 C CNN
F 1 "+3.3V" H 3800 6450 50  0000 C CNN
F 2 "" H 3800 6300 50  0001 C CNN
F 3 "" H 3800 6300 50  0001 C CNN
	1    3800 6300
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR012
U 1 1 5F7869A6
P 3800 7600
F 0 "#PWR012" H 3800 7350 50  0001 C CNN
F 1 "Earth" H 3800 7450 50  0001 C CNN
F 2 "" H 3800 7600 50  0001 C CNN
F 3 "~" H 3800 7600 50  0001 C CNN
	1    3800 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 7600 3800 7550
$Comp
L Amplifier_Operational:MAX4239AUT U5
U 1 1 5F7BA499
P 5800 6800
F 0 "U5" H 5750 7050 50  0000 L CNN
F 1 "MAX4239AUT" H 5750 6950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 5800 6800 50  0001 C CNN
F 3 "" H 5950 6950 50  0001 C CNN
F 4 "MAX4239AUT+T" H 5800 6800 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 5800 6800 50  0001 C CNN "Voltage Rating"
	1    5800 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R20
U 1 1 5F7BA49F
P 5100 6500
F 0 "R20" H 5168 6546 50  0000 L CNN
F 1 "5.1kΩ" H 5168 6455 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 5140 6490 50  0001 C CNN
F 3 "" H 5100 6500 50  0001 C CNN
F 4 "ERJ-1GNF5101C" H 5100 6500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 5100 6500 50  0001 C CNN "Voltage Rating"
	1    5100 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R21
U 1 1 5F7BA4A5
P 5100 6900
F 0 "R21" H 5168 6946 50  0000 L CNN
F 1 "976Ω" H 5168 6855 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 5140 6890 50  0001 C CNN
F 3 "" H 5100 6900 50  0001 C CNN
F 4 "ERJ-1GNF9760C" H 5100 6900 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 5100 6900 50  0001 C CNN "Voltage Rating"
	1    5100 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R19
U 1 1 5F7BA4AB
P 4900 6700
F 0 "R19" V 4800 6700 50  0000 C CNN
F 1 "86.6Ω" V 5000 6700 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 4940 6690 50  0001 C CNN
F 3 "" H 4900 6700 50  0001 C CNN
F 4 "ERJ-1GNF86R6C" H 4900 6700 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4900 6700 50  0001 C CNN "Voltage Rating"
	1    4900 6700
	0    1    1    0   
$EndComp
$Comp
L Device:C C16
U 1 1 5F7BA4B1
P 5400 7350
F 0 "C16" H 5400 7450 50  0000 L CNN
F 1 "6.8nF" H 5400 7250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 5438 7200 50  0001 C CNN
F 3 "" H 5400 7350 50  0001 C CNN
F 4 "25V" H 5400 7350 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 5400 7350 50  0001 C CNN "Manufacturer Part #"
	1    5400 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 6700 5100 6700
Wire Wire Line
	5100 6650 5100 6700
Connection ~ 5100 6700
Wire Wire Line
	5100 6700 5100 6750
Text GLabel 4750 6700 0    50   Input ~ 0
SHUNT_W
$Comp
L Device:R_US R23
U 1 1 5F7BA4BC
P 6350 7400
F 0 "R23" H 6418 7446 50  0000 L CNN
F 1 "470Ω" H 6418 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 6390 7390 50  0001 C CNN
F 3 "" H 6350 7400 50  0001 C CNN
F 4 "ERJ-1GNF4700C" H 6350 7400 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6350 7400 50  0001 C CNN "Voltage Rating"
	1    6350 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R22
U 1 1 5F7BA4C2
P 6350 7000
F 0 "R22" H 6418 7046 50  0000 L CNN
F 1 "15kΩ" H 6418 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 6390 6990 50  0001 C CNN
F 3 "" H 6350 7000 50  0001 C CNN
F 4 "ERJ-1GNF1502C" H 6350 7000 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6350 7000 50  0001 C CNN "Voltage Rating"
	1    6350 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 7050 5100 7550
Wire Wire Line
	5700 7100 5700 7550
Connection ~ 5700 7550
Wire Wire Line
	5100 7550 5400 7550
Wire Wire Line
	5100 6700 5400 6700
Wire Wire Line
	5400 7500 5400 7550
Connection ~ 5400 7550
Wire Wire Line
	5400 7550 5700 7550
Wire Wire Line
	5400 7200 5400 6700
Connection ~ 5400 6700
Wire Wire Line
	5400 6700 5500 6700
Wire Wire Line
	5500 6900 5500 7200
Text GLabel 6400 6800 2    50   Output ~ 0
CUR_W
Wire Wire Line
	5800 7100 6250 7100
Wire Wire Line
	6350 7200 6350 7150
Wire Wire Line
	5500 7200 6350 7200
Wire Wire Line
	6350 7200 6350 7250
Connection ~ 6350 7200
Wire Wire Line
	5700 7550 6350 7550
Wire Wire Line
	6350 6850 6350 6800
Wire Wire Line
	6350 6800 6400 6800
Wire Wire Line
	6350 6800 6100 6800
Connection ~ 6350 6800
Wire Wire Line
	6250 6350 6250 7100
$Comp
L power:+3.3V #PWR018
U 1 1 5F7BA4E4
P 5700 6300
F 0 "#PWR018" H 5700 6150 50  0001 C CNN
F 1 "+3.3V" H 5700 6450 50  0000 C CNN
F 2 "" H 5700 6300 50  0001 C CNN
F 3 "" H 5700 6300 50  0001 C CNN
	1    5700 6300
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR019
U 1 1 5F7BA4EB
P 5700 7600
F 0 "#PWR019" H 5700 7350 50  0001 C CNN
F 1 "Earth" H 5700 7450 50  0001 C CNN
F 2 "" H 5700 7600 50  0001 C CNN
F 3 "~" H 5700 7600 50  0001 C CNN
	1    5700 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 7600 5700 7550
Text Notes 550  6150 0    50   ~ 0
CURRENT SENSE AMPLIFIERS
$Comp
L Device:R_US R3
U 1 1 5F892A6E
P 1200 5200
F 0 "R3" V 1100 5200 50  0000 C CNN
F 1 "1mΩ" V 1300 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1240 5190 50  0001 C CNN
F 3 "" H 1200 5200 50  0001 C CNN
F 4 "CSR1206-0R001F1" H 1200 5200 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1200 5200 50  0001 C CNN "Voltage Rating"
	1    1200 5200
	0    1    1    0   
$EndComp
Text GLabel 950  5200 0    50   UnSpc ~ 0
SHUNT_U
$Comp
L Device:R_US R2
U 1 1 5F905093
P 1150 5500
F 0 "R2" V 1050 5500 50  0000 C CNN
F 1 "1.5kΩ" V 1250 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 1190 5490 50  0001 C CNN
F 3 "" H 1150 5500 50  0001 C CNN
F 4 "ERJ-1GNF1501C" H 1150 5500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1150 5500 50  0001 C CNN "Voltage Rating"
	1    1150 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	1000 5200 1000 5500
Text Notes 550  5050 0    50   ~ 0
CURRENT SHUNTS & OCP COMPARATOR FILTERS
Wire Wire Line
	1100 1550 1000 1550
Wire Wire Line
	1100 1500 1100 1550
Wire Wire Line
	1200 1500 1100 1500
Wire Wire Line
	1100 1400 1200 1400
Wire Wire Line
	1100 1350 1000 1350
Wire Wire Line
	1100 1400 1100 1350
Text Notes 5650 3250 0    50   ~ 0
FET DRIVER BYPASS CAPACITORS
Text Notes 3500 600  0    50   ~ 0
FET DRIVER\n
Wire Wire Line
	6900 3550 7000 3550
Wire Wire Line
	6600 3550 6600 3500
Wire Wire Line
	6200 3550 6300 3550
Connection ~ 6200 3550
Wire Wire Line
	6200 3550 6200 3500
Wire Wire Line
	6100 3550 6200 3550
Wire Wire Line
	5800 3550 5800 3500
$Comp
L power:VBUS #PWR028
U 1 1 5F4CF6CC
P 7000 3500
F 0 "#PWR028" H 7000 3350 50  0001 C CNN
F 1 "VBUS" H 7000 3650 50  0000 C CNN
F 2 "" H 7000 3500 50  0001 C CNN
F 3 "" H 7000 3500 50  0001 C CNN
	1    7000 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR025
U 1 1 5F4CE3F6
P 6600 3500
F 0 "#PWR025" H 6600 3350 50  0001 C CNN
F 1 "+12V" H 6600 3650 50  0000 C CNN
F 2 "" H 6600 3500 50  0001 C CNN
F 3 "" H 6600 3500 50  0001 C CNN
	1    6600 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 5F4CC9A8
P 6200 3500
F 0 "#PWR023" H 6200 3350 50  0001 C CNN
F 1 "+5V" H 6200 3650 50  0000 C CNN
F 2 "" H 6200 3500 50  0001 C CNN
F 3 "" H 6200 3500 50  0001 C CNN
	1    6200 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 5F4CB5BF
P 5800 3500
F 0 "#PWR020" H 5800 3350 50  0001 C CNN
F 1 "+3.3V" H 5800 3650 50  0000 C CNN
F 2 "" H 5800 3500 50  0001 C CNN
F 3 "" H 5800 3500 50  0001 C CNN
	1    5800 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C26
U 1 1 5F4CB34C
P 6900 3700
F 0 "C26" H 6900 3800 50  0000 L CNN
F 1 "1µF" H 6900 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6938 3550 50  0001 C CNN
F 3 "" H 6900 3700 50  0001 C CNN
F 4 "50V" H 6900 3700 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6900 3700 50  0001 C CNN "Manufacturer Part #"
	1    6900 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C27
U 1 1 5F4CAE8E
P 7100 3700
F 0 "C27" H 7100 3800 50  0000 L CNN
F 1 "100nF" H 7100 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7138 3550 50  0001 C CNN
F 3 "" H 7100 3700 50  0001 C CNN
F 4 "50V" H 7100 3700 50  0001 C CNN "Voltage Rating"
F 5 "C1005X7R1H104K050BB" H 7100 3700 50  0001 C CNN "Manufacturer Part #"
	1    7100 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 5F4CAC58
P 6600 3700
F 0 "C22" H 6600 3800 50  0000 L CNN
F 1 "10µF" H 6600 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6638 3550 50  0001 C CNN
F 3 "" H 6600 3700 50  0001 C CNN
F 4 "25V" H 6600 3700 50  0001 C CNN "Voltage Rating"
F 5 "C1608X5R1E106M080AC" H 6600 3700 50  0001 C CNN "Manufacturer Part #"
	1    6600 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5F4CAA4D
P 6300 3700
F 0 "C19" H 6300 3800 50  0000 L CNN
F 1 "1µF" H 6300 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6338 3550 50  0001 C CNN
F 3 "" H 6300 3700 50  0001 C CNN
F 4 "50V" H 6300 3700 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6300 3700 50  0001 C CNN "Manufacturer Part #"
	1    6300 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C18
U 1 1 5F4CA4EA
P 6100 3700
F 0 "C18" H 6100 3800 50  0000 L CNN
F 1 "1µF" H 6100 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6138 3550 50  0001 C CNN
F 3 "" H 6100 3700 50  0001 C CNN
F 4 "50V" H 6100 3700 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6100 3700 50  0001 C CNN "Manufacturer Part #"
	1    6100 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5F4C9EF6
P 5800 3700
F 0 "C17" H 5800 3800 50  0000 L CNN
F 1 "100nF" H 5800 3600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 5838 3550 50  0001 C CNN
F 3 "" H 5800 3700 50  0001 C CNN
F 4 "25V" H 5800 3700 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 5800 3700 50  0001 C CNN "Manufacturer Part #"
	1    5800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2600 7050 2600
Wire Wire Line
	7050 2600 7050 2550
$Comp
L power:+3.3V #PWR029
U 1 1 5F49BD7C
P 7050 2550
F 0 "#PWR029" H 7050 2400 50  0001 C CNN
F 1 "+3.3V" H 7050 2700 50  0000 C CNN
F 2 "" H 7050 2550 50  0001 C CNN
F 3 "" H 7050 2550 50  0001 C CNN
	1    7050 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2700 6400 2700
$Comp
L power:Earth #PWR027
U 1 1 5F493A9F
P 6850 2700
F 0 "#PWR027" H 6850 2450 50  0001 C CNN
F 1 "Earth" H 6850 2550 50  0001 C CNN
F 2 "" H 6850 2700 50  0001 C CNN
F 3 "~" H 6850 2700 50  0001 C CNN
	1    6850 2700
	1    0    0    -1  
$EndComp
Text GLabel 6400 2500 2    50   Output ~ 0
DRV_FAULT
Text GLabel 6400 2400 2    50   Input ~ 0
DRV_EN
Wire Wire Line
	7200 1350 7200 1900
Connection ~ 7200 1900
Wire Wire Line
	7200 2300 7200 1900
Wire Wire Line
	6400 2300 7200 2300
Wire Wire Line
	6450 2100 6400 2100
$Comp
L Device:C C21
U 1 1 5F488471
P 6600 2100
F 0 "C21" V 6550 2250 50  0000 C CNN
F 1 "22nF" V 6650 2250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6638 1950 50  0001 C CNN
F 3 "" H 6600 2100 50  0001 C CNN
F 4 "100V" H 6600 2100 50  0001 C CNN "Voltage Rating"
F 5 "C1608X7R2A223K080AA" H 6600 2100 50  0001 C CNN "Manufacturer Part #"
	1    6600 2100
	0    1    1    0   
$EndComp
$Comp
L power:VBUS #PWR030
U 1 1 5F4831B0
P 7200 1350
F 0 "#PWR030" H 7200 1200 50  0001 C CNN
F 1 "VBUS" H 7200 1500 50  0000 C CNN
F 2 "" H 7200 1350 50  0001 C CNN
F 3 "" H 7200 1350 50  0001 C CNN
	1    7200 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1900 6400 1900
$Comp
L Device:C C20
U 1 1 5F47F307
P 6600 1900
F 0 "C20" V 6550 2050 50  0000 C CNN
F 1 "100nF" V 6650 2100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 6638 1750 50  0001 C CNN
F 3 "" H 6600 1900 50  0001 C CNN
F 4 "25V" H 6600 1900 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 6600 1900 50  0001 C CNN "Manufacturer Part #"
	1    6600 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 2500 4200 2500
Wire Wire Line
	3750 1600 4200 1600
Wire Wire Line
	3750 1550 3750 1600
$Comp
L power:+3.3V #PWR09
U 1 1 5F470C44
P 3600 1650
F 0 "#PWR09" H 3600 1500 50  0001 C CNN
F 1 "+3.3V" H 3600 1800 50  0000 C CNN
F 2 "" H 3600 1650 50  0001 C CNN
F 3 "" H 3600 1650 50  0001 C CNN
	1    3600 1650
	1    0    0    -1  
$EndComp
Text GLabel 6400 2800 2    50   Input ~ 0
PWM_W_L
Text GLabel 4200 2800 0    50   Input ~ 0
PWM_W_H
Text GLabel 4200 2700 0    50   Input ~ 0
PWM_V_L
Text GLabel 4200 2600 0    50   Input ~ 0
PWM_V_H
Text GLabel 4200 2400 0    50   Input ~ 0
PWM_U_L
Text GLabel 4200 2300 0    50   Input ~ 0
PWM_U_H
Text GLabel 4200 2200 0    50   Output ~ 0
SPI_SDO
Text GLabel 4200 2100 0    50   Input ~ 0
SPI_SDI
Text GLabel 4200 2000 0    50   Input ~ 0
SPI_SCK
Wire Wire Line
	4100 1700 4200 1700
$Comp
L power:Earth #PWR026
U 1 1 5F4356FF
P 6650 1000
F 0 "#PWR026" H 6650 750 50  0001 C CNN
F 1 "Earth" H 6650 850 50  0001 C CNN
F 2 "" H 6650 1000 50  0001 C CNN
F 3 "~" H 6650 1000 50  0001 C CNN
	1    6650 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 1000 6400 1000
Wire Wire Line
	3900 1450 3900 1500
$Comp
L power:+5V #PWR010
U 1 1 5F465222
P 3750 1550
F 0 "#PWR010" H 3750 1400 50  0001 C CNN
F 1 "+5V" H 3750 1700 50  0000 C CNN
F 2 "" H 3750 1550 50  0001 C CNN
F 3 "" H 3750 1550 50  0001 C CNN
	1    3750 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1500 4200 1500
$Comp
L power:+12V #PWR013
U 1 1 5F45F829
P 3900 1450
F 0 "#PWR013" H 3900 1300 50  0001 C CNN
F 1 "+12V" H 3900 1600 50  0000 C CNN
F 2 "" H 3900 1450 50  0001 C CNN
F 3 "" H 3900 1450 50  0001 C CNN
	1    3900 1450
	1    0    0    -1  
$EndComp
Text GLabel 4200 1900 0    50   Input ~ 0
SPI_CSN
Wire Wire Line
	6650 1650 6600 1650
Wire Wire Line
	6650 1450 6600 1450
Wire Wire Line
	6650 1200 6400 1200
NoConn ~ 4200 1100
$Comp
L TMC6100-LA:TMC6100-LA U4
U 1 1 5F3DD2A6
P 4200 1000
F 0 "U4" H 5300 1387 60  0000 C CNN
F 1 "TMC6100-LA" H 5300 1281 60  0000 C CNN
F 2 "TMC6100-LA:TMC6100-LA" H 5300 1240 60  0001 C CNN
F 3 "" H 4200 1000 60  0000 C CNN
F 4 "TMC6100-LA-T" H 4200 1000 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4200 1000 50  0001 C CNN "Voltage Rating"
	1    4200 1000
	1    0    0    -1  
$EndComp
NoConn ~ 4200 1800
$Comp
L power:Earth #PWR015
U 1 1 5F4345AE
P 4100 1700
F 0 "#PWR015" H 4100 1450 50  0001 C CNN
F 1 "Earth" H 4100 1550 50  0001 C CNN
F 2 "" H 4100 1700 50  0001 C CNN
F 3 "~" H 4100 1700 50  0001 C CNN
	1    4100 1700
	1    0    0    -1  
$EndComp
Text GLabel 4200 1400 0    50   Output ~ 0
LSU
Text GLabel 4200 1300 0    50   Output ~ 0
LSV
Text GLabel 4200 1200 0    50   Output ~ 0
LSW
Wire Wire Line
	6600 1600 6400 1600
Wire Wire Line
	6600 1650 6600 1600
Wire Wire Line
	6600 1500 6600 1450
Wire Wire Line
	6400 1500 6600 1500
Text GLabel 6400 1100 2    50   Output ~ 0
HSU
Text GLabel 6400 1700 2    50   Output ~ 0
HSW
Text GLabel 6400 1400 2    50   Output ~ 0
HSV
$Comp
L Device:C C24
U 1 1 5F40C21D
P 6800 1450
F 0 "C24" V 6850 1500 50  0000 L CNN
F 1 "1µF" V 6750 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6838 1300 50  0001 C CNN
F 3 "" H 6800 1450 50  0001 C CNN
F 4 "50V" H 6800 1450 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6800 1450 50  0001 C CNN "Manufacturer Part #"
	1    6800 1450
	0    -1   -1   0   
$EndComp
Text GLabel 6950 1450 2    50   UnSpc ~ 0
CV
$Comp
L Device:C C25
U 1 1 5F40BEFD
P 6800 1650
F 0 "C25" V 6850 1700 50  0000 L CNN
F 1 "1µF" V 6750 1700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6838 1500 50  0001 C CNN
F 3 "" H 6800 1650 50  0001 C CNN
F 4 "50V" H 6800 1650 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6800 1650 50  0001 C CNN "Manufacturer Part #"
	1    6800 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C23
U 1 1 5F40A651
P 6800 1200
F 0 "C23" V 6850 1250 50  0000 L CNN
F 1 "1µF" V 6750 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6838 1050 50  0001 C CNN
F 3 "" H 6800 1200 50  0001 C CNN
F 4 "50V" H 6800 1200 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 6800 1200 50  0001 C CNN "Manufacturer Part #"
	1    6800 1200
	0    -1   -1   0   
$EndComp
Text GLabel 6950 1650 2    50   UnSpc ~ 0
CW
Text GLabel 6950 1200 2    50   UnSpc ~ 0
CU
Text GLabel 6400 1800 2    50   UnSpc ~ 0
CW
Text GLabel 6400 1300 2    50   UnSpc ~ 0
CV
Text GLabel 4200 1000 0    50   UnSpc ~ 0
CU
Text Notes 550  4200 0    50   ~ 0
MCU BYPASS CAPACITORS
Wire Wire Line
	1750 4750 1750 4700
$Comp
L power:Earth #PWR03
U 1 1 5F417CA3
P 1750 4750
F 0 "#PWR03" H 1750 4500 50  0001 C CNN
F 1 "Earth" H 1750 4600 50  0001 C CNN
F 2 "" H 1750 4750 50  0001 C CNN
F 3 "~" H 1750 4750 50  0001 C CNN
	1    1750 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4700 2750 4700
Connection ~ 2500 4700
Wire Wire Line
	2250 4700 2500 4700
Connection ~ 2250 4700
Wire Wire Line
	2000 4700 2250 4700
Connection ~ 2000 4700
Wire Wire Line
	1750 4700 2000 4700
Connection ~ 1750 4700
Wire Wire Line
	1500 4700 1750 4700
Connection ~ 1500 4700
Wire Wire Line
	1250 4700 1500 4700
Connection ~ 1250 4700
Wire Wire Line
	1000 4700 1250 4700
Connection ~ 1000 4700
Wire Wire Line
	750  4700 1000 4700
$Comp
L power:+3.3V #PWR02
U 1 1 5F4169A0
P 1750 4350
F 0 "#PWR02" H 1750 4200 50  0001 C CNN
F 1 "+3.3V" H 1765 4523 50  0000 C CNN
F 2 "" H 1750 4350 50  0001 C CNN
F 3 "" H 1750 4350 50  0001 C CNN
	1    1750 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4400 2750 4400
Connection ~ 2500 4400
Wire Wire Line
	2250 4400 2500 4400
Connection ~ 2250 4400
Wire Wire Line
	2000 4400 2250 4400
Connection ~ 2000 4400
Wire Wire Line
	1750 4400 2000 4400
Connection ~ 1750 4400
Wire Wire Line
	1500 4400 1750 4400
Connection ~ 1500 4400
Wire Wire Line
	1250 4400 1500 4400
Connection ~ 1250 4400
Wire Wire Line
	1000 4400 1250 4400
Connection ~ 1000 4400
Wire Wire Line
	750  4400 1000 4400
$Comp
L MCU_ST_STM32F3:STM32F303CBTx U1
U 1 1 5F3D0509
P 1900 2300
F 0 "U1" H 1350 3900 50  0000 C CNN
F 1 "STM32F303CBT7" H 1350 3800 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 1300 900 50  0001 R CNN
F 3 "" H 1900 2300 50  0001 C CNN
F 4 "STM32F303CBT7" H 1900 2300 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1900 2300 50  0001 C CNN "Voltage Rating"
	1    1900 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5F3F47FE
P 1900 700
F 0 "#PWR05" H 1900 550 50  0001 C CNN
F 1 "+3.3V" H 1900 850 50  0000 C CNN
F 2 "" H 1900 700 50  0001 C CNN
F 3 "" H 1900 700 50  0001 C CNN
	1    1900 700 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5F406B03
P 2750 4550
F 0 "C13" H 2750 4650 50  0000 L CNN
F 1 "10µF" H 2750 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2788 4400 50  0001 C CNN
F 3 "" H 2750 4550 50  0001 C CNN
F 4 "25V" H 2750 4550 50  0001 C CNN "Voltage Rating"
F 5 "C1608X5R1E106M080AC" H 2750 4550 50  0001 C CNN "Manufacturer Part #"
	1    2750 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5F4066E1
P 2500 4550
F 0 "C11" H 2500 4650 50  0000 L CNN
F 1 "1µF" H 2500 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2538 4400 50  0001 C CNN
F 3 "" H 2500 4550 50  0001 C CNN
F 4 "50V" H 2500 4550 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 2500 4550 50  0001 C CNN "Manufacturer Part #"
	1    2500 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5F4061BD
P 2250 4550
F 0 "C10" H 2250 4650 50  0000 L CNN
F 1 "6.8nF" H 2250 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 2288 4400 50  0001 C CNN
F 3 "" H 2250 4550 50  0001 C CNN
F 4 "25V" H 2250 4550 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 2250 4550 50  0001 C CNN "Manufacturer Part #"
	1    2250 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5F405820
P 1750 4550
F 0 "C8" H 1750 4650 50  0000 L CNN
F 1 "100nF" H 1750 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1788 4400 50  0001 C CNN
F 3 "" H 1750 4550 50  0001 C CNN
F 4 "25V" H 1750 4550 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 1750 4550 50  0001 C CNN "Manufacturer Part #"
	1    1750 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5F40581A
P 2000 4550
F 0 "C9" H 2000 4650 50  0000 L CNN
F 1 "6.8nF" H 2000 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 2038 4400 50  0001 C CNN
F 3 "" H 2000 4550 50  0001 C CNN
F 4 "25V" H 2000 4550 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 2000 4550 50  0001 C CNN "Manufacturer Part #"
	1    2000 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5F404A66
P 1250 4550
F 0 "C4" H 1250 4650 50  0000 L CNN
F 1 "100nF" H 1250 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1288 4400 50  0001 C CNN
F 3 "" H 1250 4550 50  0001 C CNN
F 4 "25V" H 1250 4550 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 1250 4550 50  0001 C CNN "Manufacturer Part #"
	1    1250 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F404A60
P 1500 4550
F 0 "C5" H 1500 4650 50  0000 L CNN
F 1 "6.8nF" H 1500 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1538 4400 50  0001 C CNN
F 3 "" H 1500 4550 50  0001 C CNN
F 4 "25V" H 1500 4550 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 1500 4550 50  0001 C CNN "Manufacturer Part #"
	1    1500 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F3F7558
P 750 4550
F 0 "C1" H 750 4650 50  0000 L CNN
F 1 "100nF" H 750 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 788 4400 50  0001 C CNN
F 3 "" H 750 4550 50  0001 C CNN
F 4 "25V" H 750 4550 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 750 4550 50  0001 C CNN "Manufacturer Part #"
	1    750  4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F3F99E2
P 1000 4550
F 0 "C3" H 1000 4650 50  0000 L CNN
F 1 "6.8nF" H 1000 4450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1038 4400 50  0001 C CNN
F 3 "" H 1000 4550 50  0001 C CNN
F 4 "25V" H 1000 4550 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 1000 4550 50  0001 C CNN "Manufacturer Part #"
	1    1000 4550
	1    0    0    -1  
$EndComp
Text Notes 550  600  0    50   ~ 0
MCU
Connection ~ 800  1450
Wire Wire Line
	800  1500 800  1450
Wire Wire Line
	1850 3850 1900 3850
Connection ~ 1850 3850
Wire Wire Line
	1850 3850 1850 3900
Wire Wire Line
	1900 3850 2000 3850
Connection ~ 1900 3850
Wire Wire Line
	1900 3800 1900 3850
Wire Wire Line
	1800 3850 1850 3850
Connection ~ 1800 3850
Wire Wire Line
	1800 3800 1800 3850
Wire Wire Line
	2000 3850 2000 3800
Wire Wire Line
	1700 3850 1800 3850
Wire Wire Line
	1700 3800 1700 3850
$Comp
L power:Earth #PWR04
U 1 1 5F3E2CF2
P 1850 3900
F 0 "#PWR04" H 1850 3650 50  0001 C CNN
F 1 "Earth" H 1850 3750 50  0001 C CNN
F 2 "" H 1850 3900 50  0001 C CNN
F 3 "~" H 1850 3900 50  0001 C CNN
	1    1850 3900
	1    0    0    -1  
$EndComp
Text GLabel 2500 3600 2    50   Output ~ 0
SPI_CSN
Text GLabel 2500 3500 2    50   Input ~ 0
SWCLK
Text GLabel 2500 3400 2    50   BiDi ~ 0
SWDIO
Text GLabel 2500 3200 2    50   Output ~ 0
PWM_U_L
Text GLabel 2500 3100 2    50   Output ~ 0
PWM_W_H
Text GLabel 2500 3000 2    50   Output ~ 0
PWM_V_H
Text GLabel 2500 2900 2    50   Output ~ 0
PWM_U_H
Text GLabel 2500 2800 2    50   Input ~ 0
OCP_COMP_U
Text GLabel 2500 2300 2    50   Input ~ 0
CUR_W
Text GLabel 2500 2200 2    50   Input ~ 0
CUR_V
Text GLabel 2500 2100 2    50   Input ~ 0
CUR_U
NoConn ~ 2500 3300
NoConn ~ 2500 2700
NoConn ~ 2500 2600
NoConn ~ 2500 2500
NoConn ~ 2500 2400
Text GLabel 1200 3600 0    50   Output ~ 0
PWM_W_L
Text GLabel 1200 3500 0    50   Output ~ 0
PWM_V_L
Text GLabel 1200 3400 0    50   Input ~ 0
VBUS_SENSE
Text GLabel 1200 3300 0    50   Input ~ 0
DRV_FAULT
Text GLabel 1200 3200 0    50   Input ~ 0
OCP_COMP_W
Text GLabel 1200 3100 0    50   Output ~ 0
DRV_EN
NoConn ~ 1200 3000
NoConn ~ 1200 2900
Text GLabel 1200 2800 0    50   Input ~ 0
UART_RX_DSHOT
Text GLabel 1200 2700 0    50   Output ~ 0
UART_TX
Text GLabel 1200 2600 0    50   Output ~ 0
SPI_SDI
Text GLabel 1200 2500 0    50   Input ~ 0
SPI_SDO
Text GLabel 1200 2400 0    50   Output ~ 0
SPI_SCK
$Comp
L power:Earth #PWR01
U 1 1 5F3DD560
P 800 1500
F 0 "#PWR01" H 800 1250 50  0001 C CNN
F 1 "Earth" H 800 1350 50  0001 C CNN
F 2 "" H 800 1500 50  0001 C CNN
F 3 "~" H 800 1500 50  0001 C CNN
	1    800  1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  1450 900  1450
Wire Wire Line
	850  1000 800  1000
Wire Wire Line
	1200 1000 1150 1000
Text GLabel 1200 2100 0    50   Input ~ 0
OCP_COMP_V
NoConn ~ 1200 2300
NoConn ~ 1200 2200
NoConn ~ 1200 1900
NoConn ~ 1200 1800
NoConn ~ 1200 1700
$Comp
L Device:Crystal_GND2_Small Y1
U 1 1 5F3D462E
P 1000 1450
F 0 "Y1" V 850 1400 50  0000 L CNN
F 1 "8MHz" V 1150 1350 50  0000 L CNN
F 2 "Crystal:Resonator_SMD_muRata_CSTxExxV-3Pin_3.0x1.1mm" H 1000 1450 50  0001 C CNN
F 3 "" H 1000 1450 50  0001 C CNN
F 4 "CSTNE8M00GH5C000R0" H 1000 1450 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 1000 1450 50  0001 C CNN "Voltage Rating"
	1    1000 1450
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5F3D5114
P 1000 1000
F 0 "C2" V 850 1000 50  0000 C CNN
F 1 "100nF" V 1150 1000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1038 850 50  0001 C CNN
F 3 "" H 1000 1000 50  0001 C CNN
F 4 "25V" V 1000 1000 50  0001 C CNN "Voltage Rating"
F 5 "C0603X5R1E104K030BB" H 1000 1000 50  0001 C CNN "Manufacturer Part #"
	1    1000 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	1000 5200 950  5200
Text GLabel 1400 5700 2    50   Output ~ 0
OCP_COMP_U
Wire Wire Line
	1300 5500 1350 5500
Connection ~ 1350 5500
Wire Wire Line
	1350 5500 1400 5500
$Comp
L Device:R_US R7
U 1 1 5FCF0848
P 2200 5200
F 0 "R7" V 2100 5200 50  0000 C CNN
F 1 "1mΩ" V 2300 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2240 5190 50  0001 C CNN
F 3 "" H 2200 5200 50  0001 C CNN
F 4 "CSR1206-0R001F1" H 2200 5200 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 2200 5200 50  0001 C CNN "Voltage Rating"
	1    2200 5200
	0    1    1    0   
$EndComp
Text GLabel 1950 5200 0    50   UnSpc ~ 0
SHUNT_V
$Comp
L Device:R_US R6
U 1 1 5FCF084F
P 2150 5500
F 0 "R6" V 2050 5500 50  0000 C CNN
F 1 "1.5kΩ" V 2250 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 2190 5490 50  0001 C CNN
F 3 "" H 2150 5500 50  0001 C CNN
F 4 "ERJ-1GNF1501C" H 2150 5500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 2150 5500 50  0001 C CNN "Voltage Rating"
	1    2150 5500
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 5FCF0855
P 2550 5500
F 0 "C12" V 2500 5650 50  0000 C CNN
F 1 "6.8nF" V 2600 5650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 2588 5350 50  0001 C CNN
F 3 "" H 2550 5500 50  0001 C CNN
F 4 "25V" H 2550 5500 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 2550 5500 50  0001 C CNN "Manufacturer Part #"
	1    2550 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 5200 2000 5500
Wire Wire Line
	2000 5200 1950 5200
Text GLabel 2400 5700 2    50   Output ~ 0
OCP_COMP_V
Wire Wire Line
	2300 5500 2350 5500
Connection ~ 2350 5500
Wire Wire Line
	2350 5500 2400 5500
$Comp
L Device:R_US R12
U 1 1 5FD05B7D
P 3200 5200
F 0 "R12" V 3100 5200 50  0000 C CNN
F 1 "1mΩ" V 3300 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3240 5190 50  0001 C CNN
F 3 "" H 3200 5200 50  0001 C CNN
F 4 "CSR1206-0R001F1" H 3200 5200 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3200 5200 50  0001 C CNN "Voltage Rating"
	1    3200 5200
	0    1    1    0   
$EndComp
Text GLabel 2950 5200 0    50   UnSpc ~ 0
SHUNT_W
$Comp
L Device:R_US R11
U 1 1 5FD05B84
P 3150 5500
F 0 "R11" V 3050 5500 50  0000 C CNN
F 1 "1.5kΩ" V 3250 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 3190 5490 50  0001 C CNN
F 3 "" H 3150 5500 50  0001 C CNN
F 4 "ERJ-1GNF1501C" H 3150 5500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 3150 5500 50  0001 C CNN "Voltage Rating"
	1    3150 5500
	0    1    1    0   
$EndComp
$Comp
L Device:C C15
U 1 1 5FD05B8A
P 3550 5500
F 0 "C15" V 3500 5650 50  0000 C CNN
F 1 "6.8nF" V 3600 5650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 3588 5350 50  0001 C CNN
F 3 "" H 3550 5500 50  0001 C CNN
F 4 "25V" H 3550 5500 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 3550 5500 50  0001 C CNN "Manufacturer Part #"
	1    3550 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	3000 5200 3000 5500
Wire Wire Line
	3000 5200 2950 5200
Text GLabel 3400 5700 2    50   Output ~ 0
OCP_COMP_W
Wire Wire Line
	3300 5500 3350 5500
Connection ~ 3350 5500
Wire Wire Line
	3350 5500 3400 5500
$Comp
L power:Earth #PWR08
U 1 1 5FD4AAC3
P 2950 5850
F 0 "#PWR08" H 2950 5600 50  0001 C CNN
F 1 "Earth" H 2950 5700 50  0001 C CNN
F 2 "" H 2950 5850 50  0001 C CNN
F 3 "~" H 2950 5850 50  0001 C CNN
	1    2950 5850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSC026N08NS5 Q1
U 1 1 5F4460B2
P 9200 2700
F 0 "Q1" H 9404 2746 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 2655 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 2625 50  0001 L CIN
F 3 "" V 9200 2700 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 2700 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 2700 50  0001 C CNN "Voltage Rating"
	1    9200 2700
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR033
U 1 1 5F44B979
P 9300 2400
F 0 "#PWR033" H 9300 2250 50  0001 C CNN
F 1 "VBUS" H 9300 2550 50  0000 C CNN
F 2 "" H 9300 2400 50  0001 C CNN
F 3 "" H 9300 2400 50  0001 C CNN
	1    9300 2400
	1    0    0    -1  
$EndComp
Text GLabel 8950 2700 0    50   Input ~ 0
HSU
Wire Wire Line
	8950 2700 9000 2700
$Comp
L Transistor_FET:BSC026N08NS5 Q2
U 1 1 5F47E07B
P 9200 3200
F 0 "Q2" H 9404 3246 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 3155 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 3125 50  0001 L CIN
F 3 "" V 9200 3200 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 3200 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 3200 50  0001 C CNN "Voltage Rating"
	1    9200 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 2900 9300 2950
Text GLabel 8950 3200 0    50   Input ~ 0
LSU
Wire Wire Line
	8950 3200 9000 3200
Text GLabel 9600 2950 2    50   Output ~ 0
U
Connection ~ 9300 2950
Wire Wire Line
	9300 2950 9300 3000
Text GLabel 9350 3450 2    50   UnSpc ~ 0
SHUNT_U
Wire Wire Line
	1050 5200 1000 5200
Connection ~ 1000 5200
Wire Wire Line
	2050 5200 2000 5200
Connection ~ 2000 5200
Wire Wire Line
	3050 5200 3000 5200
Connection ~ 3000 5200
Wire Wire Line
	2400 5200 2350 5200
Wire Wire Line
	3400 5200 3350 5200
Wire Wire Line
	1350 5200 1400 5200
Wire Wire Line
	9300 3450 9300 3400
$Comp
L Transistor_FET:BSC026N08NS5 Q3
U 1 1 5F5E420E
P 9200 4000
F 0 "Q3" H 9404 4046 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 3955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 3925 50  0001 L CIN
F 3 "" V 9200 4000 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 4000 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 4000 50  0001 C CNN "Voltage Rating"
	1    9200 4000
	1    0    0    -1  
$EndComp
Text GLabel 8950 4000 0    50   Input ~ 0
HSV
Wire Wire Line
	8950 4000 9000 4000
$Comp
L Transistor_FET:BSC026N08NS5 Q4
U 1 1 5F5E4223
P 9200 4500
F 0 "Q4" H 9404 4546 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 4455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 4425 50  0001 L CIN
F 3 "" V 9200 4500 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 4500 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 4500 50  0001 C CNN "Voltage Rating"
	1    9200 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 4200 9300 4250
Text GLabel 8950 4500 0    50   Input ~ 0
LSV
Wire Wire Line
	8950 4500 9000 4500
Text GLabel 9600 4250 2    50   Output ~ 0
V
Connection ~ 9300 4250
Wire Wire Line
	9300 4250 9300 4300
Text GLabel 9350 4750 2    50   UnSpc ~ 0
SHUNT_V
Wire Wire Line
	9300 4750 9300 4700
Wire Wire Line
	9300 3450 9350 3450
Wire Wire Line
	9350 4750 9300 4750
$Comp
L Transistor_FET:BSC026N08NS5 Q5
U 1 1 5F6A1A2C
P 9200 5300
F 0 "Q5" H 9404 5346 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 5255 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 5225 50  0001 L CIN
F 3 "" V 9200 5300 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 5300 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 5300 50  0001 C CNN "Voltage Rating"
	1    9200 5300
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR035
U 1 1 5F6A1A32
P 9300 5000
F 0 "#PWR035" H 9300 4850 50  0001 C CNN
F 1 "VBUS" H 9300 5150 50  0000 C CNN
F 2 "" H 9300 5000 50  0001 C CNN
F 3 "" H 9300 5000 50  0001 C CNN
	1    9300 5000
	1    0    0    -1  
$EndComp
Text GLabel 8950 5300 0    50   Input ~ 0
HSW
Wire Wire Line
	8950 5300 9000 5300
$Comp
L Transistor_FET:BSC026N08NS5 Q6
U 1 1 5F6A1A41
P 9200 5800
F 0 "Q6" H 9404 5846 50  0000 L CNN
F 1 "BSC010N04LSI" H 9404 5755 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 9400 5725 50  0001 L CIN
F 3 "" V 9200 5800 50  0001 L CNN
F 4 "BSC010N04LSIATMA1" H 9200 5800 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 9200 5800 50  0001 C CNN "Voltage Rating"
	1    9200 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 5500 9300 5550
Text GLabel 8950 5800 0    50   Input ~ 0
LSW
Wire Wire Line
	8950 5800 9000 5800
Text GLabel 9600 5550 2    50   Output ~ 0
W
Connection ~ 9300 5550
Wire Wire Line
	9300 5550 9300 5600
Text GLabel 9350 6050 2    50   UnSpc ~ 0
SHUNT_W
Wire Wire Line
	9300 6050 9300 6000
Wire Wire Line
	9350 6050 9300 6050
Text Notes 8750 2200 0    50   ~ 0
POWER STAGE
Wire Wire Line
	2450 6350 1900 6350
Wire Wire Line
	1900 6300 1900 6350
Connection ~ 1900 6350
Wire Wire Line
	1900 6350 1300 6350
Wire Wire Line
	1900 6350 1900 6500
Wire Wire Line
	4350 6350 3800 6350
Wire Wire Line
	3800 6300 3800 6350
Connection ~ 3800 6350
Wire Wire Line
	3800 6350 3200 6350
Wire Wire Line
	3800 6350 3800 6500
Wire Wire Line
	6250 6350 5700 6350
Wire Wire Line
	5700 6300 5700 6350
Connection ~ 5700 6350
Wire Wire Line
	5700 6350 5100 6350
Wire Wire Line
	5700 6350 5700 6500
Wire Wire Line
	1750 4400 1750 4350
Wire Wire Line
	3600 2500 3600 1650
$Comp
L Connector:Conn_01x01_Female J1
U 1 1 5F575983
P 6550 4250
F 0 "J1" H 6600 4300 50  0000 L CNN
F 1 "Pad_3x2mm" H 6600 4200 50  0000 L CNN
F 2 "Pads:Pad_3x2mm" H 6550 4250 50  0001 C CNN
F 3 "" H 6550 4250 50  0001 C CNN
F 4 "-" H 6550 4250 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 4250 50  0001 C CNN "Voltage Rating"
	1    6550 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J2
U 1 1 5F576313
P 6550 4450
F 0 "J2" H 6600 4500 50  0000 L CNN
F 1 "Pad_3x2mm" H 6600 4400 50  0000 L CNN
F 2 "Pads:Pad_3x2mm" H 6550 4450 50  0001 C CNN
F 3 "" H 6550 4450 50  0001 C CNN
F 4 "-" H 6550 4450 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 4450 50  0001 C CNN "Voltage Rating"
	1    6550 4450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 5F577E35
P 6550 4650
F 0 "J3" H 6600 4700 50  0000 L CNN
F 1 "Pad_3x2mm" H 6600 4600 50  0000 L CNN
F 2 "Pads:Pad_3x2mm" H 6550 4650 50  0001 C CNN
F 3 "" H 6550 4650 50  0001 C CNN
F 4 "-" H 6550 4650 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 4650 50  0001 C CNN "Voltage Rating"
	1    6550 4650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 5F578276
P 6550 4850
F 0 "J4" H 6600 4900 50  0000 L CNN
F 1 "Pad_3x2mm" H 6600 4800 50  0000 L CNN
F 2 "Pads:Pad_3x2mm" H 6550 4850 50  0001 C CNN
F 3 "" H 6550 4850 50  0001 C CNN
F 4 "-" H 6550 4850 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 4850 50  0001 C CNN "Voltage Rating"
	1    6550 4850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J5
U 1 1 5F578426
P 6550 5050
F 0 "J5" H 6600 5100 50  0000 L CNN
F 1 "Pad_3x2mm" H 6600 5000 50  0000 L CNN
F 2 "Pads:Pad_3x2mm" H 6550 5050 50  0001 C CNN
F 3 "" H 6550 5050 50  0001 C CNN
F 4 "-" H 6550 5050 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 5050 50  0001 C CNN "Voltage Rating"
	1    6550 5050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female J6
U 1 1 5F5787DA
P 6550 5250
F 0 "J6" H 6600 5300 50  0000 L CNN
F 1 "Pad_1.5mm_Square" H 6600 5200 50  0000 L CNN
F 2 "Pads:Pad_1.5mm_Square" H 6550 5250 50  0001 C CNN
F 3 "" H 6550 5250 50  0001 C CNN
F 4 "-" H 6550 5250 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6550 5250 50  0001 C CNN "Voltage Rating"
	1    6550 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Alt TP1
U 1 1 5F578D11
P 6350 5450
F 0 "TP1" V 6304 5638 50  0000 L CNN
F 1 "Pad_0.8mm_Square" V 6395 5638 50  0000 L CNN
F 2 "Pads:Pad_0.8mm_Square" H 6550 5450 50  0001 C CNN
F 3 "" H 6550 5450 50  0001 C CNN
F 4 "-" H 6350 5450 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6350 5450 50  0001 C CNN "Voltage Rating"
	1    6350 5450
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint_Alt TP2
U 1 1 5F57AB31
P 6350 5650
F 0 "TP2" V 6304 5838 50  0000 L CNN
F 1 "Pad_0.8mm_Square" V 6395 5838 50  0000 L CNN
F 2 "Pads:Pad_0.8mm_Square" H 6550 5650 50  0001 C CNN
F 3 "" H 6550 5650 50  0001 C CNN
F 4 "-" H 6350 5650 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6350 5650 50  0001 C CNN "Voltage Rating"
	1    6350 5650
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint_Alt TP3
U 1 1 5F598370
P 6350 5850
F 0 "TP3" V 6304 6038 50  0000 L CNN
F 1 "Pad_0.8mm_Square" V 6395 6038 50  0000 L CNN
F 2 "Pads:Pad_0.8mm_Square" H 6550 5850 50  0001 C CNN
F 3 "" H 6550 5850 50  0001 C CNN
F 4 "-" H 6350 5850 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 6350 5850 50  0001 C CNN "Voltage Rating"
	1    6350 5850
	0    1    1    0   
$EndComp
Text GLabel 6350 4250 0    50   UnSpc ~ 0
U
Text GLabel 6350 4450 0    50   UnSpc ~ 0
V
Text GLabel 6350 4650 0    50   UnSpc ~ 0
W
$Comp
L power:VBUS #PWR021
U 1 1 5F598AEF
P 6150 4800
F 0 "#PWR021" H 6150 4650 50  0001 C CNN
F 1 "VBUS" H 6150 4950 50  0000 C CNN
F 2 "" H 6150 4800 50  0001 C CNN
F 3 "" H 6150 4800 50  0001 C CNN
	1    6150 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4800 6150 4850
Wire Wire Line
	6150 4850 6350 4850
Text GLabel 6350 5250 0    50   UnSpc ~ 0
UART_RX_DSHOT
$Comp
L power:Earth #PWR022
U 1 1 5F5B6ABF
P 6150 5050
F 0 "#PWR022" H 6150 4800 50  0001 C CNN
F 1 "Earth" H 6150 4900 50  0001 C CNN
F 2 "" H 6150 5050 50  0001 C CNN
F 3 "~" H 6150 5050 50  0001 C CNN
	1    6150 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 5050 6150 5050
Text GLabel 6350 5450 0    50   UnSpc ~ 0
SWDIO
Text GLabel 6350 5650 0    50   UnSpc ~ 0
SWCLK
Text GLabel 6350 5850 0    50   UnSpc ~ 0
UART_TX
Text Notes 5650 4250 0    50   ~ 0
BREAKOUTS
Text Notes 5650 4350 0    50   ~ 0
& TEST POINTS
Wire Wire Line
	800  1000 800  1200
Wire Wire Line
	1200 1200 800  1200
Connection ~ 800  1200
Wire Wire Line
	800  1200 800  1450
$Comp
L power:VBUS #PWR034
U 1 1 5F5E4214
P 9300 3700
F 0 "#PWR034" H 9300 3550 50  0001 C CNN
F 1 "VBUS" H 9300 3850 50  0000 C CNN
F 2 "" H 9300 3700 50  0001 C CNN
F 3 "" H 9300 3700 50  0001 C CNN
	1    9300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 2400 9300 2450
Wire Wire Line
	9300 3700 9300 3750
Wire Wire Line
	9300 5000 9300 5050
Wire Wire Line
	10000 2450 9300 2450
Connection ~ 9300 2450
Wire Wire Line
	9300 2450 9300 2500
Wire Wire Line
	10000 3100 10000 3400
Wire Wire Line
	10000 2800 10000 2450
$Comp
L power:Earth #PWR036
U 1 1 5F818231
P 10000 3400
F 0 "#PWR036" H 10000 3150 50  0001 C CNN
F 1 "Earth" H 10000 3250 50  0001 C CNN
F 2 "" H 10000 3400 50  0001 C CNN
F 3 "~" H 10000 3400 50  0001 C CNN
	1    10000 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C31
U 1 1 5F6464B6
P 10000 2950
F 0 "C31" H 10000 3050 50  0000 L CNN
F 1 "1µF" H 10000 2850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 10038 2800 50  0001 C CNN
F 3 "" H 10000 2950 50  0001 C CNN
F 4 "50V" H 10000 2950 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 10000 2950 50  0001 C CNN "Manufacturer Part #"
	1    10000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3750 9300 3750
Wire Wire Line
	10000 4400 10000 4700
Wire Wire Line
	10000 4100 10000 3750
$Comp
L power:Earth #PWR037
U 1 1 5FA8D53E
P 10000 4700
F 0 "#PWR037" H 10000 4450 50  0001 C CNN
F 1 "Earth" H 10000 4550 50  0001 C CNN
F 2 "" H 10000 4700 50  0001 C CNN
F 3 "~" H 10000 4700 50  0001 C CNN
	1    10000 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C32
U 1 1 5FA8D544
P 10000 4250
F 0 "C32" H 10000 4350 50  0000 L CNN
F 1 "1µF" H 10000 4150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 10038 4100 50  0001 C CNN
F 3 "" H 10000 4250 50  0001 C CNN
F 4 "50V" H 10000 4250 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 10000 4250 50  0001 C CNN "Manufacturer Part #"
	1    10000 4250
	1    0    0    -1  
$EndComp
Connection ~ 9300 3750
Wire Wire Line
	9300 3750 9300 3800
Connection ~ 9300 5050
Wire Wire Line
	9300 5050 9300 5100
Wire Wire Line
	10000 5050 9300 5050
Wire Wire Line
	10000 5700 10000 6000
Wire Wire Line
	10000 5400 10000 5050
$Comp
L power:Earth #PWR038
U 1 1 5FAAFBDA
P 10000 6000
F 0 "#PWR038" H 10000 5750 50  0001 C CNN
F 1 "Earth" H 10000 5850 50  0001 C CNN
F 2 "" H 10000 6000 50  0001 C CNN
F 3 "~" H 10000 6000 50  0001 C CNN
	1    10000 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C33
U 1 1 5FAAFBE0
P 10000 5550
F 0 "C33" H 10000 5650 50  0000 L CNN
F 1 "1µF" H 10000 5450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 10038 5400 50  0001 C CNN
F 3 "" H 10000 5550 50  0001 C CNN
F 4 "50V" H 10000 5550 50  0001 C CNN "Voltage Rating"
F 5 "UMK105CBJ105MV-F" H 10000 5550 50  0001 C CNN "Manufacturer Part #"
	1    10000 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3500 7000 3550
Connection ~ 7000 3550
Wire Wire Line
	7000 3550 7100 3550
$Comp
L power:Earth #PWR017
U 1 1 5F645E4A
P 4300 4250
F 0 "#PWR017" H 4300 4000 50  0001 C CNN
F 1 "Earth" H 4300 4100 50  0001 C CNN
F 2 "" H 4300 4250 50  0001 C CNN
F 3 "~" H 4300 4250 50  0001 C CNN
	1    4300 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4250 4300 4200
Wire Wire Line
	4000 3850 4050 3850
Text GLabel 4000 3850 0    50   Output ~ 0
VBUS_SENSE
Text Notes 3500 3250 0    50   ~ 0
VBUS MONITORING
Wire Wire Line
	4050 3800 4050 3850
$Comp
L Device:R_US R16
U 1 1 5FE90D17
P 4300 4050
F 0 "R16" H 4368 4096 50  0000 L CNN
F 1 "15kΩ" H 4368 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 4340 4040 50  0001 C CNN
F 3 "" H 4300 4050 50  0001 C CNN
F 4 "ERJ-1GNF1502C" H 4300 4050 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4300 4050 50  0001 C CNN "Voltage Rating"
	1    4300 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3850 4300 3900
Connection ~ 4050 3850
Wire Wire Line
	4050 3850 4300 3850
Connection ~ 4300 3850
Wire Wire Line
	4300 3850 4300 3800
Wire Wire Line
	4300 3500 4300 3450
$Comp
L Device:R_US R15
U 1 1 5FE8F81E
P 4300 3650
F 0 "R15" H 4368 3696 50  0000 L CNN
F 1 "120kΩ" H 4368 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 4340 3640 50  0001 C CNN
F 3 "" H 4300 3650 50  0001 C CNN
F 4 "ERJ-1GNF1203C" H 4300 3650 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4300 3650 50  0001 C CNN "Voltage Rating"
	1    4300 3650
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR016
U 1 1 5FE93AE2
P 4300 3450
F 0 "#PWR016" H 4300 3300 50  0001 C CNN
F 1 "VBUS" H 4300 3600 50  0000 C CNN
F 2 "" H 4300 3450 50  0001 C CNN
F 3 "" H 4300 3450 50  0001 C CNN
	1    4300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3450 4050 3500
$Comp
L power:+3.3V #PWR014
U 1 1 5FE945C6
P 4050 3450
F 0 "#PWR014" H 4050 3300 50  0001 C CNN
F 1 "+3.3V" H 4050 3600 50  0000 C CNN
F 2 "" H 4050 3450 50  0001 C CNN
F 3 "" H 4050 3450 50  0001 C CNN
	1    4050 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 5FE91D15
P 4050 3650
F 0 "D1" V 4000 3500 50  0000 L CNN
F 1 "BAT30KFILM" V 4100 3150 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-523" H 4050 3650 50  0001 C CNN
F 3 "" H 4050 3650 50  0001 C CNN
F 4 "BAT30KFILM" H 4050 3650 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 4050 3650 50  0001 C CNN "Voltage Rating"
	1    4050 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 3850 6100 3850
Connection ~ 6100 3850
Wire Wire Line
	6100 3850 6300 3850
Connection ~ 6300 3850
Connection ~ 6600 3850
Wire Wire Line
	6600 3850 6900 3850
Connection ~ 6900 3850
Wire Wire Line
	6900 3850 7100 3850
Wire Wire Line
	9300 2950 9600 2950
Wire Wire Line
	9300 4250 9600 4250
Wire Wire Line
	9300 5550 9600 5550
Wire Wire Line
	6300 3850 6450 3850
$Comp
L power:Earth #PWR024
U 1 1 5F4DDA7E
P 6450 3900
F 0 "#PWR024" H 6450 3650 50  0001 C CNN
F 1 "Earth" H 6450 3750 50  0001 C CNN
F 2 "" H 6450 3900 50  0001 C CNN
F 3 "~" H 6450 3900 50  0001 C CNN
	1    6450 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3900 6450 3850
Connection ~ 6450 3850
Wire Wire Line
	6450 3850 6600 3850
Wire Notes Line
	500  500  3100 500 
Wire Notes Line
	3100 500  3100 4050
Wire Notes Line
	3100 4050 500  4050
Wire Notes Line
	500  4050 500  500 
Wire Notes Line
	500  4100 3100 4100
Wire Notes Line
	3100 4100 3100 4900
Wire Notes Line
	3100 4900 500  4900
Wire Notes Line
	500  4900 500  4100
Wire Notes Line
	500  4950 4000 4950
Wire Notes Line
	4000 4950 4000 6000
Wire Notes Line
	4000 6000 500  6000
Wire Notes Line
	500  6000 500  4950
Wire Notes Line
	500  6050 6750 6050
Wire Notes Line
	6750 6050 6750 7750
Wire Notes Line
	6750 7750 500  7750
Wire Notes Line
	500  7750 500  6050
Wire Notes Line
	3450 500  7350 500 
Wire Notes Line
	7350 500  7350 3050
Wire Notes Line
	7350 3050 3450 3050
Wire Notes Line
	3450 3050 3450 500 
Wire Notes Line
	3450 3150 4650 3150
Wire Notes Line
	4650 3150 4650 4400
Wire Notes Line
	4650 4400 3450 4400
Wire Notes Line
	3450 4400 3450 3150
Wire Notes Line
	5600 3150 7350 3150
Wire Notes Line
	7350 3150 7350 4050
Wire Notes Line
	7350 4050 5600 4050
Wire Notes Line
	5600 4050 5600 3150
Wire Notes Line
	5600 4150 7350 4150
Wire Notes Line
	7350 4150 7350 5950
Wire Notes Line
	7350 5950 5600 5950
Wire Notes Line
	5600 5950 5600 4150
$Comp
L Device:R_US R25
U 1 1 5F65328F
P 10000 1550
F 0 "R25" V 10100 1550 50  0000 L CNN
F 1 "120kΩ" V 9900 1400 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" V 10040 1540 50  0001 C CNN
F 3 "" H 10000 1550 50  0001 C CNN
F 4 "ERJ-1GNF1203C" H 10000 1550 50  0001 C CNN "Manufacturer Part #"
F 5 "-" H 10000 1550 50  0001 C CNN "Voltage Rating"
	1    10000 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R24
U 1 1 5F654D6C
P 9600 1550
F 0 "R24" V 9500 1450 50  0000 L CNN
F 1 "52.3kΩ" V 9700 1450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" V 9640 1540 50  0001 C CNN
F 3 "~" H 9600 1550 50  0001 C CNN
F 4 "ERJ-1GNF5232C" H 9600 1550 50  0001 C CNN "Manufacturer Part #"
	1    9600 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 5500 1950 5500
Connection ~ 1950 5500
Wire Wire Line
	2700 5500 2950 5500
Wire Wire Line
	3700 5500 3950 5500
Connection ~ 2950 5500
Connection ~ 3950 5500
Wire Wire Line
	6750 1900 6950 1900
Wire Wire Line
	6950 2000 6950 1900
Wire Wire Line
	6400 2000 6950 2000
Connection ~ 6950 1900
Wire Wire Line
	6950 1900 7200 1900
Wire Wire Line
	6950 2200 6950 2100
Wire Wire Line
	6750 2100 6950 2100
Wire Wire Line
	6400 2200 6950 2200
Wire Wire Line
	2950 5500 2950 5800
Wire Wire Line
	1950 5800 2950 5800
Wire Wire Line
	1950 5500 1950 5800
Connection ~ 2950 5800
Wire Wire Line
	2950 5800 2950 5850
Wire Wire Line
	2950 5800 3950 5800
Wire Wire Line
	3950 5500 3950 5800
Wire Wire Line
	1400 5700 1350 5700
Wire Wire Line
	1350 5500 1350 5700
Wire Wire Line
	2400 5700 2350 5700
Wire Wire Line
	2350 5500 2350 5700
Wire Wire Line
	3400 5700 3350 5700
Wire Wire Line
	3350 5500 3350 5700
Wire Wire Line
	3400 5200 3400 5300
Wire Wire Line
	3400 5300 3950 5300
Wire Wire Line
	3950 5300 3950 5500
Wire Wire Line
	2400 5200 2400 5300
Wire Wire Line
	2400 5300 2950 5300
Wire Wire Line
	2950 5300 2950 5500
Wire Wire Line
	1400 5200 1400 5300
Wire Wire Line
	1400 5300 1950 5300
Wire Wire Line
	1950 5300 1950 5500
$Comp
L Device:C C6
U 1 1 5F905B02
P 1550 5500
F 0 "C6" V 1500 5650 50  0000 C CNN
F 1 "6.8nF" V 1600 5650 50  0000 C CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 1588 5350 50  0001 C CNN
F 3 "" H 1550 5500 50  0001 C CNN
F 4 "25V" H 1550 5500 50  0001 C CNN "Voltage Rating"
F 5 "GRM033R71E682KE14D" H 1550 5500 50  0001 C CNN "Manufacturer Part #"
	1    1550 5500
	0    1    1    0   
$EndComp
$Comp
L Device:C C29
U 1 1 5F74E343
P 8500 1450
F 0 "C29" H 8500 1550 50  0000 L CNN
F 1 "1µF" H 8500 1350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8538 1300 50  0001 C CNN
F 3 "~" H 8500 1450 50  0001 C CNN
F 4 "UMK105CBJ105MV-F" H 8500 1450 50  0001 C CNN "Manufacturer Part #"
	1    8500 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C28
U 1 1 5F74F5F6
P 8300 1450
F 0 "C28" H 8300 1550 50  0000 L CNN
F 1 "1µF" H 8300 1350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8338 1300 50  0001 C CNN
F 3 "~" H 8300 1450 50  0001 C CNN
F 4 "UMK105CBJ105MV-F" H 8300 1450 50  0001 C CNN "Manufacturer Part #"
	1    8300 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C34
U 1 1 5F75085B
P 10400 1300
F 0 "C34" V 10350 1350 50  0000 L CNN
F 1 "10µF" V 10450 1350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10438 1150 50  0001 C CNN
F 3 "~" H 10400 1300 50  0001 C CNN
F 4 "C1608X5R1E106M080AC" H 10400 1300 50  0001 C CNN "Manufacturer Part #"
	1    10400 1300
	0    1    1    0   
$EndComp
$Comp
L Device:C C35
U 1 1 5F751A50
P 10400 1550
F 0 "C35" V 10350 1600 50  0000 L CNN
F 1 "10µF" V 10450 1600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10438 1400 50  0001 C CNN
F 3 "~" H 10400 1550 50  0001 C CNN
F 4 "C1608X5R1E106M080AC" H 10400 1550 50  0001 C CNN "Manufacturer Part #"
	1    10400 1550
	0    1    1    0   
$EndComp
$Comp
L Device:C C30
U 1 1 5F752C67
P 9500 1200
F 0 "C30" V 9450 1250 50  0000 L CNN
F 1 "100nF" V 9550 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 9538 1050 50  0001 C CNN
F 3 "~" H 9500 1200 50  0001 C CNN
F 4 "C0603X5R1E104K030BB" H 9500 1200 50  0001 C CNN "Manufacturer Part #"
	1    9500 1200
	0    1    1    0   
$EndComp
$Comp
L Device:L L1
U 1 1 5F756084
P 10000 1300
F 0 "L1" V 10100 1250 50  0000 L CNN
F 1 "10µH" V 9950 1200 50  0000 L CNN
F 2 "Inductor_SMD:L_1210_3225Metric" H 10000 1300 50  0001 C CNN
F 3 "~" H 10000 1300 50  0001 C CNN
F 4 "CBC3225T100KR" H 10000 1300 50  0001 C CNN "Manufacturer Part #"
	1    10000 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8700 1200 8650 1200
Wire Wire Line
	8300 1200 8300 1300
Wire Wire Line
	8500 1300 8500 1200
Connection ~ 8500 1200
Wire Wire Line
	8500 1200 8300 1200
Wire Wire Line
	8700 1400 8650 1400
Wire Wire Line
	8650 1400 8650 1200
Connection ~ 8650 1200
Wire Wire Line
	8650 1200 8500 1200
Wire Wire Line
	1900 700  1900 750 
Wire Wire Line
	1700 800  1700 750 
Wire Wire Line
	2100 750  2100 800 
Wire Wire Line
	2100 750  2000 750 
Connection ~ 1900 750 
Wire Wire Line
	1900 750  1900 800 
Wire Wire Line
	1900 750  1800 750 
Wire Wire Line
	1800 800  1800 750 
Connection ~ 1800 750 
Wire Wire Line
	1800 750  1700 750 
Wire Wire Line
	2000 800  2000 750 
Connection ~ 2000 750 
Wire Wire Line
	2000 750  1900 750 
$Comp
L TPS560430X:TPS560430X U6
U 1 1 5F504EA7
P 9000 1300
F 0 "U6" H 9000 1600 50  0000 C CNN
F 1 "TPS560430XDBVR" H 9000 1500 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 9800 1400 50  0001 C CNN
F 3 "" H 9800 1400 50  0001 C CNN
F 4 "TPS560430XDBVR" H 9000 1300 50  0001 C CNN "Manufacturer Part #"
	1    9000 1300
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR031
U 1 1 5F505FBE
P 8500 1150
F 0 "#PWR031" H 8500 1000 50  0001 C CNN
F 1 "VBUS" H 8500 1300 50  0000 C CNN
F 2 "" H 8500 1150 50  0001 C CNN
F 3 "" H 8500 1150 50  0001 C CNN
	1    8500 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 1200 9300 1200
Wire Wire Line
	9300 1300 9800 1300
Wire Wire Line
	9800 1300 9800 1200
Wire Wire Line
	9800 1200 9650 1200
Wire Wire Line
	9850 1300 9800 1300
Connection ~ 9800 1300
Wire Wire Line
	10150 1300 10200 1300
Wire Wire Line
	10200 1300 10200 1550
Wire Wire Line
	9450 1550 9400 1550
Wire Wire Line
	9750 1550 9800 1550
Wire Wire Line
	9800 1550 9800 1400
Wire Wire Line
	9800 1400 9300 1400
Connection ~ 9800 1550
Wire Wire Line
	9800 1550 9850 1550
Wire Wire Line
	10200 1550 10150 1550
Wire Wire Line
	10250 1550 10200 1550
Connection ~ 10200 1550
Wire Wire Line
	10250 1300 10200 1300
Connection ~ 10200 1300
Wire Wire Line
	9400 1700 10650 1700
Wire Wire Line
	10650 1700 10650 1550
Wire Wire Line
	10650 1550 10550 1550
Wire Wire Line
	10650 1550 10650 1300
Wire Wire Line
	10650 1300 10550 1300
Connection ~ 10650 1550
Wire Wire Line
	9400 1550 9400 1700
Wire Wire Line
	8300 1700 8500 1700
Wire Wire Line
	8300 1600 8300 1700
Connection ~ 9400 1700
Wire Wire Line
	8500 1600 8500 1700
Connection ~ 8500 1700
Wire Wire Line
	8500 1700 9000 1700
Wire Wire Line
	9000 1550 9000 1700
Connection ~ 9000 1700
Wire Wire Line
	9000 1700 9400 1700
Wire Wire Line
	8500 1150 8500 1200
$Comp
L power:Earth #PWR032
U 1 1 5F815B06
P 8500 1750
F 0 "#PWR032" H 8500 1500 50  0001 C CNN
F 1 "Earth" H 8500 1600 50  0001 C CNN
F 2 "" H 8500 1750 50  0001 C CNN
F 3 "~" H 8500 1750 50  0001 C CNN
	1    8500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1750 8500 1700
Text Notes 8200 900  0    50   ~ 0
3.3V BUCK CONVERTER
Wire Notes Line
	8150 800  10700 800 
Wire Notes Line
	10700 800  10700 1900
Wire Notes Line
	10700 1900 8150 1900
Wire Notes Line
	8150 1900 8150 800 
Wire Notes Line
	8700 2100 10200 2100
Wire Notes Line
	10200 2100 10200 6150
Wire Notes Line
	10200 6150 8700 6150
Wire Notes Line
	8700 6150 8700 2100
$EndSCHEMATC
