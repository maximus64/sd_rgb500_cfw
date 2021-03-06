EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
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
L Amplifier_Operational:TL074 U1
U 2 1 5ED5EF73
P 3700 4725
F 0 "U1" H 3700 5092 50  0000 C CNN
F 1 "TL074" H 3700 5001 50  0000 C CNN
F 2 "" H 3650 4825 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 3750 4925 50  0001 C CNN
	2    3700 4725
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TL074 U1
U 3 1 5ED60072
P 12250 5700
F 0 "U1" H 12250 5333 50  0000 C CNN
F 1 "TL074" H 12250 5424 50  0000 C CNN
F 2 "" H 12200 5800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 12300 5900 50  0001 C CNN
	3    12250 5700
	-1   0    0    1   
$EndComp
$Comp
L Amplifier_Operational:TL074 U1
U 4 1 5ED60CE2
P 3975 6900
F 0 "U1" H 3975 7267 50  0000 C CNN
F 1 "TL074" H 3975 7176 50  0000 C CNN
F 2 "" H 3925 7000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 4025 7100 50  0001 C CNN
	4    3975 6900
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TL074 U1
U 5 1 5ED62062
P 10200 7425
F 0 "U1" H 10158 7471 50  0000 L CNN
F 1 "TL074" H 10158 7380 50  0000 L CNN
F 2 "" H 10150 7525 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 10250 7625 50  0001 C CNN
	5    10200 7425
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R58
U 1 1 5ED85AAF
P 3675 5350
F 0 "R58" V 3575 5150 50  0000 C CNN
F 1 "100K" V 3570 5350 50  0000 C CNN
F 2 "" H 3675 5350 50  0001 C CNN
F 3 "~" H 3675 5350 50  0001 C CNN
	1    3675 5350
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5ED85AB5
P 3675 5100
F 0 "C8" V 3550 4875 50  0000 C CNN
F 1 "22pF" V 3537 5100 50  0000 C CNN
F 2 "" H 3675 5100 50  0001 C CNN
F 3 "~" H 3675 5100 50  0001 C CNN
	1    3675 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 4725 4100 4725
Wire Wire Line
	3775 5100 4100 5100
Wire Wire Line
	4100 5100 4100 4725
Connection ~ 4100 4725
Wire Wire Line
	4100 4725 4700 4725
Wire Wire Line
	3775 5350 4100 5350
Wire Wire Line
	4100 5350 4100 5100
Connection ~ 4100 5100
Wire Wire Line
	3575 5350 3225 5350
Wire Wire Line
	3225 5350 3225 5100
Wire Wire Line
	3225 4825 3400 4825
Wire Wire Line
	3575 5100 3225 5100
Connection ~ 3225 5100
Wire Wire Line
	3225 5100 3225 4825
$Comp
L power:GND #PWR?
U 1 1 5ED85AC9
P 3150 4625
F 0 "#PWR?" H 3150 4375 50  0001 C CNN
F 1 "GND" V 3155 4497 50  0000 R CNN
F 2 "" H 3150 4625 50  0001 C CNN
F 3 "" H 3150 4625 50  0001 C CNN
	1    3150 4625
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 4625 3150 4625
$Comp
L Device:R_Small R25
U 1 1 5ED85AD0
P 2975 4825
F 0 "R25" V 3050 4625 50  0000 C CNN
F 1 "1K" V 3050 4825 50  0000 C CNN
F 2 "" H 2975 4825 50  0001 C CNN
F 3 "~" H 2975 4825 50  0001 C CNN
	1    2975 4825
	0    1    1    0   
$EndComp
Wire Wire Line
	3075 4825 3225 4825
Connection ~ 3225 4825
Wire Wire Line
	2875 4825 2275 4825
Text Label 2275 4825 0    50   ~ 0
PD_IN2
Wire Wire Line
	4700 4250 4700 4725
Wire Wire Line
	5100 4250 4700 4250
Wire Wire Line
	4700 4050 5100 4050
Wire Wire Line
	4700 3550 4700 4050
Wire Wire Line
	5300 4050 5425 4050
$Comp
L Device:R_Small R65
U 1 1 5ED992CA
P 5200 4250
F 0 "R65" V 5100 4050 50  0000 C CNN
F 1 "2.2K" V 5095 4250 50  0000 C CNN
F 2 "" H 5200 4250 50  0001 C CNN
F 3 "~" H 5200 4250 50  0001 C CNN
	1    5200 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R64
U 1 1 5ED98AD0
P 5200 4050
F 0 "R64" V 5100 3850 50  0000 C CNN
F 1 "2.2K" V 5095 4050 50  0000 C CNN
F 2 "" H 5200 4050 50  0001 C CNN
F 3 "~" H 5200 4050 50  0001 C CNN
	1    5200 4050
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:TL074 U2
U 1 1 5ED92401
P 5850 4150
F 0 "U2" H 5850 4517 50  0000 C CNN
F 1 "324" H 5850 4426 50  0000 C CNN
F 2 "" H 5800 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 5900 4350 50  0001 C CNN
	1    5850 4150
	1    0    0    -1  
$EndComp
Text Label 2275 3650 0    50   ~ 0
PD_IN1
Wire Wire Line
	2875 3650 2275 3650
Connection ~ 3225 3650
Wire Wire Line
	3075 3650 3225 3650
$Comp
L Device:R_Small R24
U 1 1 5ED76C36
P 2975 3650
F 0 "R24" V 3050 3450 50  0000 C CNN
F 1 "1K" V 3050 3650 50  0000 C CNN
F 2 "" H 2975 3650 50  0001 C CNN
F 3 "~" H 2975 3650 50  0001 C CNN
	1    2975 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 3450 3150 3450
$Comp
L power:GND #PWR?
U 1 1 5ED7625F
P 3150 3450
F 0 "#PWR?" H 3150 3200 50  0001 C CNN
F 1 "GND" V 3155 3322 50  0000 R CNN
F 2 "" H 3150 3450 50  0001 C CNN
F 3 "" H 3150 3450 50  0001 C CNN
	1    3150 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	3225 3925 3225 3650
Connection ~ 3225 3925
Wire Wire Line
	3575 3925 3225 3925
Wire Wire Line
	3225 3650 3400 3650
Wire Wire Line
	3225 4175 3225 3925
Wire Wire Line
	3575 4175 3225 4175
Connection ~ 4100 3925
Wire Wire Line
	4100 4175 4100 3925
Wire Wire Line
	3775 4175 4100 4175
Wire Wire Line
	4100 3550 4700 3550
Connection ~ 4100 3550
Wire Wire Line
	4100 3925 4100 3550
Wire Wire Line
	3775 3925 4100 3925
Wire Wire Line
	4000 3550 4100 3550
$Comp
L Device:C_Small C7
U 1 1 5ED6976C
P 3675 3925
F 0 "C7" V 3550 3700 50  0000 C CNN
F 1 "22pF" V 3537 3925 50  0000 C CNN
F 2 "" H 3675 3925 50  0001 C CNN
F 3 "~" H 3675 3925 50  0001 C CNN
	1    3675 3925
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R57
U 1 1 5ED683F9
P 3675 4175
F 0 "R57" V 3575 3975 50  0000 C CNN
F 1 "100K" V 3570 4175 50  0000 C CNN
F 2 "" H 3675 4175 50  0001 C CNN
F 3 "~" H 3675 4175 50  0001 C CNN
	1    3675 4175
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:TL074 U1
U 1 1 5ED5DDB4
P 3700 3550
F 0 "U1" H 3700 3917 50  0000 C CNN
F 1 "TL074" H 3700 3826 50  0000 C CNN
F 2 "" H 3650 3650 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 3750 3750 50  0001 C CNN
	1    3700 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R77
U 1 1 5EDEF565
P 5750 4600
F 0 "R77" V 5650 4400 50  0000 C CNN
F 1 "6.8K" V 5645 4600 50  0000 C CNN
F 2 "" H 5750 4600 50  0001 C CNN
F 3 "~" H 5750 4600 50  0001 C CNN
	1    5750 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 4600 5425 4600
Wire Wire Line
	6400 4150 6150 4150
$Comp
L Device:R_Small R1
U 1 1 5EDF4308
P 6675 4600
F 0 "R1" V 6575 4600 50  0000 C CNN
F 1 "1K" V 6750 4600 50  0000 C CNN
F 2 "" H 6675 4600 50  0001 C CNN
F 3 "~" H 6675 4600 50  0001 C CNN
	1    6675 4600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6775 4600 7350 4600
Text Label 7350 4600 2    50   ~ 0
POSITION_OUT
Wire Wire Line
	5300 4250 5425 4250
Wire Wire Line
	5850 4600 6400 4600
Wire Wire Line
	6400 4150 6400 4600
Connection ~ 6400 4600
Wire Wire Line
	6400 4600 6575 4600
Wire Wire Line
	5425 4600 5425 4250
Connection ~ 5425 4250
Wire Wire Line
	5425 4250 5550 4250
$Comp
L Device:R_Small R76
U 1 1 5EE114C5
P 5750 3575
F 0 "R76" V 5650 3375 50  0000 C CNN
F 1 "6.8K" V 5645 3575 50  0000 C CNN
F 2 "" H 5750 3575 50  0001 C CNN
F 3 "~" H 5750 3575 50  0001 C CNN
	1    5750 3575
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3575 5425 3575
Wire Wire Line
	5425 3575 5425 4050
Connection ~ 5425 4050
Wire Wire Line
	5425 4050 5550 4050
$Comp
L power:GND #PWR?
U 1 1 5EE12973
P 6075 3575
F 0 "#PWR?" H 6075 3325 50  0001 C CNN
F 1 "GND" H 6080 3402 50  0000 C CNN
F 2 "" H 6075 3575 50  0001 C CNN
F 3 "" H 6075 3575 50  0001 C CNN
	1    6075 3575
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3575 6075 3575
$Comp
L Device:C_Small C19
U 1 1 5EE23B80
P 7025 4150
F 0 "C19" V 6900 3925 50  0000 C CNN
F 1 "0.022uF" V 6887 4150 50  0000 C CNN
F 2 "" H 7025 4150 50  0001 C CNN
F 3 "~" H 7025 4150 50  0001 C CNN
	1    7025 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	6925 4150 6625 4150
Connection ~ 6400 4150
$Comp
L Device:C_Small C10
U 1 1 5EE2B77F
P 7025 3900
F 0 "C10" V 6900 3675 50  0000 C CNN
F 1 "DNP" V 6887 3900 50  0000 C CNN
F 2 "" H 7025 3900 50  0001 C CNN
F 3 "~" H 7025 3900 50  0001 C CNN
	1    7025 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	6925 3900 6625 3900
Wire Wire Line
	6625 3900 6625 4150
Connection ~ 6625 4150
Wire Wire Line
	6625 4150 6400 4150
$Comp
L Device:R_Small R66
U 1 1 5EE2EF3B
P 7650 4150
F 0 "R66" V 7550 4150 50  0000 C CNN
F 1 "2.2K" V 7725 4150 50  0000 C CNN
F 2 "" H 7650 4150 50  0001 C CNN
F 3 "~" H 7650 4150 50  0001 C CNN
	1    7650 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7125 4150 7350 4150
Wire Wire Line
	7750 4150 8000 4150
Wire Wire Line
	7125 3900 7350 3900
Wire Wire Line
	7350 3900 7350 4150
Connection ~ 7350 4150
Wire Wire Line
	7350 4150 7550 4150
$Comp
L Device:C_Small C14
U 1 1 5EE363DA
P 8500 4450
F 0 "C14" V 8375 4225 50  0000 C CNN
F 1 "470pF" V 8362 4450 50  0000 C CNN
F 2 "" H 8500 4450 50  0001 C CNN
F 3 "~" H 8500 4450 50  0001 C CNN
	1    8500 4450
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R48
U 1 1 5EE36BF2
P 8500 4675
F 0 "R48" V 8400 4475 50  0000 C CNN
F 1 "10K" V 8395 4675 50  0000 C CNN
F 2 "" H 8500 4675 50  0001 C CNN
F 3 "~" H 8500 4675 50  0001 C CNN
	1    8500 4675
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 4150 8000 4450
Wire Wire Line
	8000 4450 8400 4450
Connection ~ 8000 4150
Wire Wire Line
	8000 4150 8200 4150
Wire Wire Line
	8000 4450 8000 4675
Wire Wire Line
	8000 4675 8400 4675
Connection ~ 8000 4450
Wire Wire Line
	8800 4050 9000 4050
Wire Wire Line
	9000 4050 9000 4450
Wire Wire Line
	9000 4450 8600 4450
Wire Wire Line
	8600 4675 9000 4675
Wire Wire Line
	9000 4675 9000 4450
Connection ~ 9000 4450
$Comp
L power:GND #PWR?
U 1 1 5EE3DAC3
P 8000 3950
F 0 "#PWR?" H 8000 3700 50  0001 C CNN
F 1 "GND" V 8005 3822 50  0000 R CNN
F 2 "" H 8000 3950 50  0001 C CNN
F 3 "" H 8000 3950 50  0001 C CNN
	1    8000 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	8200 3950 8000 3950
$Comp
L Amplifier_Operational:TL074 U2
U 4 1 5EE482B0
P 8500 4050
F 0 "U2" H 8500 4417 50  0000 C CNN
F 1 "324" H 8500 4326 50  0000 C CNN
F 2 "" H 8450 4150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 8550 4250 50  0001 C CNN
	4    8500 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV?
U 1 1 5EE5247B
P 9450 3800
F 0 "RV?" H 9381 3846 50  0000 R CNN
F 1 "10K" H 9381 3755 50  0000 R CNN
F 2 "" H 9450 3800 50  0001 C CNN
F 3 "~" H 9450 3800 50  0001 C CNN
	1    9450 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4050 9450 4050
Wire Wire Line
	9450 4050 9450 3950
Connection ~ 9000 4050
Text Notes 9300 4275 0    50   ~ 0
LOW FREQUENCY\nDAMPING
$Comp
L power:GND #PWR?
U 1 1 5EE57FB2
P 9450 3575
F 0 "#PWR?" H 9450 3325 50  0001 C CNN
F 1 "GND" H 9455 3402 50  0000 C CNN
F 2 "" H 9450 3575 50  0001 C CNN
F 3 "" H 9450 3575 50  0001 C CNN
	1    9450 3575
	-1   0    0    1   
$EndComp
Wire Wire Line
	9450 3650 9450 3575
$Comp
L Device:R_Small R37
U 1 1 5EE5AFBC
P 9925 3800
F 0 "R37" V 9825 3600 50  0000 C CNN
F 1 "4.7K" V 9820 3800 50  0000 C CNN
F 2 "" H 9925 3800 50  0001 C CNN
F 3 "~" H 9925 3800 50  0001 C CNN
	1    9925 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	9600 3800 9825 3800
$Comp
L Amplifier_Operational:TL074 U2
U 3 1 5EE6A64D
P 10800 3700
F 0 "U2" H 10800 4067 50  0000 C CNN
F 1 "324" H 10800 3976 50  0000 C CNN
F 2 "" H 10750 3800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 10850 3900 50  0001 C CNN
	3    10800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10025 3800 10200 3800
$Comp
L Device:R_Small R55
U 1 1 5EE73D9E
P 10800 4075
F 0 "R55" V 10700 3875 50  0000 C CNN
F 1 "100K" V 10695 4075 50  0000 C CNN
F 2 "" H 10800 4075 50  0001 C CNN
F 3 "~" H 10800 4075 50  0001 C CNN
	1    10800 4075
	0    1    1    0   
$EndComp
Wire Wire Line
	10350 3800 10350 4075
Wire Wire Line
	10350 4075 10700 4075
Connection ~ 10350 3800
Wire Wire Line
	10350 3800 10500 3800
Wire Wire Line
	10900 4075 11350 4075
Wire Wire Line
	11350 4075 11350 3700
Wire Wire Line
	11350 3700 11100 3700
$Comp
L power:GND #PWR?
U 1 1 5EE78764
P 10350 3600
F 0 "#PWR?" H 10350 3350 50  0001 C CNN
F 1 "GND" V 10355 3472 50  0000 R CNN
F 2 "" H 10350 3600 50  0001 C CNN
F 3 "" H 10350 3600 50  0001 C CNN
	1    10350 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	10500 3600 10350 3600
$Comp
L Device:R_Small R46
U 1 1 5EE7AC39
P 12300 3700
F 0 "R46" V 12200 3500 50  0000 C CNN
F 1 "15K" V 12195 3700 50  0000 C CNN
F 2 "" H 12300 3700 50  0001 C CNN
F 3 "~" H 12300 3700 50  0001 C CNN
	1    12300 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	11350 3700 12200 3700
Connection ~ 11350 3700
$Comp
L Amplifier_Operational:TL074 U2
U 2 1 5EE86A26
P 8475 5475
F 0 "U2" H 8475 5842 50  0000 C CNN
F 1 "324" H 8475 5751 50  0000 C CNN
F 2 "" H 8425 5575 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 8525 5675 50  0001 C CNN
	2    8475 5475
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R35
U 1 1 5EE8D53C
P 6700 5575
F 0 "R35" V 6600 5575 50  0000 C CNN
F 1 "100K" V 6775 5575 50  0000 C CNN
F 2 "" H 6700 5575 50  0001 C CNN
F 3 "~" H 6700 5575 50  0001 C CNN
	1    6700 5575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8175 5575 7875 5575
Wire Wire Line
	6400 4600 6400 5575
Wire Wire Line
	6400 5575 6600 5575
$Comp
L power:GND #PWR?
U 1 1 5EEACC2B
P 8025 5375
F 0 "#PWR?" H 8025 5125 50  0001 C CNN
F 1 "GND" V 8030 5247 50  0000 R CNN
F 2 "" H 8025 5375 50  0001 C CNN
F 3 "" H 8025 5375 50  0001 C CNN
	1    8025 5375
	0    1    1    0   
$EndComp
Wire Wire Line
	8025 5375 8175 5375
Text Notes 4325 3250 0    50   ~ 0
POSITION DEMODULATOR
$Comp
L Device:R_POT RV?
U 1 1 5EEC5858
P 7175 6250
F 0 "RV?" H 7106 6296 50  0000 R CNN
F 1 "10K" H 7106 6205 50  0000 R CNN
F 2 "" H 7175 6250 50  0001 C CNN
F 3 "~" H 7175 6250 50  0001 C CNN
	1    7175 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEC65BA
P 7175 6650
F 0 "#PWR?" H 7175 6400 50  0001 C CNN
F 1 "GND" H 7180 6477 50  0000 C CNN
F 2 "" H 7175 6650 50  0001 C CNN
F 3 "" H 7175 6650 50  0001 C CNN
	1    7175 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7175 6400 7175 6650
Wire Wire Line
	7175 6100 7175 5900
Wire Wire Line
	7175 5900 6375 5900
Text Label 6375 5900 0    50   ~ 0
POS_IN
$Comp
L Device:R_Small R34
U 1 1 5EEF56AA
P 7650 6250
F 0 "R34" V 7550 6250 50  0000 C CNN
F 1 "100K" V 7725 6250 50  0000 C CNN
F 2 "" H 7650 6250 50  0001 C CNN
F 3 "~" H 7650 6250 50  0001 C CNN
	1    7650 6250
	0    -1   -1   0   
$EndComp
Text Notes 6425 6300 0    50   ~ 0
INPUT SCALE
Wire Wire Line
	7325 6250 7550 6250
Wire Wire Line
	7750 6250 7875 6250
Wire Wire Line
	7875 6250 7875 5575
Connection ~ 7875 5575
Wire Wire Line
	7875 5575 6800 5575
$Comp
L Device:R_Small R45
U 1 1 5EEFF9DF
P 8425 6250
F 0 "R45" V 8325 6250 50  0000 C CNN
F 1 "100K" V 8500 6250 50  0000 C CNN
F 2 "" H 8425 6250 50  0001 C CNN
F 3 "~" H 8425 6250 50  0001 C CNN
	1    8425 6250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7875 6250 8325 6250
Connection ~ 7875 6250
Wire Wire Line
	8525 6250 9025 6250
Wire Wire Line
	9025 6250 9025 5475
Wire Wire Line
	9025 5475 8775 5475
$Comp
L Device:R_POT RV?
U 1 1 5EF046BE
P 9025 5225
F 0 "RV?" H 8956 5271 50  0000 R CNN
F 1 "10K" H 8956 5180 50  0000 R CNN
F 2 "" H 9025 5225 50  0001 C CNN
F 3 "~" H 9025 5225 50  0001 C CNN
	1    9025 5225
	1    0    0    -1  
$EndComp
Wire Wire Line
	9025 5375 9025 5475
Connection ~ 9025 5475
Text Notes 9100 5375 0    50   ~ 0
SERVO GAIN
Text Notes 8025 3550 0    50   ~ 0
POSITION DIFFRENTIATOR
Text Notes 8150 5950 0    50   ~ 0
ERROR AMPLIFIER
$Comp
L power:GND #PWR?
U 1 1 5EF2FE4F
P 9025 4975
F 0 "#PWR?" H 9025 4725 50  0001 C CNN
F 1 "GND" H 9030 4802 50  0000 C CNN
F 2 "" H 9025 4975 50  0001 C CNN
F 3 "" H 9025 4975 50  0001 C CNN
	1    9025 4975
	-1   0    0    1   
$EndComp
Wire Wire Line
	9025 4975 9025 5075
$Comp
L Device:R_Small R36
U 1 1 5EF5A736
P 9875 5225
F 0 "R36" V 9775 5025 50  0000 C CNN
F 1 "4.7K" V 9770 5225 50  0000 C CNN
F 2 "" H 9875 5225 50  0001 C CNN
F 3 "~" H 9875 5225 50  0001 C CNN
	1    9875 5225
	0    1    1    0   
$EndComp
Wire Wire Line
	9175 5225 9775 5225
Wire Wire Line
	9975 5225 10200 5225
Wire Wire Line
	10200 5225 10200 3800
Connection ~ 10200 3800
Wire Wire Line
	10200 3800 10350 3800
$Comp
L Device:C_Small C1
U 1 1 5EF63767
P 12275 4425
F 0 "C1" V 12150 4200 50  0000 C CNN
F 1 "220pF" V 12137 4425 50  0000 C CNN
F 2 "" H 12275 4425 50  0001 C CNN
F 3 "~" H 12275 4425 50  0001 C CNN
	1    12275 4425
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R75
U 1 1 5EF63B63
P 12275 4625
F 0 "R75" V 12175 4425 50  0000 C CNN
F 1 "10K" V 12170 4625 50  0000 C CNN
F 2 "" H 12275 4625 50  0001 C CNN
F 3 "~" H 12275 4625 50  0001 C CNN
	1    12275 4625
	0    1    1    0   
$EndComp
Wire Wire Line
	12400 3700 12625 3700
Wire Wire Line
	12625 3700 12625 4425
Wire Wire Line
	12625 4625 12375 4625
Connection ~ 12625 3700
Wire Wire Line
	12375 4425 12625 4425
Connection ~ 12625 4425
Wire Wire Line
	12625 4425 12625 4625
Wire Wire Line
	12175 4425 11800 4425
Wire Wire Line
	11800 4425 11800 4625
Wire Wire Line
	11800 4625 12175 4625
Wire Wire Line
	11800 4625 11400 4625
Connection ~ 11800 4625
Wire Wire Line
	13950 5800 13475 5800
Text Label 13950 5800 2    50   ~ 0
MOTOR-
$Comp
L Device:R_Small R?
U 1 1 5EF9AE68
P 13475 6050
F 0 "R?" V 13375 6050 50  0000 C CNN
F 1 "0.1" V 13550 6050 50  0000 C CNN
F 2 "" H 13475 6050 50  0001 C CNN
F 3 "~" H 13475 6050 50  0001 C CNN
	1    13475 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	13475 5950 13475 5800
Connection ~ 13475 5800
Wire Wire Line
	13475 5800 13250 5800
$Comp
L power:GND #PWR?
U 1 1 5EF9E9CE
P 13475 6325
F 0 "#PWR?" H 13475 6075 50  0001 C CNN
F 1 "GND" H 13480 6152 50  0000 C CNN
F 2 "" H 13475 6325 50  0001 C CNN
F 3 "" H 13475 6325 50  0001 C CNN
	1    13475 6325
	1    0    0    -1  
$EndComp
Wire Wire Line
	13475 6150 13475 6325
$Comp
L Device:R_Small R88
U 1 1 5EFA3A13
P 13150 5800
F 0 "R88" V 13050 5600 50  0000 C CNN
F 1 "100" V 13045 5800 50  0000 C CNN
F 2 "" H 13150 5800 50  0001 C CNN
F 3 "~" H 13150 5800 50  0001 C CNN
	1    13150 5800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R17
U 1 1 5EFA4C75
P 12900 6025
F 0 "R17" V 12800 6025 50  0000 C CNN
F 1 "2K" V 12975 6025 50  0000 C CNN
F 2 "" H 12900 6025 50  0001 C CNN
F 3 "~" H 12900 6025 50  0001 C CNN
	1    12900 6025
	1    0    0    -1  
$EndComp
Wire Wire Line
	12900 5800 12900 5925
$Comp
L power:GND #PWR?
U 1 1 5EFA851D
P 12900 6350
F 0 "#PWR?" H 12900 6100 50  0001 C CNN
F 1 "GND" H 12905 6177 50  0000 C CNN
F 2 "" H 12900 6350 50  0001 C CNN
F 3 "" H 12900 6350 50  0001 C CNN
	1    12900 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	12900 6125 12900 6350
Wire Wire Line
	12900 5800 12550 5800
Wire Wire Line
	12900 5800 13050 5800
Connection ~ 12900 5800
$Comp
L Device:R_Small R87
U 1 1 5EFE599E
P 12800 5600
F 0 "R87" V 12700 5400 50  0000 C CNN
F 1 "100" V 12695 5600 50  0000 C CNN
F 2 "" H 12800 5600 50  0001 C CNN
F 3 "~" H 12800 5600 50  0001 C CNN
	1    12800 5600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EFE60EF
P 13050 5600
F 0 "#PWR?" H 13050 5350 50  0001 C CNN
F 1 "GND" V 13055 5472 50  0000 R CNN
F 2 "" H 13050 5600 50  0001 C CNN
F 3 "" H 13050 5600 50  0001 C CNN
	1    13050 5600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	12550 5600 12625 5600
Wire Wire Line
	12900 5600 13050 5600
$Comp
L Device:R_Small R16
U 1 1 5EFED6BD
P 12350 5250
F 0 "R16" V 12250 5050 50  0000 C CNN
F 1 "2K" V 12245 5250 50  0000 C CNN
F 2 "" H 12350 5250 50  0001 C CNN
F 3 "~" H 12350 5250 50  0001 C CNN
	1    12350 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	12625 5600 12625 5250
Wire Wire Line
	12625 5250 12450 5250
Connection ~ 12625 5600
Wire Wire Line
	12625 5600 12700 5600
Wire Wire Line
	12250 5250 11850 5250
Wire Wire Line
	11850 5250 11850 5700
Wire Wire Line
	11850 5700 11950 5700
Wire Wire Line
	11850 5700 11400 5700
Wire Wire Line
	11400 4625 11400 5700
Connection ~ 11850 5700
Text Notes 11950 6150 0    50   ~ 0
CURRENT MONITOR
$Comp
L power:GND #PWR?
U 1 1 5F014ED7
P 12900 7450
F 0 "#PWR?" H 12900 7200 50  0001 C CNN
F 1 "GND" H 12905 7277 50  0000 C CNN
F 2 "" H 12900 7450 50  0001 C CNN
F 3 "" H 12900 7450 50  0001 C CNN
	1    12900 7450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R54
U 1 1 5F015F69
P 13250 7325
F 0 "R54" V 13150 7125 50  0000 C CNN
F 1 "24K" V 13145 7325 50  0000 C CNN
F 2 "" H 13250 7325 50  0001 C CNN
F 3 "~" H 13250 7325 50  0001 C CNN
	1    13250 7325
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C13
U 1 1 5F0163F6
P 13250 7100
F 0 "C13" V 13125 6875 50  0000 C CNN
F 1 "22pF" V 13112 7100 50  0000 C CNN
F 2 "" H 13250 7100 50  0001 C CNN
F 3 "~" H 13250 7100 50  0001 C CNN
	1    13250 7100
	0    1    1    0   
$EndComp
Wire Wire Line
	13150 7100 12900 7100
Wire Wire Line
	12900 7100 12900 7325
Wire Wire Line
	13150 7325 12900 7325
Connection ~ 12900 7325
Wire Wire Line
	12900 7325 12900 7450
$Comp
L power:GNDPWR #PWR?
U 1 1 5F020734
P 13550 7475
F 0 "#PWR?" H 13550 7275 50  0001 C CNN
F 1 "GNDPWR" H 13554 7321 50  0000 C CNN
F 2 "" H 13550 7425 50  0001 C CNN
F 3 "" H 13550 7425 50  0001 C CNN
	1    13550 7475
	1    0    0    -1  
$EndComp
Wire Wire Line
	13350 7100 13550 7100
Wire Wire Line
	13550 7100 13550 7325
Wire Wire Line
	13350 7325 13550 7325
Connection ~ 13550 7325
Wire Wire Line
	13550 7325 13550 7475
$Comp
L Device:C_Small C15
U 1 1 5F036977
P 11400 6025
F 0 "C15" H 11492 6071 50  0000 L CNN
F 1 "???" H 11492 5980 50  0000 L CNN
F 2 "" H 11400 6025 50  0001 C CNN
F 3 "~" H 11400 6025 50  0001 C CNN
	1    11400 6025
	1    0    0    -1  
$EndComp
Wire Wire Line
	11400 5700 11400 5925
Connection ~ 11400 5700
$Comp
L power:GND #PWR?
U 1 1 5F03D5EE
P 11400 6350
F 0 "#PWR?" H 11400 6100 50  0001 C CNN
F 1 "GND" H 11405 6177 50  0000 C CNN
F 2 "" H 11400 6350 50  0001 C CNN
F 3 "" H 11400 6350 50  0001 C CNN
	1    11400 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	11400 6125 11400 6350
$Comp
L Diode:1N4148 D3
U 1 1 5F0BB7B8
P 4025 7300
F 0 "D3" H 4025 7083 50  0000 C CNN
F 1 "1N4148" H 4025 7174 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4025 7125 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4025 7300 50  0001 C CNN
	1    4025 7300
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5F0BCEDC
P 4025 7550
F 0 "C2" V 3900 7325 50  0000 C CNN
F 1 "1000pF" V 3887 7550 50  0000 C CNN
F 2 "" H 4025 7550 50  0001 C CNN
F 3 "~" H 4025 7550 50  0001 C CNN
	1    4025 7550
	0    1    1    0   
$EndComp
Wire Wire Line
	4275 6900 4475 6900
Wire Wire Line
	4475 6900 4475 7300
Wire Wire Line
	4475 7300 4175 7300
Wire Wire Line
	4475 7300 4475 7550
Wire Wire Line
	4475 7550 4125 7550
Connection ~ 4475 7300
Wire Wire Line
	3875 7300 3450 7300
Wire Wire Line
	3450 7300 3450 7000
Wire Wire Line
	3450 7000 3675 7000
Wire Wire Line
	3925 7550 3450 7550
Wire Wire Line
	3450 7550 3450 7300
Connection ~ 3450 7300
$Comp
L Device:Q_NPN_BCE Q1
U 1 1 5F0E3C1D
P 4875 6900
F 0 "Q1" H 5066 6946 50  0000 L CNN
F 1 "Q_NPN_BCE" H 5066 6855 50  0000 L CNN
F 2 "" H 5075 7000 50  0001 C CNN
F 3 "~" H 4875 6900 50  0001 C CNN
	1    4875 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4475 6900 4675 6900
Connection ~ 4475 6900
$Comp
L Device:R_Small R13
U 1 1 5F0EAE0B
P 4775 7300
F 0 "R13" V 4675 7300 50  0000 C CNN
F 1 "150" V 4850 7300 50  0000 C CNN
F 2 "" H 4775 7300 50  0001 C CNN
F 3 "~" H 4775 7300 50  0001 C CNN
	1    4775 7300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4675 7300 4475 7300
Wire Wire Line
	4875 7300 4975 7300
Wire Wire Line
	4975 7300 4975 7100
$Comp
L Device:R_Small R15
U 1 1 5F0F729A
P 5275 6550
F 0 "R15" V 5175 6550 50  0000 C CNN
F 1 "300" V 5350 6550 50  0000 C CNN
F 2 "" H 5275 6550 50  0001 C CNN
F 3 "~" H 5275 6550 50  0001 C CNN
F 4 "0805 0.1W" V 5100 6550 50  0000 C CNN "Note"
	1    5275 6550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5375 6550 5850 6550
Text Label 5850 6550 2    50   ~ 0
AGC_OUT
Wire Wire Line
	5175 6550 4975 6550
Wire Wire Line
	4975 6550 4975 6700
$Comp
L power:GND #PWR?
U 1 1 5F10C8F5
P 3475 6800
F 0 "#PWR?" H 3475 6550 50  0001 C CNN
F 1 "GND" V 3480 6672 50  0000 R CNN
F 2 "" H 3475 6800 50  0001 C CNN
F 3 "" H 3475 6800 50  0001 C CNN
	1    3475 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	3675 6800 3475 6800
Wire Wire Line
	4700 3550 5100 3550
Connection ~ 4700 3550
Wire Wire Line
	4700 4725 5125 4725
Connection ~ 4700 4725
Text Label 5100 3550 2    50   ~ 0
VA
Text Label 5125 4725 2    50   ~ 0
VB
$Comp
L Device:R_POT RV?
U 1 1 5F13E009
P 2250 6475
F 0 "RV?" V 2043 6475 50  0000 C CNN
F 1 "10K" V 2134 6475 50  0000 C CNN
F 2 "" H 2250 6475 50  0001 C CNN
F 3 "~" H 2250 6475 50  0001 C CNN
	1    2250 6475
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5F170233
P 2250 6800
F 0 "R9" V 2150 6800 50  0000 C CNN
F 1 "24K" V 2325 6800 50  0000 C CNN
F 2 "" H 2250 6800 50  0001 C CNN
F 3 "~" H 2250 6800 50  0001 C CNN
	1    2250 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 6625 2250 6700
Wire Wire Line
	3450 7000 2575 7000
Wire Wire Line
	2250 7000 2250 6900
Connection ~ 3450 7000
$Comp
L Device:R_Small R6
U 1 1 5F18BCF9
P 2575 6800
F 0 "R6" V 2475 6800 50  0000 C CNN
F 1 "10K" V 2650 6800 50  0000 C CNN
F 2 "" H 2575 6800 50  0001 C CNN
F 3 "~" H 2575 6800 50  0001 C CNN
	1    2575 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5F18BF49
P 1925 6800
F 0 "R5" V 1825 6800 50  0000 C CNN
F 1 "10K" V 2000 6800 50  0000 C CNN
F 2 "" H 1925 6800 50  0001 C CNN
F 3 "~" H 1925 6800 50  0001 C CNN
	1    1925 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6475 2575 6475
Wire Wire Line
	2575 6475 2575 6700
Wire Wire Line
	2575 6900 2575 7000
Connection ~ 2575 7000
Wire Wire Line
	2575 7000 2250 7000
Wire Wire Line
	2100 6475 1925 6475
Wire Wire Line
	1925 6475 1925 6700
Wire Wire Line
	1925 6475 1925 6075
Connection ~ 1925 6475
Wire Wire Line
	2575 6475 2575 6075
Connection ~ 2575 6475
Wire Wire Line
	2250 7000 1925 7000
Wire Wire Line
	1925 7000 1925 6900
Connection ~ 2250 7000
Text Label 1925 6075 3    50   ~ 0
VA
Text Label 2575 6075 3    50   ~ 0
VB
$Comp
L Device:R_Small R10
U 1 1 5F1DF931
P 2250 7225
F 0 "R10" V 2150 7225 50  0000 C CNN
F 1 "15K" V 2325 7225 50  0000 C CNN
F 2 "" H 2250 7225 50  0001 C CNN
F 3 "~" H 2250 7225 50  0001 C CNN
	1    2250 7225
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 7125 2250 7000
Wire Wire Line
	2250 7325 2250 7375
$Comp
L power:-8V #PWR?
U 1 1 5F1F08AC
P 2250 7375
F 0 "#PWR?" H 2250 7475 50  0001 C CNN
F 1 "-8V" H 2265 7548 50  0000 C CNN
F 2 "" H 2250 7375 50  0001 C CNN
F 3 "" H 2250 7375 50  0001 C CNN
	1    2250 7375
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Linear:L79L08_SO8 U8
U 1 1 5F2130A8
P 8475 7300
F 0 "U8" H 8475 7058 50  0000 C CNN
F 1 "L79L08_SO8" H 8475 7149 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 8475 7100 50  0001 C CIN
F 3 "http://www.farnell.com/datasheets/1827870.pdf" H 8475 7350 50  0001 C CNN
	1    8475 7300
	-1   0    0    1   
$EndComp
$Comp
L power:-8V #PWR?
U 1 1 5F24D208
P 7950 7300
F 0 "#PWR?" H 7950 7400 50  0001 C CNN
F 1 "-8V" V 7965 7428 50  0000 L CNN
F 2 "" H 7950 7300 50  0001 C CNN
F 3 "" H 7950 7300 50  0001 C CNN
	1    7950 7300
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5F252C15
P 8050 7625
F 0 "C11" H 8142 7625 50  0000 L CNN
F 1 "???" H 8142 7580 50  0001 L CNN
F 2 "" H 8050 7625 50  0001 C CNN
F 3 "~" H 8050 7625 50  0001 C CNN
	1    8050 7625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F25398B
P 8475 7850
F 0 "#PWR?" H 8475 7600 50  0001 C CNN
F 1 "GND" H 8480 7677 50  0000 C CNN
F 2 "" H 8475 7850 50  0001 C CNN
F 3 "" H 8475 7850 50  0001 C CNN
	1    8475 7850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C23
U 1 1 5F2541AD
P 8875 7625
F 0 "C23" H 8967 7625 50  0000 L CNN
F 1 "???" H 8967 7580 50  0001 L CNN
F 2 "" H 8875 7625 50  0001 C CNN
F 3 "~" H 8875 7625 50  0001 C CNN
	1    8875 7625
	1    0    0    -1  
$EndComp
Wire Wire Line
	8475 7600 8475 7725
Wire Wire Line
	8050 7725 8475 7725
Connection ~ 8475 7725
Wire Wire Line
	8475 7725 8475 7850
Wire Wire Line
	8175 7300 8050 7300
Wire Wire Line
	8050 7300 8050 7525
Connection ~ 8050 7300
Wire Wire Line
	8050 7300 7950 7300
Wire Wire Line
	8775 7300 8875 7300
Wire Wire Line
	8875 7300 8875 7525
Wire Wire Line
	8875 7725 8475 7725
$Comp
L power:+12V #PWR?
U 1 1 5F288322
P 9125 7225
F 0 "#PWR?" H 9125 7075 50  0001 C CNN
F 1 "+12V" H 9140 7398 50  0000 C CNN
F 2 "" H 9125 7225 50  0001 C CNN
F 3 "" H 9125 7225 50  0001 C CNN
	1    9125 7225
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 7225 9125 7300
Wire Wire Line
	9125 7300 8875 7300
Connection ~ 8875 7300
$Comp
L Amplifier_Operational:TL074 U2
U 5 1 5F29E6F6
P 10600 7425
F 0 "U2" H 10558 7471 50  0000 L CNN
F 1 "324" H 10558 7380 50  0000 L CNN
F 2 "" H 10550 7525 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 10650 7625 50  0001 C CNN
	5    10600 7425
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5F2C036F
P 10100 7125
F 0 "#PWR?" H 10100 6975 50  0001 C CNN
F 1 "+12V" H 10115 7298 50  0000 C CNN
F 2 "" H 10100 7125 50  0001 C CNN
F 3 "" H 10100 7125 50  0001 C CNN
	1    10100 7125
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5F2C10B3
P 10500 7125
F 0 "#PWR?" H 10500 6975 50  0001 C CNN
F 1 "+12V" H 10515 7298 50  0000 C CNN
F 2 "" H 10500 7125 50  0001 C CNN
F 3 "" H 10500 7125 50  0001 C CNN
	1    10500 7125
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR?
U 1 1 5F2C199A
P 10100 7725
F 0 "#PWR?" H 10100 7825 50  0001 C CNN
F 1 "-12V" H 10115 7898 50  0000 C CNN
F 2 "" H 10100 7725 50  0001 C CNN
F 3 "" H 10100 7725 50  0001 C CNN
	1    10100 7725
	-1   0    0    1   
$EndComp
$Comp
L power:-12V #PWR?
U 1 1 5F2C1DE3
P 10500 7725
F 0 "#PWR?" H 10500 7825 50  0001 C CNN
F 1 "-12V" H 10515 7898 50  0000 C CNN
F 2 "" H 10500 7725 50  0001 C CNN
F 3 "" H 10500 7725 50  0001 C CNN
	1    10500 7725
	-1   0    0    1   
$EndComp
$Comp
L Amplifier_Audio:TDA2030 U5
U 1 1 5F2D4317
P 13600 3600
F 0 "U5" H 13944 3646 50  0000 L CNN
F 1 "TDA2030" H 13944 3555 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-5_P3.4x3.7mm_StaggerOdd_Lead3.8mm_Vertical" H 13600 3600 50  0001 C CIN
F 3 "http://www.st.com/resource/en/datasheet/cd00000128.pdf" H 13600 3600 50  0001 C CNN
	1    13600 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	12625 3700 13225 3700
$Comp
L power:GND #PWR?
U 1 1 5F2E7BD7
P 13125 3500
F 0 "#PWR?" H 13125 3250 50  0001 C CNN
F 1 "GND" V 13130 3372 50  0000 R CNN
F 2 "" H 13125 3500 50  0001 C CNN
F 3 "" H 13125 3500 50  0001 C CNN
	1    13125 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	13300 3500 13125 3500
$Comp
L power:+12V #PWR?
U 1 1 5F2F6868
P 13500 3300
F 0 "#PWR?" H 13500 3150 50  0001 C CNN
F 1 "+12V" H 13515 3473 50  0000 C CNN
F 2 "" H 13500 3300 50  0001 C CNN
F 3 "" H 13500 3300 50  0001 C CNN
	1    13500 3300
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR?
U 1 1 5F300DF7
P 13500 3900
F 0 "#PWR?" H 13500 4000 50  0001 C CNN
F 1 "-12V" H 13515 4073 50  0000 C CNN
F 2 "" H 13500 3900 50  0001 C CNN
F 3 "" H 13500 3900 50  0001 C CNN
	1    13500 3900
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R73
U 1 1 5F303CA2
P 13550 4375
F 0 "R73" V 13450 4175 50  0000 C CNN
F 1 "68K" V 13445 4375 50  0000 C CNN
F 2 "" H 13550 4375 50  0001 C CNN
F 3 "~" H 13550 4375 50  0001 C CNN
	1    13550 4375
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5F3040B7
P 13550 4600
F 0 "C4" V 13425 4375 50  0000 C CNN
F 1 "???" V 13412 4600 50  0000 C CNN
F 2 "" H 13550 4600 50  0001 C CNN
F 3 "~" H 13550 4600 50  0001 C CNN
	1    13550 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	13450 4375 13225 4375
Wire Wire Line
	13225 4375 13225 3700
Connection ~ 13225 3700
Wire Wire Line
	13225 3700 13300 3700
Wire Wire Line
	13225 4375 13225 4600
Wire Wire Line
	13225 4600 13450 4600
Connection ~ 13225 4375
Wire Wire Line
	13650 4375 13950 4375
Wire Wire Line
	13950 4375 13950 3600
Wire Wire Line
	13950 3600 13900 3600
Wire Wire Line
	13950 4375 13950 4600
Wire Wire Line
	13950 4600 13650 4600
Connection ~ 13950 4375
Wire Wire Line
	13950 3600 14450 3600
Connection ~ 13950 3600
Text Label 14875 3600 2    50   ~ 0
MOTOR+
$Comp
L Device:C_Small C18
U 1 1 5F36D630
P 14650 4075
F 0 "C18" H 14742 4121 50  0000 L CNN
F 1 "???" H 14742 4030 50  0000 L CNN
F 2 "" H 14650 4075 50  0001 C CNN
F 3 "~" H 14650 4075 50  0001 C CNN
	1    14650 4075
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5F36DD08
P 14450 4075
F 0 "R3" V 14350 4075 50  0000 C CNN
F 1 "24" V 14525 4075 50  0000 C CNN
F 2 "" H 14450 4075 50  0001 C CNN
F 3 "~" H 14450 4075 50  0001 C CNN
	1    14450 4075
	1    0    0    -1  
$EndComp
Wire Wire Line
	14450 3975 14450 3850
Connection ~ 14450 3600
Wire Wire Line
	14450 3600 14875 3600
Wire Wire Line
	14650 3975 14650 3850
Wire Wire Line
	14650 3850 14450 3850
Connection ~ 14450 3850
Wire Wire Line
	14450 3850 14450 3600
Wire Wire Line
	14450 4175 14450 4250
Wire Wire Line
	14450 4250 14650 4250
Wire Wire Line
	14650 4250 14650 4175
$Comp
L power:GND #PWR?
U 1 1 5F38DB04
P 14650 4325
F 0 "#PWR?" H 14650 4075 50  0001 C CNN
F 1 "GND" H 14655 4152 50  0000 C CNN
F 2 "" H 14650 4325 50  0001 C CNN
F 3 "" H 14650 4325 50  0001 C CNN
	1    14650 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	14650 4250 14650 4325
Connection ~ 14650 4250
$EndSCHEMATC
