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
LIBS:special
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
LIBS:opendous
LIBS:vreg
LIBS:board-cache
EELAYER 24 0
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
L PIC16F628A U2
U 1 1 53F0F681
P 4600 4000
F 0 "U2" H 4900 4800 60  0000 C CNN
F 1 "PIC16F628A" H 5050 3200 60  0000 C CNN
F 2 "" H 4600 4000 60  0001 C CNN
F 3 "" H 4600 4000 60  0001 C CNN
	1    4600 4000
	1    0    0    -1  
$EndComp
$Comp
L BC547 Q1
U 1 1 53F0F6F2
P 1875 6500
F 0 "Q1" H 1875 6351 40  0000 R CNN
F 1 "BC547" H 1875 6650 40  0000 R CNN
F 2 "TO92" H 1775 6602 29  0000 C CNN
F 3 "" H 1875 6500 60  0001 C CNN
	1    1875 6500
	1    0    0    -1  
$EndComp
$Comp
L SPEAKER SP1
U 1 1 53F0FF84
P 2275 6900
F 0 "SP1" H 2175 7150 70  0000 C CNN
F 1 "SPEAKER" H 2275 6550 70  0000 C CNN
F 2 "" H 2275 6900 60  0001 C CNN
F 3 "" H 2275 6900 60  0001 C CNN
	1    2275 6900
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 53F103D6
P 1375 6500
F 0 "R1" V 1455 6500 40  0000 C CNN
F 1 "1k" V 1382 6501 40  0000 C CNN
F 2 "" H 1375 6500 60  0001 C CNN
F 3 "" H 1375 6500 60  0001 C CNN
	1    1375 6500
	0    -1   -1   0   
$EndComp
$Comp
L CONN_4 BT
U 1 1 53F113F7
P 4350 6625
F 0 "BT MOD" V 4300 6625 50  0000 C CNN
F 1 "CONN_4" V 4400 6625 50  0000 C CNN
F 2 "" H 4350 6625 60  0001 C CNN
F 3 "" H 4350 6625 60  0001 C CNN
	1    4350 6625
	1    0    0    -1  
$EndComp
Text GLabel 3900 6475 0    60   Input ~ 0
+5V
Text GLabel 1975 6250 1    60   Input ~ 0
+5V
Text GLabel 2650 3350 0    60   Input ~ 0
+5V
$Comp
L CONN_6 Pickit3
U 1 1 53F11F1C
P 4350 6025
F 0 "Pickit3" V 4300 6025 60  0000 C CNN
F 1 "CONN_6" V 4400 6025 60  0000 C CNN
F 2 "" H 4350 6025 60  0001 C CNN
F 3 "" H 4350 6025 60  0001 C CNN
	1    4350 6025
	1    0    0    -1  
$EndComp
Text GLabel 3900 5875 0    60   Output ~ 0
+5V
$Comp
L C C3
U 1 1 53F1207E
P 3200 3650
F 0 "C3" H 3200 3750 40  0000 L CNN
F 1 "100nF" H 3206 3565 40  0000 L CNN
F 2 "" H 3200 3650 60  0001 C CNN
F 3 "" H 3200 3650 60  0001 C CNN
	1    3200 3650
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 53F120A3
P 2900 3950
F 0 "R2" V 2980 3950 40  0000 C CNN
F 1 "10k" V 2907 3951 40  0000 C CNN
F 2 "" H 2900 3950 60  0001 C CNN
F 3 "" H 2900 3950 60  0001 C CNN
	1    2900 3950
	1    0    0    -1  
$EndComp
Text GLabel 5775 1725 2    60   Output ~ 0
+5V
$Comp
L FUSE F1
U 1 1 53F13E56
P 2025 1350
F 0 "F1" H 2125 1400 40  0000 C CNN
F 1 "1A" H 1925 1300 40  0000 C CNN
F 2 "" H 2025 1350 60  0001 C CNN
F 3 "" H 2025 1350 60  0001 C CNN
	1    2025 1350
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 53F13EA7
P 3225 1975
F 0 "C1" H 3275 2075 40  0000 L CNN
F 1 "100uF" H 3275 1875 40  0000 L CNN
F 2 "" H 3225 1975 60  0001 C CNN
F 3 "" H 3225 1975 60  0001 C CNN
	1    3225 1975
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 53F13ECE
P 5625 1975
F 0 "C2" H 5675 2075 40  0000 L CNN
F 1 "6800uF" H 5675 1875 40  0000 L CNN
F 2 "" H 5625 1975 60  0001 C CNN
F 3 "" H 5625 1975 60  0001 C CNN
	1    5625 1975
	1    0    0    -1  
$EndComp
NoConn ~ 3550 3550
NoConn ~ 3550 3850
Text Label 975  6500 0    60   ~ 0
BZR
Text Label 6000 3550 0    60   ~ 0
BZR
Text Label 6000 3350 0    60   ~ 0
PK3_4
Text Label 6000 3450 0    60   ~ 0
PK3_5
Text Label 6000 3650 0    60   ~ 0
PK3_6
Text Label 3900 6275 2    60   ~ 0
PK3_6
Text Label 3900 6175 2    60   ~ 0
PK3_5
Text Label 3900 6075 2    60   ~ 0
PK3_4
Text Label 3900 5775 2    60   ~ 0
PK3_1
Text Label 3350 4350 2    60   ~ 0
PK3_1
Text Label 3900 6675 2    60   ~ 0
P1_RX
Text Label 6000 3950 0    60   ~ 0
P1_RX
Text Label 6000 3850 0    60   ~ 0
P1_TX
Text Label 3900 6775 2    60   ~ 0
P1_TX
Text Label 6000 3750 0    60   ~ 0
LED
NoConn ~ 5900 4050
NoConn ~ 5900 4250
$Comp
L LED D2
U 1 1 53F28C48
P 1725 5225
F 0 "D2" H 1725 5325 50  0000 C CNN
F 1 "LED" H 1725 5125 50  0000 C CNN
F 2 "" H 1725 5225 60  0001 C CNN
F 3 "" H 1725 5225 60  0001 C CNN
	1    1725 5225
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 53F29102
P 2325 5225
F 0 "R3" H 2405 5225 40  0000 C CNN
F 1 "1k" V 2332 5226 40  0000 C CNN
F 2 "" H 2325 5225 60  0001 C CNN
F 3 "" H 2325 5225 60  0001 C CNN
	1    2325 5225
	0    1    1    0   
$EndComp
Text Label 1325 5225 0    60   ~ 0
LED
$Comp
L PWR_FLAG #FLG09
U 1 1 53F2A9E2
P 5475 5325
F 0 "#FLG09" H 5475 5420 30  0001 C CNN
F 1 "PWR_FLAG" H 5475 5505 30  0000 C CNN
F 2 "" H 5475 5325 60  0001 C CNN
F 3 "" H 5475 5325 60  0001 C CNN
	1    5475 5325
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG010
U 1 1 53F2AC17
P 5100 6625
F 0 "#FLG010" H 5100 6720 30  0001 C CNN
F 1 "PWR_FLAG" H 5100 6805 30  0000 C CNN
F 2 "" H 5100 6625 60  0001 C CNN
F 3 "" H 5100 6625 60  0001 C CNN
	1    5100 6625
	1    0    0    -1  
$EndComp
$Comp
L LM2576 U1
U 1 1 54429063
P 3925 1625
F 0 "U1" H 3575 1875 60  0000 C CNN
F 1 "LM2576" H 4175 1875 60  0000 C CNN
F 2 "" H 3925 1625 60  0000 C CNN
F 3 "" H 3925 1625 60  0000 C CNN
F 4 "Texas Instruments" H 3925 1975 60  0001 C CNN "Manufacturer"
	1    3925 1625
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D2
U 1 1 544291E1
P 4775 1975
F 0 "D2" V 4775 2075 40  0000 C CNN
F 1 "DIODESCH" H 4775 1875 40  0000 C CNN
F 2 "" H 4775 1975 60  0000 C CNN
F 3 "" H 4775 1975 60  0000 C CNN
	1    4775 1975
	0    -1   -1   0   
$EndComp
$Comp
L INDUCTOR L1
U 1 1 54429228
P 5225 1725
F 0 "L1" V 5175 1725 40  0000 C CNN
F 1 "INDUCTOR" V 5325 1725 40  0000 C CNN
F 2 "" H 5225 1725 60  0000 C CNN
F 3 "" H 5225 1725 60  0000 C CNN
	1    5225 1725
	0    -1   -1   0   
$EndComp
$Comp
L VR VR1
U 1 1 54429DD8
P 2625 1775
F 0 "VR1" H 2775 1775 40  0000 C TNN
F 1 "VR" V 2625 1775 40  0000 C CNN
F 2 "" H 2625 1775 60  0000 C CNN
F 3 "" H 2625 1775 60  0000 C CNN
	1    2625 1775
	-1   0    0    1   
$EndComp
$Comp
L PC817 IC?
U 1 1 5443F4AB
P 8950 1550
F 0 "IC?" H 8740 1740 40  0000 C CNN
F 1 "PC817" H 9100 1360 40  0000 C CNN
F 2 "DIP4" H 8750 1370 30  0000 C CIN
F 3 "" H 8950 1550 60  0000 C CNN
	1    8950 1550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5443F9FA
P 8350 1750
F 0 "R?" V 8430 1750 40  0000 C CNN
F 1 "R" V 8357 1751 40  0000 C CNN
F 2 "" V 8280 1750 30  0000 C CNN
F 3 "" H 8350 1750 30  0000 C CNN
	1    8350 1750
	0    1    1    0   
$EndComp
$Comp
L MOSFET_N_Opendous Q?
U 1 1 5458198E
P 9775 1350
F 0 "Q?" H 9775 1540 30  0000 R CNN
F 1 "RFP50N06" H 10275 1325 30  0000 R CNN
F 2 "" H 9775 1350 60  0000 C CNN
F 3 "" H 9775 1350 60  0000 C CNN
	1    9775 1350
	1    0    0    -1  
$EndComp
$Comp
L MOSFET_N_Opendous Q?
U 1 1 54581A5C
P 9775 2250
F 0 "Q?" H 9775 2440 30  0000 R CNN
F 1 "RFP50N06" H 10275 2225 30  0000 R CNN
F 2 "" H 9775 2250 60  0000 C CNN
F 3 "" H 9775 2250 60  0000 C CNN
	1    9775 2250
	1    0    0    -1  
$EndComp
$Comp
L MOSFET_N_Opendous Q?
U 1 1 54581B2D
P 9775 3150
F 0 "Q?" H 9775 3340 30  0000 R CNN
F 1 "RFP50N06" H 10275 3125 30  0000 R CNN
F 2 "" H 9775 3150 60  0000 C CNN
F 3 "" H 9775 3150 60  0000 C CNN
	1    9775 3150
	1    0    0    -1  
$EndComp
$Comp
L MOSFET_N_Opendous Q?
U 1 1 54581BE7
P 9775 4050
F 0 "Q?" H 9775 4240 30  0000 R CNN
F 1 "RFP50N06" H 10275 4025 30  0000 R CNN
F 2 "" H 9775 4050 60  0000 C CNN
F 3 "" H 9775 4050 60  0000 C CNN
	1    9775 4050
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 54582232
P 8950 2000
F 0 "IC?" H 8740 2190 40  0000 C CNN
F 1 "PC817" H 9100 1810 40  0000 C CNN
F 2 "DIP4" H 8750 1820 30  0000 C CIN
F 3 "" H 8950 2000 60  0000 C CNN
	1    8950 2000
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 545822B3
P 8950 2450
F 0 "IC?" H 8740 2640 40  0000 C CNN
F 1 "PC817" H 9100 2260 40  0000 C CNN
F 2 "DIP4" H 8750 2270 30  0000 C CIN
F 3 "" H 8950 2450 60  0000 C CNN
	1    8950 2450
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 545822EF
P 8950 2900
F 0 "IC?" H 8740 3090 40  0000 C CNN
F 1 "PC817" H 9100 2710 40  0000 C CNN
F 2 "DIP4" H 8750 2720 30  0000 C CIN
F 3 "" H 8950 2900 60  0000 C CNN
	1    8950 2900
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 54582328
P 8950 3350
F 0 "IC?" H 8740 3540 40  0000 C CNN
F 1 "PC817" H 9100 3160 40  0000 C CNN
F 2 "DIP4" H 8750 3170 30  0000 C CIN
F 3 "" H 8950 3350 60  0000 C CNN
	1    8950 3350
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 545823B4
P 8950 3800
F 0 "IC?" H 8740 3990 40  0000 C CNN
F 1 "PC817" H 9100 3610 40  0000 C CNN
F 2 "DIP4" H 8750 3620 30  0000 C CIN
F 3 "" H 8950 3800 60  0000 C CNN
	1    8950 3800
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 5458240B
P 8950 4250
F 0 "IC?" H 8740 4440 40  0000 C CNN
F 1 "PC817" H 9100 4060 40  0000 C CNN
F 2 "DIP4" H 8750 4070 30  0000 C CIN
F 3 "" H 8950 4250 60  0000 C CNN
	1    8950 4250
	1    0    0    -1  
$EndComp
$Comp
L PC817 IC?
U 1 1 54582BA2
P 8950 4700
F 0 "IC?" H 8740 4890 40  0000 C CNN
F 1 "PC817" H 9100 4510 40  0000 C CNN
F 2 "DIP4" H 8750 4520 30  0000 C CIN
F 3 "" H 8950 4700 60  0000 C CNN
	1    8950 4700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54582D22
P 8350 2650
F 0 "R?" V 8430 2650 40  0000 C CNN
F 1 "R" V 8357 2651 40  0000 C CNN
F 2 "" V 8280 2650 30  0000 C CNN
F 3 "" H 8350 2650 30  0000 C CNN
	1    8350 2650
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54582D98
P 8350 3550
F 0 "R?" V 8430 3550 40  0000 C CNN
F 1 "R" V 8357 3551 40  0000 C CNN
F 2 "" V 8280 3550 30  0000 C CNN
F 3 "" H 8350 3550 30  0000 C CNN
	1    8350 3550
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54582E2B
P 8350 4450
F 0 "R?" V 8430 4450 40  0000 C CNN
F 1 "R" V 8357 4451 40  0000 C CNN
F 2 "" V 8280 4450 30  0000 C CNN
F 3 "" H 8350 4450 30  0000 C CNN
	1    8350 4450
	0    1    1    0   
$EndComp
$Comp
L LM7810ACT U?
U 1 1 54583F67
P 8950 5325
F 0 "U?" H 8750 5525 40  0000 C CNN
F 1 "LM7810ACT" H 8950 5525 40  0000 L CNN
F 2 "TO-220" H 8950 5425 30  0000 C CIN
F 3 "" H 8950 5325 60  0000 C CNN
	1    8950 5325
	1    0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 5458472A
P 9900 4600
F 0 "D?" H 9775 4500 40  0000 C CNN
F 1 "DIODE" H 9925 4500 40  0000 C CNN
F 2 "" H 9900 4600 60  0000 C CNN
F 3 "" H 9900 4600 60  0000 C CNN
	1    9900 4600
	-1   0    0    1   
$EndComp
$Comp
L DIODE D?
U 1 1 545864A1
P 9900 3700
F 0 "D?" H 9775 3600 40  0000 C CNN
F 1 "DIODE" H 9925 3600 40  0000 C CNN
F 2 "" H 9900 3700 60  0000 C CNN
F 3 "" H 9900 3700 60  0000 C CNN
	1    9900 3700
	-1   0    0    1   
$EndComp
$Comp
L DIODE D?
U 1 1 54586526
P 9900 2800
F 0 "D?" H 9775 2700 40  0000 C CNN
F 1 "DIODE" H 9925 2700 40  0000 C CNN
F 2 "" H 9900 2800 60  0000 C CNN
F 3 "" H 9900 2800 60  0000 C CNN
	1    9900 2800
	-1   0    0    1   
$EndComp
$Comp
L DIODE D?
U 1 1 545865BB
P 9900 1900
F 0 "D?" H 9775 1800 40  0000 C CNN
F 1 "DIODE" H 9925 1800 40  0000 C CNN
F 2 "" H 9900 1900 60  0000 C CNN
F 3 "" H 9900 1900 60  0000 C CNN
	1    9900 1900
	-1   0    0    1   
$EndComp
$Comp
L LED D3
U 1 1 545873B3
P 8300 1450
F 0 "D3" H 8250 1325 50  0000 C CNN
F 1 "LED" H 8400 1325 50  0000 C CNN
F 2 "" H 8300 1450 60  0000 C CNN
F 3 "" H 8300 1450 60  0000 C CNN
	1    8300 1450
	1    0    0    -1  
$EndComp
$Comp
L LED D4
U 1 1 54587428
P 8300 2100
F 0 "D4" H 8350 1975 50  0000 C CNN
F 1 "LED" H 8200 1975 50  0000 C CNN
F 2 "" H 8300 2100 60  0000 C CNN
F 3 "" H 8300 2100 60  0000 C CNN
	1    8300 2100
	-1   0    0    1   
$EndComp
$Comp
L LED D5
U 1 1 545874A1
P 8300 2350
F 0 "D5" H 8250 2225 50  0000 C CNN
F 1 "LED" H 8400 2225 50  0000 C CNN
F 2 "" H 8300 2350 60  0000 C CNN
F 3 "" H 8300 2350 60  0000 C CNN
	1    8300 2350
	1    0    0    -1  
$EndComp
$Comp
L LED D7
U 1 1 5458751E
P 8300 3250
F 0 "D7" H 8250 3125 50  0000 C CNN
F 1 "LED" H 8400 3125 50  0000 C CNN
F 2 "" H 8300 3250 60  0000 C CNN
F 3 "" H 8300 3250 60  0000 C CNN
	1    8300 3250
	1    0    0    -1  
$EndComp
$Comp
L LED D9
U 1 1 5458759B
P 8300 4150
F 0 "D9" H 8250 4025 50  0000 C CNN
F 1 "LED" H 8400 4025 50  0000 C CNN
F 2 "" H 8300 4150 60  0000 C CNN
F 3 "" H 8300 4150 60  0000 C CNN
	1    8300 4150
	1    0    0    -1  
$EndComp
$Comp
L LED D10
U 1 1 54587618
P 8300 4800
F 0 "D10" H 8350 4675 50  0000 C CNN
F 1 "LED" H 8200 4675 50  0000 C CNN
F 2 "" H 8300 4800 60  0000 C CNN
F 3 "" H 8300 4800 60  0000 C CNN
	1    8300 4800
	-1   0    0    1   
$EndComp
$Comp
L LED D8
U 1 1 545876A9
P 8300 3900
F 0 "D8" H 8350 3800 50  0000 C CNN
F 1 "LED" H 8200 3800 50  0000 C CNN
F 2 "" H 8300 3900 60  0000 C CNN
F 3 "" H 8300 3900 60  0000 C CNN
	1    8300 3900
	-1   0    0    1   
$EndComp
$Comp
L LED D6
U 1 1 54587730
P 8300 3000
F 0 "D6" H 8350 2900 50  0000 C CNN
F 1 "LED" H 8200 2900 50  0000 C CNN
F 2 "" H 8300 3000 60  0000 C CNN
F 3 "" H 8300 3000 60  0000 C CNN
	1    8300 3000
	-1   0    0    1   
$EndComp
Text GLabel 8000 1450 0    60   Input ~ 0
+5V
Text GLabel 8000 2350 0    60   Input ~ 0
+5V
Text GLabel 8000 3250 0    60   Input ~ 0
+5V
Text GLabel 8000 4150 0    60   Input ~ 0
+5V
$Comp
L C_MINI C?
U 1 1 54583944
P 9400 4500
F 0 "C?" V 9350 4540 30  0000 C CNN
F 1 "C_MINI" V 9450 4570 25  0000 C CNN
F 2 "" H 9400 4500 60  0000 C CNN
F 3 "" H 9400 4500 60  0000 C CNN
	1    9400 4500
	0    1    1    0   
$EndComp
$Comp
L C_MINI C?
U 1 1 54585C06
P 9400 3600
F 0 "C?" V 9350 3640 30  0000 C CNN
F 1 "C_MINI" V 9450 3670 25  0000 C CNN
F 2 "" H 9400 3600 60  0000 C CNN
F 3 "" H 9400 3600 60  0000 C CNN
	1    9400 3600
	0    1    1    0   
$EndComp
$Comp
L C_MINI C?
U 1 1 54585D8E
P 9400 2700
F 0 "C?" V 9350 2740 30  0000 C CNN
F 1 "C_MINI" V 9450 2770 25  0000 C CNN
F 2 "" H 9400 2700 60  0000 C CNN
F 3 "" H 9400 2700 60  0000 C CNN
	1    9400 2700
	0    1    1    0   
$EndComp
$Comp
L C_MINI C?
U 1 1 54585EC1
P 9400 1800
F 0 "C?" V 9350 1840 30  0000 C CNN
F 1 "C_MINI" V 9450 1870 25  0000 C CNN
F 2 "" H 9400 1800 60  0000 C CNN
F 3 "" H 9400 1800 60  0000 C CNN
	1    9400 1800
	0    1    1    0   
$EndComp
Text Label 10100 1150 0    39   ~ 0
S0_I
Text Label 10100 1550 0    39   ~ 0
S0_O
Text Label 10100 2050 0    39   ~ 0
S1_I
Text Label 10100 2450 0    39   ~ 0
S1_O
Text Label 10100 2950 0    39   ~ 0
S2_I
Text Label 10100 3350 0    39   ~ 0
S2_O
Text Label 10100 3850 0    39   ~ 0
S3_I
Text Label 10100 4250 0    39   ~ 0
S3_O
Text GLabel 10150 1900 2    39   Input ~ 0
+10V
Text GLabel 10150 2800 2    39   Input ~ 0
+10V
Text GLabel 10150 3700 2    39   Input ~ 0
+10V
Text GLabel 10150 4600 2    39   Input ~ 0
+10V
Text GLabel 9600 5275 2    39   Output ~ 0
+10V
$Comp
L CP C?
U 1 1 54598E1A
P 8450 5475
F 0 "C?" H 8500 5575 40  0000 L CNN
F 1 "CP" H 8500 5375 40  0000 L CNN
F 2 "" H 8550 5325 30  0000 C CNN
F 3 "" H 8450 5475 300 0000 C CNN
	1    8450 5475
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 54598E7A
P 9450 5475
F 0 "C?" H 9500 5575 40  0000 L CNN
F 1 "CP" H 9500 5375 40  0000 L CNN
F 2 "" H 9550 5325 30  0000 C CNN
F 3 "" H 9450 5475 300 0000 C CNN
	1    9450 5475
	1    0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 5459CD98
P 1525 1700
F 0 "D?" H 1450 1600 40  0000 C CNN
F 1 "DIODE" H 1625 1600 40  0000 C CNN
F 2 "" H 1525 1700 60  0000 C CNN
F 3 "" H 1525 1700 60  0000 C CNN
	1    1525 1700
	1    0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 5459CE17
P 2075 1700
F 0 "D?" H 2000 1600 40  0000 C CNN
F 1 "DIODE" H 2175 1600 40  0000 C CNN
F 2 "" H 2075 1700 60  0000 C CNN
F 3 "" H 2075 1700 60  0000 C CNN
	1    2075 1700
	-1   0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 5459CE91
P 2075 1950
F 0 "D?" H 2000 1850 40  0000 C CNN
F 1 "DIODE" H 2175 1850 40  0000 C CNN
F 2 "" H 2075 1950 60  0000 C CNN
F 3 "" H 2075 1950 60  0000 C CNN
	1    2075 1950
	1    0    0    -1  
$EndComp
$Comp
L DIODE D?
U 1 1 5459CEFE
P 1525 1950
F 0 "D?" H 1450 1850 40  0000 C CNN
F 1 "DIODE" H 1625 1850 40  0000 C CNN
F 2 "" H 1525 1950 60  0000 C CNN
F 3 "" H 1525 1950 60  0000 C CNN
	1    1525 1950
	-1   0    0    -1  
$EndComp
$Comp
L AGND #PWR?
U 1 1 545A7286
P 8950 5850
F 0 "#PWR?" H 8950 5850 40  0001 C CNN
F 1 "AGND" H 8950 5780 50  0000 C CNN
F 2 "" H 8950 5850 60  0000 C CNN
F 3 "" H 8950 5850 60  0000 C CNN
	1    8950 5850
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR?
U 1 1 545A7577
P 1325 2225
F 0 "#PWR?" H 1325 2225 40  0001 C CNN
F 1 "AGND" H 1325 2155 50  0000 C CNN
F 2 "" H 1325 2225 60  0000 C CNN
F 3 "" H 1325 2225 60  0000 C CNN
	1    1325 2225
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A7D8D
P 8050 3075
F 0 "#PWR?" H 8050 3075 40  0001 C CNN
F 1 "DGND" H 8050 3005 40  0000 C CNN
F 2 "" H 8050 3075 60  0000 C CNN
F 3 "" H 8050 3075 60  0000 C CNN
	1    8050 3075
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A7EF8
P 8050 3975
F 0 "#PWR?" H 8050 3975 40  0001 C CNN
F 1 "DGND" H 8050 3905 40  0000 C CNN
F 2 "" H 8050 3975 60  0000 C CNN
F 3 "" H 8050 3975 60  0000 C CNN
	1    8050 3975
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A7FF1
P 8050 2175
F 0 "#PWR?" H 8050 2175 40  0001 C CNN
F 1 "DGND" H 8050 2105 40  0000 C CNN
F 2 "" H 8050 2175 60  0000 C CNN
F 3 "" H 8050 2175 60  0000 C CNN
	1    8050 2175
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A8123
P 8050 4875
F 0 "#PWR?" H 8050 4875 40  0001 C CNN
F 1 "DGND" H 8050 4805 40  0000 C CNN
F 2 "" H 8050 4875 60  0000 C CNN
F 3 "" H 8050 4875 60  0000 C CNN
	1    8050 4875
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A8A90
P 1975 7325
F 0 "#PWR?" H 1975 7325 40  0001 C CNN
F 1 "DGND" H 1975 7255 40  0000 C CNN
F 2 "" H 1975 7325 60  0000 C CNN
F 3 "" H 1975 7325 60  0000 C CNN
	1    1975 7325
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A8DEA
P 3300 6725
F 0 "#PWR?" H 3300 6725 40  0001 C CNN
F 1 "DGND" H 3300 6655 40  0000 C CNN
F 2 "" H 3300 6725 60  0000 C CNN
F 3 "" H 3300 6725 60  0000 C CNN
	1    3300 6725
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A9114
P 2575 5450
F 0 "#PWR?" H 2575 5450 40  0001 C CNN
F 1 "DGND" H 2575 5380 40  0000 C CNN
F 2 "" H 2575 5450 60  0000 C CNN
F 3 "" H 2575 5450 60  0000 C CNN
	1    2575 5450
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A9684
P 3550 4825
F 0 "#PWR?" H 3550 4825 40  0001 C CNN
F 1 "DGND" H 3550 4755 40  0000 C CNN
F 2 "" H 3550 4825 60  0000 C CNN
F 3 "" H 3550 4825 60  0000 C CNN
	1    3550 4825
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A9975
P 3200 3975
F 0 "#PWR?" H 3200 3975 40  0001 C CNN
F 1 "DGND" H 3200 3905 40  0000 C CNN
F 2 "" H 3200 3975 60  0000 C CNN
F 3 "" H 3200 3975 60  0000 C CNN
	1    3200 3975
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545A9D11
P 4275 2575
F 0 "#PWR?" H 4275 2575 40  0001 C CNN
F 1 "DGND" H 4275 2505 40  0000 C CNN
F 2 "" H 4275 2575 60  0000 C CNN
F 3 "" H 4275 2575 60  0000 C CNN
	1    4275 2575
	1    0    0    -1  
$EndComp
$Comp
L DGND #PWR?
U 1 1 545AA4FB
P 3300 6125
F 0 "#PWR?" H 3300 6125 40  0001 C CNN
F 1 "DGND" H 3300 6055 40  0000 C CNN
F 2 "" H 3300 6125 60  0000 C CNN
F 3 "" H 3300 6125 60  0000 C CNN
	1    3300 6125
	1    0    0    -1  
$EndComp
Text Label 8125 5300 0    39   ~ 0
S0_I
Connection ~ 2625 1525
Wire Wire Line
	2625 2025 2625 2175
Connection ~ 3225 2175
Connection ~ 4025 2175
Connection ~ 4275 2175
Connection ~ 3825 2175
Connection ~ 4775 2175
Wire Wire Line
	1800 2175 5625 2175
Connection ~ 4475 2175
Wire Wire Line
	5525 1725 5775 1725
Wire Wire Line
	5625 1525 4625 1525
Wire Wire Line
	5625 1525 5625 1775
Wire Wire Line
	4775 1725 4775 1775
Wire Wire Line
	4625 1725 4925 1725
Connection ~ 4775 1725
Connection ~ 5625 1725
Wire Wire Line
	3225 1525 3225 1775
Wire Wire Line
	1975 7000 1975 7325
Wire Wire Line
	3200 3850 3200 3975
Wire Wire Line
	2575 5225 2575 5450
Wire Wire Line
	4275 2175 4275 2575
Wire Wire Line
	1325 5225 1525 5225
Wire Wire Line
	2075 5225 1925 5225
Wire Wire Line
	3300 5975 3300 6125
Wire Wire Line
	4000 5975 3300 5975
Wire Wire Line
	6000 3750 5900 3750
Wire Wire Line
	3300 6575 3300 6725
Wire Wire Line
	3550 4650 3550 4825
Wire Wire Line
	3350 4350 3350 4200
Wire Wire Line
	3900 6275 4000 6275
Wire Wire Line
	6000 3650 5900 3650
Wire Wire Line
	5900 4650 6250 4650
Wire Wire Line
	5900 4550 6250 4550
Wire Wire Line
	5900 4450 6250 4450
Wire Wire Line
	5900 4350 6250 4350
Wire Wire Line
	2900 4200 3550 4200
Wire Wire Line
	2650 3350 3550 3350
Connection ~ 3350 4200
Wire Wire Line
	5900 3950 6000 3950
Wire Wire Line
	5900 3850 6000 3850
Wire Wire Line
	1625 6500 1675 6500
Wire Wire Line
	1975 6700 1975 6800
Wire Wire Line
	975  6500 1125 6500
Wire Wire Line
	3900 6675 4000 6675
Wire Wire Line
	3900 6775 4000 6775
Wire Wire Line
	3900 6475 4000 6475
Wire Wire Line
	1975 6250 1975 6300
Connection ~ 3200 3350
Connection ~ 2900 3350
Wire Wire Line
	2900 3350 2900 3700
Wire Wire Line
	3200 3350 3200 3450
Wire Wire Line
	6000 3550 5900 3550
Wire Wire Line
	3900 5775 4000 5775
Wire Wire Line
	3900 5875 4000 5875
Wire Wire Line
	4000 6075 3900 6075
Wire Wire Line
	4000 6175 3900 6175
Wire Wire Line
	5900 3350 6000 3350
Wire Wire Line
	5900 3450 6000 3450
Wire Wire Line
	3300 6575 4000 6575
Wire Wire Line
	8600 4350 8600 4600
Connection ~ 8600 4450
Wire Wire Line
	8600 3450 8600 3700
Connection ~ 8600 3550
Wire Wire Line
	8600 2550 8600 2800
Connection ~ 8600 2650
Wire Wire Line
	8600 1650 8600 1900
Connection ~ 8600 1750
Wire Wire Line
	8500 1450 8600 1450
Wire Wire Line
	8600 2100 8500 2100
Wire Wire Line
	8600 2350 8500 2350
Wire Wire Line
	8600 3000 8500 3000
Wire Wire Line
	8600 3250 8500 3250
Wire Wire Line
	8600 3900 8500 3900
Wire Wire Line
	8600 4150 8500 4150
Wire Wire Line
	8600 4800 8500 4800
Wire Wire Line
	8100 1450 8000 1450
Wire Wire Line
	8100 2350 8000 2350
Wire Wire Line
	8100 3250 8000 3250
Wire Wire Line
	8100 4150 8000 4150
Wire Wire Line
	8100 3900 8050 3900
Wire Wire Line
	9300 1900 9700 1900
Connection ~ 9400 1900
Wire Wire Line
	9300 2800 9700 2800
Connection ~ 9400 2800
Wire Wire Line
	9300 3700 9700 3700
Connection ~ 9400 3700
Wire Wire Line
	9300 4600 9700 4600
Connection ~ 9400 4600
Wire Wire Line
	9400 4350 9400 4400
Connection ~ 9400 3450
Wire Wire Line
	9400 3450 9400 3500
Connection ~ 9400 4350
Wire Wire Line
	9400 1650 9400 1700
Connection ~ 9400 1650
Wire Wire Line
	9400 2550 9400 2600
Connection ~ 9400 2550
Wire Wire Line
	9300 4150 9575 4150
Wire Wire Line
	9300 3250 9575 3250
Wire Wire Line
	9575 2350 9300 2350
Wire Wire Line
	9575 1450 9300 1450
Wire Wire Line
	9300 4350 9875 4350
Wire Wire Line
	9875 4350 9875 4250
Wire Wire Line
	9575 4150 9575 4800
Wire Wire Line
	9575 4800 9300 4800
Wire Wire Line
	9300 3900 9575 3900
Wire Wire Line
	9575 3900 9575 3250
Wire Wire Line
	9300 3450 9875 3450
Wire Wire Line
	9875 3450 9875 3350
Wire Wire Line
	9300 3000 9575 3000
Wire Wire Line
	9575 3000 9575 2350
Wire Wire Line
	9875 2550 9875 2450
Wire Wire Line
	9300 2550 9875 2550
Wire Wire Line
	9300 1650 9875 1650
Wire Wire Line
	9875 1650 9875 1550
Wire Wire Line
	9300 2100 9575 2100
Wire Wire Line
	9575 2100 9575 1450
Wire Wire Line
	9875 2050 10100 2050
Wire Wire Line
	9875 2450 10100 2450
Wire Wire Line
	9875 1550 10100 1550
Wire Wire Line
	9875 1150 10100 1150
Wire Wire Line
	9875 2950 10100 2950
Wire Wire Line
	9875 3350 10100 3350
Wire Wire Line
	9875 3850 10100 3850
Wire Wire Line
	9875 4250 10100 4250
Wire Wire Line
	10100 1900 10150 1900
Wire Wire Line
	10150 2800 10100 2800
Wire Wire Line
	10150 3700 10100 3700
Wire Wire Line
	10100 4600 10150 4600
Wire Wire Line
	9350 5275 9600 5275
Connection ~ 9450 5275
Wire Wire Line
	8275 5275 8550 5275
Connection ~ 8450 5275
Connection ~ 1800 1700
Connection ~ 1800 1950
Wire Wire Line
	1725 1700 1875 1700
Wire Wire Line
	1725 1950 1875 1950
Wire Wire Line
	2275 1350 2275 1950
Wire Wire Line
	1800 1525 3225 1525
Wire Wire Line
	1800 1525 1800 1700
Wire Wire Line
	1800 1950 1800 2175
Connection ~ 2625 2175
Connection ~ 2275 1700
Connection ~ 1325 1700
Wire Wire Line
	8950 5575 8950 5850
Wire Wire Line
	8450 5675 9450 5675
Connection ~ 8950 5675
Connection ~ 1325 1950
Wire Wire Line
	8050 3900 8050 3975
Wire Wire Line
	8050 2175 8050 2100
Wire Wire Line
	8050 2100 8100 2100
Wire Wire Line
	8050 3075 8050 3000
Wire Wire Line
	8050 3000 8100 3000
Wire Wire Line
	8050 4875 8050 4800
Wire Wire Line
	8050 4800 8100 4800
Wire Wire Line
	8100 2650 7850 2650
Wire Wire Line
	8100 1750 7850 1750
Wire Wire Line
	8100 3550 7850 3550
Wire Wire Line
	8100 4450 7850 4450
Text Label 6250 4650 0    39   ~ 0
PWRSW
Text Label 7850 1750 0    39   ~ 0
PWRSW
Text Label 7850 2650 0    39   ~ 0
CSW_1
Text Label 7850 3550 0    39   ~ 0
CSW_2
Text Label 7850 4450 0    39   ~ 0
CSW_3
Text Label 6250 4550 0    39   ~ 0
CSW_1
Text Label 6250 4450 0    39   ~ 0
CSW_2
Text Label 6250 4350 0    39   ~ 0
CSW_3
$Comp
L CONN_01X03 P?
U 1 1 545AEC09
P 5875 5575
F 0 "P?" H 5875 5775 50  0000 C CNN
F 1 "SCREW TERM 1" H 6250 5575 50  0000 C CNN
F 2 "" H 5875 5575 60  0000 C CNN
F 3 "" H 5875 5575 60  0000 C CNN
	1    5875 5575
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 545AFCB9
P 5875 6150
F 0 "P?" H 5875 6350 50  0000 C CNN
F 1 "SCREW TERM 2" H 6250 6150 50  0000 C CNN
F 2 "" H 5875 6150 60  0000 C CNN
F 3 "" H 5875 6150 60  0000 C CNN
	1    5875 6150
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 545AFD22
P 5875 6700
F 0 "P?" H 5875 6900 50  0000 C CNN
F 1 "SCREW TERM 3" H 6250 6700 50  0000 C CNN
F 2 "" H 5875 6700 60  0000 C CNN
F 3 "" H 5875 6700 60  0000 C CNN
	1    5875 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5475 5475 5675 5475
Wire Wire Line
	5675 6050 5475 6050
Wire Wire Line
	5675 6250 5300 6250
Wire Wire Line
	5675 6150 5400 6150
Wire Wire Line
	5675 5675 5300 5675
Wire Wire Line
	5675 5575 5400 5575
Wire Wire Line
	5675 6600 5475 6600
Wire Wire Line
	5675 6700 5400 6700
Text Label 5500 5475 0    39   ~ 0
S0_I
Wire Wire Line
	5475 5475 5475 5325
Wire Wire Line
	1775 1350 1625 1350
Text Label 1625 1350 0    39   ~ 0
S0_I
$Comp
L AGND #PWR?
U 1 1 545B2C19
P 5100 6950
F 0 "#PWR?" H 5100 6950 40  0001 C CNN
F 1 "AGND" H 5100 6880 50  0000 C CNN
F 2 "" H 5100 6950 60  0000 C CNN
F 3 "" H 5100 6950 60  0000 C CNN
	1    5100 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 6625 5100 6950
Connection ~ 5100 6800
Wire Wire Line
	1325 1700 1325 2225
Text Label 5400 5575 0    39   ~ 0
S0_O
Text Label 5300 5675 0    39   ~ 0
S1_I
Text Label 5475 6050 0    39   ~ 0
S1_O
Text Label 5400 6150 0    39   ~ 0
S2_I
Text Label 5300 6250 0    39   ~ 0
S2_O
Text Label 5475 6600 0    39   ~ 0
S3_I
Text Label 5400 6700 0    39   ~ 0
S3_O
Wire Wire Line
	5675 6800 5100 6800
$EndSCHEMATC
