/*  Arduino UNO R4 code for general method to attach an interrupt to ANY event
 *                 plus fast ADC and digital pin operation
 *
 *  Susan Parker - 19th June 2023.
 *
 * This code is "AS IS" without warranty or liability. 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

/* For external aref - ADR4540 - Ultralow Noise, High Accuracy Voltage Reference
   Using an aref gives c. +- 1 to 2 value in 14 bit reads on USB power
https://www.analog.com/media/en/technical-documentation/data-sheets/adr4520_4525_4530_4533_4540_4550.pdf
*/


#include "Arduino.h"

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

#define ICUBASE 0x40000000 // ICU Base - See 13.2.6 page 233
// 32 bits - 
#define IELSR 0x6300 // ICU Event Link Setting Register n
#define ICU_IELSR00 ((volatile unsigned int *)(ICUBASE + IELSR))            //
#define ICU_IELSR01 ((volatile unsigned int *)(ICUBASE + IELSR + ( 1 * 4))) // 
#define ICU_IELSR02 ((volatile unsigned int *)(ICUBASE + IELSR + ( 2 * 4))) // 
#define ICU_IELSR03 ((volatile unsigned int *)(ICUBASE + IELSR + ( 3 * 4))) // 
#define ICU_IELSR04 ((volatile unsigned int *)(ICUBASE + IELSR + ( 4 * 4))) // 
#define ICU_IELSR05 ((volatile unsigned int *)(ICUBASE + IELSR + ( 5 * 4))) // 
#define ICU_IELSR06 ((volatile unsigned int *)(ICUBASE + IELSR + ( 6 * 4))) // 
#define ICU_IELSR07 ((volatile unsigned int *)(ICUBASE + IELSR + ( 7 * 4))) // 
#define ICU_IELSR08 ((volatile unsigned int *)(ICUBASE + IELSR + ( 8 * 4))) // 
#define ICU_IELSR09 ((volatile unsigned int *)(ICUBASE + IELSR + ( 9 * 4))) // 
#define ICU_IELSR10 ((volatile unsigned int *)(ICUBASE + IELSR + (10 * 4))) // 
#define ICU_IELSR11 ((volatile unsigned int *)(ICUBASE + IELSR + (11 * 4))) // 
#define ICU_IELSR12 ((volatile unsigned int *)(ICUBASE + IELSR + (12 * 4))) // 
#define ICU_IELSR13 ((volatile unsigned int *)(ICUBASE + IELSR + (13 * 4))) // 
#define ICU_IELSR14 ((volatile unsigned int *)(ICUBASE + IELSR + (14 * 4))) // 
#define ICU_IELSR15 ((volatile unsigned int *)(ICUBASE + IELSR + (15 * 4))) // 
#define ICU_IELSR16 ((volatile unsigned int *)(ICUBASE + IELSR + (16 * 4))) // 
#define ICU_IELSR17 ((volatile unsigned int *)(ICUBASE + IELSR + (17 * 4))) // 
#define ICU_IELSR18 ((volatile unsigned int *)(ICUBASE + IELSR + (18 * 4))) // 
#define ICU_IELSR19 ((volatile unsigned int *)(ICUBASE + IELSR + (19 * 4))) // 
#define ICU_IELSR20 ((volatile unsigned int *)(ICUBASE + IELSR + (20 * 4))) // 
#define ICU_IELSR21 ((volatile unsigned int *)(ICUBASE + IELSR + (21 * 4))) // 
#define ICU_IELSR22 ((volatile unsigned int *)(ICUBASE + IELSR + (22 * 4))) // 
#define ICU_IELSR23 ((volatile unsigned int *)(ICUBASE + IELSR + (23 * 4))) // 
#define ICU_IELSR24 ((volatile unsigned int *)(ICUBASE + IELSR + (24 * 4))) // 
#define ICU_IELSR25 ((volatile unsigned int *)(ICUBASE + IELSR + (25 * 4))) // 
#define ICU_IELSR26 ((volatile unsigned int *)(ICUBASE + IELSR + (26 * 4))) // 
#define ICU_IELSR27 ((volatile unsigned int *)(ICUBASE + IELSR + (27 * 4))) // 
#define ICU_IELSR28 ((volatile unsigned int *)(ICUBASE + IELSR + (28 * 4))) // 
#define ICU_IELSR29 ((volatile unsigned int *)(ICUBASE + IELSR + (29 * 4))) // 
#define ICU_IELSR30 ((volatile unsigned int *)(ICUBASE + IELSR + (30 * 4))) // 
#define ICU_IELSR31 ((volatile unsigned int *)(ICUBASE + IELSR + (31 * 4))) // 


// =========== ADC14 ============
// 35.2 Register Descriptions

#define ADCBASE 0x40050000 /* ADC Base */

#define ADC140_ADCSR   ((volatile unsigned short *)(ADCBASE + 0xC000)) // A/D Control Register
#define ADC140_ADANSA0 ((volatile unsigned short *)(ADCBASE + 0xC004)) // A/D Channel Select Register A0
#define ADC140_ADANSA1 ((volatile unsigned short *)(ADCBASE + 0xC006)) // A/D Channel Select Register A1
#define ADC140_ADADS0  ((volatile unsigned short *)(ADCBASE + 0xC008)) // A/D-Converted Value Addition/Average Channel Select Register 0
#define ADC140_ADADS1  ((volatile unsigned short *)(ADCBASE + 0xC00A)) // A/D-Converted Value Addition/Average Channel Select Register 1
#define ADC140_ADCER   ((volatile unsigned short *)(ADCBASE + 0xC00E)) // A/D Control Extended Register 
#define ADC140_ADSTRGR ((volatile unsigned short *)(ADCBASE + 0xC010)) // A/D Conversion Start Trigger Select Register
#define ADC140_ADEXICR ((volatile unsigned short *)(ADCBASE + 0xC012)) // A/D Conversion Extended Input Control Register
#define ADC140_ADANSB0 ((volatile unsigned short *)(ADCBASE + 0xC014)) // A/D Channel Select Register B0
#define ADC140_ADANSB1 ((volatile unsigned short *)(ADCBASE + 0xC016)) // A/D Channel Select Register B1
#define ADC140_ADTSDR  ((volatile unsigned short *)(ADCBASE + 0xC01A)) // A/D conversion result of temperature sensor output
#define ADC140_ADOCDR  ((volatile unsigned short *)(ADCBASE + 0xC01C)) // A/D result of internal reference voltage
#define ADC140_ADRD    ((volatile unsigned short *)(ADCBASE + 0xC01E)) // A/D Self-Diagnosis Data Register

#define ADC140_ADDR00 ((volatile unsigned short *)(ADCBASE + 0xC020))      // A1 (P000 AN00 AMP+)
#define ADC140_ADDR01 ((volatile unsigned short *)(ADCBASE + 0xC020 +  2)) // A2 (P001 AN01 AMP-) 
#define ADC140_ADDR02 ((volatile unsigned short *)(ADCBASE + 0xC020 +  4)) // A3 (P002 AN02 AMPO) 
#define ADC140_ADDR05 ((volatile unsigned short *)(ADCBASE + 0xC020 + 10)) // Aref (P010 AN05 VrefH0)
#define ADC140_ADDR09 ((volatile unsigned short *)(ADCBASE + 0xC020 + 18)) // A0 (P014 AN09 DAC)
#define ADC140_ADDR21 ((volatile unsigned short *)(ADCBASE + 0xC040 + 10)) // A4 (P101 AN21 SDA) 
#define ADC140_ADDR22 ((volatile unsigned short *)(ADCBASE + 0xC040 + 12)) // A5 (P100 AN20 SCL) 

#define ADC140_ADHVREFCNT ((volatile unsigned char  *)(ADCBASE + 0xC08A)) // A/D High-Potential/Low-Potential Reference Voltage Control Register
#define ADC140_ADADC      ((volatile unsigned char  *)(ADCBASE + 0xC00C)) // A/D-Converted Value Addition/Average Count Select Register


// =========== Ports ============
// 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15)

#define PORTBASE 0x40040000 /* Port Base */
#define PFS_P100PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843))   // 8 bits - A5
#define PFS_P101PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 1 * 4))) // A4
#define PFS_P102PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 2 * 4))) // D5
#define PFS_P103PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 3 * 4))) // D4
#define PFS_P104PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 4 * 4))) // D3
#define PFS_P105PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 5 * 4))) // D2
#define PFS_P106PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 6 * 4))) // D6
#define PFS_P107PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 7 * 4))) // D7
#define PFS_P108PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 8 * 4))) // SWDIO
#define PFS_P109PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 9 * 4))) // D11 / MOSI
#define PFS_P110PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (10 * 4))) // D12 / MISO
#define PFS_P111PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (11 * 4))) // D13 / SCLK
#define PFS_P112PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (12 * 4))) // D10 / CS
#define PFS_P300PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3))            // SWCLK (P300)
#define PFS_P301PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (01 * 4))) // D0 / RxD (P301)
#define PFS_P302PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (02 * 4))) // D1 / TxD (P302) 
#define PFS_P303PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (03 * 4))) // D9
#define PFS_P304PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (04 * 4))) // D8


unsigned int int_val;
unsigned short short_val;
unsigned char char_val; 


void setup()
  {                                                   // Pins for interrupts: 0, 1, 2, 3, 8, 12, 13, (15?), 16, 17, 18 and 19
  attachInterrupt(2, adcCompleteInterrupt, FALLING);  // This IRQ will be asigned to Slot 05 IELSR05 as 0x001 PORT_IRQ0 - Table 13.4
  *ICU_IELSR05 = 0x029;                               // HiJack Slot 05 IELSR05 for ADC140_ADI

  *PFS_P107PFS_BY = 0x04;                          // Set D7 output low - IRQ time flag pin

  Serial.begin(115200);
  while (!Serial){};

  setup_adc();
  print_icu_event_links();
  }
  
void loop()
  {
  uint16_t adc_val_16;

  *PFS_P103PFS_BY = 0x05;      // Pulse on D4 to trigger scope 
  *PFS_P103PFS_BY = 0x04;      //  

   // Set High, read ADC register, trigger Conversion, set Low = c. 500nS
  *PFS_P107PFS_BY = 0x05;         // digitalWrite(monitorPin, HIGH);   // Digital Pin D7
  adc_val_16 = *ADC140_ADDR09;    // adcValue = analogRead(analogPin); // Internal 16bit register read = c. 123nS 
  *ADC140_ADCSR |= (0x01 << 15);  // Next ADC conversion = write to register c. 300nS
  *PFS_P107PFS_BY = 0x04;         // digitalWrite(monitorPin, LOW);  

/*
  *PFS_P107PFS_BY = 0x05;      // Each Port Output bit clear to set takes c. 82nS 
  *PFS_P107PFS_BY = 0x04;      // Each Port Output bit set to clear takes c. 85nS 

  *PFS_P107PFS_BY = 0x05;      // Set HIGH
  char_val = *PFS_P107PFS_BY;  // Port State Input read - takes about 165nS
  *PFS_P107PFS_BY = 0x04;      // Read plus Set LOW = c. 250nS
 */   
  Serial.println(adc_val_16);
  
  delay(100);
  }

void adcCompleteInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;      // D7 
  *PFS_P107PFS_BY = 0x04;      //  
  }

void setup_adc(void)
  {
//  *ADC140_ADHVREFCNT = 0x01;         // Set External Aref = analogReference(AR_EXTERNAL);      
  *ADC140_ADCER = 0x06;              // 14 bit mode, clear ACE bit 5
  *ADC140_ADCSR |= (0x01 << 6);      // Set b6 - GBADIE Group B Scan End Interrupt Enable
  *ADC140_ADANSA0 |= (0x01 << 9);    // Selected ANSA09
  *ADC140_ADCSR |= (0x01 << 15);     // Start a conversion
  }


// Function: print_icu_event_links();
// The following is to determin which interrupts are in use
// RA4M1 Group ICU Event Number Table 13.4 
//
// Slot - Event Number - Name of the Interrupt
// 0 - 33 - USBFS_USBI
// 1 - 34 - USBFS_USBR
// 2 - 31 - USBFS_D0FIFO
// 3 - 32 - USBFS_D1FIFO
// 4 - 1E - AGT0_AGTI
//
// The above 5 entries are always present before the code gets to setup()

// Use PROGMEM structures to place strings into program memory 
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

const char string_00[] PROGMEM = "No_Event";
const char string_01[] PROGMEM = "PORT_IRQ0";
const char string_02[] PROGMEM = "PORT_IRQ1";
const char string_03[] PROGMEM = "PORT_IRQ2";
const char string_04[] PROGMEM = "PORT_IRQ3";
const char string_05[] PROGMEM = "PORT_IRQ4";
const char string_06[] PROGMEM = "PORT_IRQ5";
const char string_07[] PROGMEM = "PORT_IRQ6";
const char string_08[] PROGMEM = "PORT_IRQ7";
const char string_09[] PROGMEM = "PORT_IRQ8";
const char string_0A[] PROGMEM = "PORT_IRQ9";
const char string_0B[] PROGMEM = "PORT_IRQ10";
const char string_0C[] PROGMEM = "PORT_IRQ11";
const char string_0D[] PROGMEM = "PORT_IRQ12";
const char string_0E[] PROGMEM = "PORT_UNUSED";
const char string_0F[] PROGMEM = "PORT_IRQ14";
const char string_10[] PROGMEM = "PORT_IRQ15";
const char string_11[] PROGMEM = "DMAC0_INT";
const char string_12[] PROGMEM = "DMAC1_INT";
const char string_13[] PROGMEM = "DMAC2_INT";
const char string_14[] PROGMEM = "DMAC3_INT";
const char string_15[] PROGMEM = "DTC_COMPLETE";
const char string_16[] PROGMEM = "UNUSED";
const char string_17[] PROGMEM = "ICU_SNZCANCEL";
const char string_18[] PROGMEM = "FCU_FRDYI";
const char string_19[] PROGMEM = "LVD_LVD1";
const char string_1A[] PROGMEM = "LVD_LVD2";
const char string_1B[] PROGMEM = "VBATT_LVD";
const char string_1C[] PROGMEM = "MOSC_STOP";
const char string_1D[] PROGMEM = "SYSTEM_SNZREQ";
const char string_1E[] PROGMEM = "AGT0_AGTI";
const char string_1F[] PROGMEM = "AGT0_AGTCMAI";
const char string_20[] PROGMEM = "AGT0_AGTCMBI";
const char string_21[] PROGMEM = "AGT1_AGTI";
const char string_22[] PROGMEM = "AGT1_AGTCMAI";
const char string_23[] PROGMEM = "AGT1_AGTCMBI";
const char string_24[] PROGMEM = "IWDT_NMIUNDF";
const char string_25[] PROGMEM = "WDT_NMIUNDF";
const char string_26[] PROGMEM = "RTC_ALM";
const char string_27[] PROGMEM = "RTC_PRD";
const char string_28[] PROGMEM = "RTC_CUP";
const char string_29[] PROGMEM = "ADC140_ADI";
const char string_2A[] PROGMEM = "ADC140_GBADI";
const char string_2B[] PROGMEM = "ADC140_CMPAI";
const char string_2C[] PROGMEM = "ADC140_CMPBI";
const char string_2D[] PROGMEM = "ADC140_WCMPM";
const char string_2E[] PROGMEM = "ADC140_WCMPUM";
const char string_2F[] PROGMEM = "ACMP_LP0";
const char string_30[] PROGMEM = "ACMP_LP1";
const char string_31[] PROGMEM = "USBFS_D0FIFO";
const char string_32[] PROGMEM = "USBFS_D1FIFO";
const char string_33[] PROGMEM = "USBFS_USBI";
const char string_34[] PROGMEM = "USBFS_USBR";
const char string_35[] PROGMEM = "IIC0_RXI";
const char string_36[] PROGMEM = "IIC0_TXI";
const char string_37[] PROGMEM = "IIC0_TEI";
const char string_38[] PROGMEM = "IIC0_EEI";
const char string_39[] PROGMEM = "IIC0_WUI";
const char string_3A[] PROGMEM = "IIC1_RXI";
const char string_3B[] PROGMEM = "IIC1_TXI";
const char string_3C[] PROGMEM = "IIC1_TEI";
const char string_3D[] PROGMEM = "IIC1_EEI";
const char string_3E[] PROGMEM = "SSIE0_SSITXI";
const char string_3F[] PROGMEM = "SSIE0_SSIRXI";
const char string_40[] PROGMEM = "UNUSED";
const char string_41[] PROGMEM = "SSIE0_SSIF";
const char string_42[] PROGMEM = "CTSU_CTSUWR";
const char string_43[] PROGMEM = "CTSU_CTSURD";
const char string_44[] PROGMEM = "CTSU_CTSUFN";
const char string_45[] PROGMEM = "KEY_INTKR";
const char string_46[] PROGMEM = "DOC_DOPCI";
const char string_47[] PROGMEM = "CAC_FERRI";
const char string_48[] PROGMEM = "CAC_MENDI";
const char string_49[] PROGMEM = "CAC_OVFI";
const char string_4A[] PROGMEM = "CAN0_ERS";
const char string_4B[] PROGMEM = "CAN0_RXF";
const char string_4C[] PROGMEM = "CAN0_TXF";
const char string_4D[] PROGMEM = "CAN0_RXM";
const char string_4E[] PROGMEM = "CAN0_TXM";
const char string_4F[] PROGMEM = "IOPORT_GROUP1";
const char string_50[] PROGMEM = "IOPORT_GROUP2";
const char string_51[] PROGMEM = "IOPORT_GROUP3";
const char string_52[] PROGMEM = "IOPORT_GROUP4";
const char string_53[] PROGMEM = "ELC_SWEVT0";
const char string_54[] PROGMEM = "ELC_SWEVT1";
const char string_55[] PROGMEM = "POEG_GROUP0";
const char string_56[] PROGMEM = "POEG_GROUP1";
const char string_57[] PROGMEM = "GPT0_CCMPA";
const char string_58[] PROGMEM = "GPT0_CCMPB";
const char string_59[] PROGMEM = "GPT0_CMPC";
const char string_5A[] PROGMEM = "GPT0_CMPD";
const char string_5B[] PROGMEM = "GPT0_CMPE";
const char string_5C[] PROGMEM = "GPT0_CMPF";
const char string_5D[] PROGMEM = "GPT0_OVF";
const char string_5E[] PROGMEM = "GPT0_UDF";
const char string_5F[] PROGMEM = "GPT1_CCMPA";
const char string_60[] PROGMEM = "GPT1_CCMPB";
const char string_61[] PROGMEM = "GPT1_CMPC";
const char string_62[] PROGMEM = "GPT1_CMPD";
const char string_63[] PROGMEM = "GPT1_CMPE";
const char string_64[] PROGMEM = "GPT1_CMPF";
const char string_65[] PROGMEM = "GPT1_OVF";
const char string_66[] PROGMEM = "GPT1_UDF";
const char string_67[] PROGMEM = "GPT2_CCMPA";
const char string_68[] PROGMEM = "GPT2_CCMPB";
const char string_69[] PROGMEM = "GPT2_CMPC";
const char string_6A[] PROGMEM = "GPT2_CMPD";
const char string_6B[] PROGMEM = "GPT2_CMPE";
const char string_6C[] PROGMEM = "GPT2_CMPF";
const char string_6D[] PROGMEM = "GPT2_OVF";
const char string_6E[] PROGMEM = "GPT2_UDF";
const char string_6F[] PROGMEM = "GPT3_CCMPA";
const char string_70[] PROGMEM = "GPT3_CCMPB";
const char string_71[] PROGMEM = "GPT3_CMPC";
const char string_72[] PROGMEM = "GPT3_CMPD";
const char string_73[] PROGMEM = "GPT3_CMPE";
const char string_74[] PROGMEM = "GPT3_CMPF";
const char string_75[] PROGMEM = "GPT3_OVF";
const char string_76[] PROGMEM = "GPT3_UDF";
const char string_77[] PROGMEM = "GPT4_CCMPA";
const char string_78[] PROGMEM = "GPT4_CCMPB";
const char string_79[] PROGMEM = "GPT4_CMPC";
const char string_7A[] PROGMEM = "GPT4_CMPD";
const char string_7B[] PROGMEM = "GPT4_CMPE";
const char string_7C[] PROGMEM = "GPT4_CMPF";
const char string_7D[] PROGMEM = "GPT4_OVF";
const char string_7E[] PROGMEM = "GPT4_UDF";
const char string_7F[] PROGMEM = "GPT5_CCMPA";
const char string_80[] PROGMEM = "GPT5_CCMPB";
const char string_81[] PROGMEM = "GPT5_CMPC";
const char string_82[] PROGMEM = "GPT5_CMPD";
const char string_83[] PROGMEM = "GPT5_CMPE";
const char string_84[] PROGMEM = "GPT5_CMPF";
const char string_85[] PROGMEM = "GPT5_OVF";
const char string_86[] PROGMEM = "GPT5_UDF";
const char string_87[] PROGMEM = "GPT6_CCMPA";
const char string_88[] PROGMEM = "GPT6_CCMPB";
const char string_89[] PROGMEM = "GPT6_CMPC";
const char string_8A[] PROGMEM = "GPT6_CMPD";
const char string_8B[] PROGMEM = "GPT6_CMPE";
const char string_8C[] PROGMEM = "GPT6_CMPF";
const char string_8D[] PROGMEM = "GPT6_OVF";
const char string_8E[] PROGMEM = "GPT6_UDF";
const char string_8F[] PROGMEM = "GPT7_CCMPA";
const char string_90[] PROGMEM = "GPT7_CCMPB";
const char string_91[] PROGMEM = "GPT7_CMPC";
const char string_92[] PROGMEM = "GPT7_CMPD";
const char string_93[] PROGMEM = "GPT7_CMPE";
const char string_94[] PROGMEM = "GPT7_CMPF";
const char string_95[] PROGMEM = "GPT7_OVF";
const char string_96[] PROGMEM = "GPT7_UDF";
const char string_97[] PROGMEM = "GPT_UVWEDGE";
const char string_98[] PROGMEM = "SCI0_RXI";
const char string_99[] PROGMEM = "SCI0_TXI";
const char string_9A[] PROGMEM = "SCI0_TEI";
const char string_9B[] PROGMEM = "SCI0_ERI";
const char string_9C[] PROGMEM = "SCI0_AM";
const char string_9D[] PROGMEM = "SCI0_RXI_OR_ERI";
const char string_9E[] PROGMEM = "SCI1_RXI";
const char string_9F[] PROGMEM = "SCI1_TXI";
const char string_A0[] PROGMEM = "SCI1_TEI";
const char string_A1[] PROGMEM = "SCI1_ERI";
const char string_A2[] PROGMEM = "SCI1_AM";
const char string_A3[] PROGMEM = "SCI2_RXI";
const char string_A4[] PROGMEM = "SCI2_TXI";
const char string_A5[] PROGMEM = "SCI2_TEI";
const char string_A6[] PROGMEM = "SCI2_ERI";
const char string_A7[] PROGMEM = "SCI2_AM";
const char string_A8[] PROGMEM = "SCI9_RXI";
const char string_A9[] PROGMEM = "SCI9_TXI";
const char string_AA[] PROGMEM = "SCI9_TEI";
const char string_AB[] PROGMEM = "SCI9_ERI";
const char string_AC[] PROGMEM = "SCI9_AM";
const char string_AD[] PROGMEM = "SPI0_SPRI";
const char string_AE[] PROGMEM = "SPI0_SPTI";
const char string_AF[] PROGMEM = "SPI0_SPII";
const char string_B0[] PROGMEM = "SPI0_SPEI";
const char string_B1[] PROGMEM = "SPI0_SPTEND";
const char string_B2[] PROGMEM = "SPI1_SPRI";
const char string_B3[] PROGMEM = "SPI1_SPTI";
const char string_B4[] PROGMEM = "SPI1_SPII";
const char string_B5[] PROGMEM = "SPI1_SPEI";
const char string_B6[] PROGMEM = "SPI1_SPTEND";
const char string_B7[] PROGMEM = "UNUSED";

const char *const string_table[] PROGMEM = {
string_00, string_01, string_02, string_03, string_04, string_05, string_06, string_07,
string_08, string_09, string_0A, string_0B, string_0C, string_0D, string_0E, string_0F,
string_10, string_11, string_12, string_13, string_14, string_15, string_16, string_17,
string_18, string_19, string_1A, string_1B, string_1C, string_1D, string_1E, string_1F,
string_20, string_21, string_22, string_23, string_24, string_25, string_26, string_27,
string_28, string_29, string_2A, string_2B, string_2C, string_2D, string_2E, string_2F,
string_30, string_31, string_32, string_33, string_34, string_35, string_36, string_37,
string_38, string_39, string_3A, string_3B, string_3C, string_3D, string_3E, string_3F,
string_40, string_41, string_42, string_43, string_44, string_45, string_46, string_47,
string_48, string_49, string_4A, string_4B, string_4C, string_4D, string_4E, string_4F,
string_50, string_51, string_52, string_53, string_54, string_55, string_56, string_57,
string_58, string_59, string_5A, string_5B, string_5C, string_5D, string_5E, string_5F,
string_60, string_61, string_62, string_63, string_64, string_65, string_66, string_67,
string_68, string_69, string_6A, string_6B, string_6C, string_6D, string_6E, string_6F,
string_70, string_71, string_72, string_73, string_74, string_75, string_76, string_77,
string_78, string_79, string_7A, string_7B, string_7C, string_7D, string_7E, string_7F,
string_80, string_81, string_82, string_83, string_84, string_85, string_86, string_87,
string_88, string_89, string_8A, string_8B, string_8C, string_8D, string_8E, string_8F,
string_90, string_91, string_92, string_93, string_94, string_95, string_96, string_97,
string_98, string_99, string_9A, string_9B, string_9C, string_9D, string_9E, string_9F,
string_A0, string_A1, string_A2, string_A3, string_A4, string_A5, string_A6, string_A7,
string_A8, string_A9, string_AA, string_AB, string_AC, string_AD, string_AE, string_AF,
string_B0, string_B1, string_B2, string_B3, string_B4, string_B5, string_B6, string_B7
};

char message_buffer[30];  // 

void print_icu_event_links(void)
  {
  unsigned int local_icu_val = 0;
  unsigned char icu_val_index = 0;

  for(icu_val_index = 0; icu_val_index < 32; icu_val_index++)
    {
    Serial.print(icu_val_index);  
    Serial.print(" - ");  
    local_icu_val = *((volatile unsigned int *)(ICUBASE + IELSR + (icu_val_index * 4)));            //
    Serial.print(local_icu_val, HEX);
    strcpy_P(message_buffer, (char *)pgm_read_word(&(string_table[local_icu_val])));  // 
    Serial.print(" - ");  
    Serial.println(message_buffer);
    if(local_icu_val == 0) break;      // Only print active allocations - these are always contigious from 0
    }
  }

