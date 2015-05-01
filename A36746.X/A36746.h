#ifndef __A36746_H
#define __A36746_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <incap.h>

#include "P1395_CAN_SLAVE.h"
#include "ETM_ANALOG.h"
#include "P1395_MODULE_CONFIG.h"


/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module


  Timer1 - Used for 10msTicToc
  Timer2 - Used for Input Capture module


  ADC Module - See Below For Specifics

*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  Pins that are overidden by a hardware module and should be left as inputs during port configuration

  RA9  ADC VREF-
  RA10 ADC VREF+

  RB0 PROGRAM
  RB1 PROGRAM
  RB3  - Analog Input
  RB4  - Analog Input
  RB5  - Analog Input
  RB6  - Analog Input
  RB7  - Analog Input
  RB8  - Analog Input
  RB9  - Analog Input
  RB10 - Analog Input
  RB11 - Analog Input
  RB12 - Analog Input
  RB13 - Analog Input
  RB14 - Analog Input
  RB15 - Analog Input

  RC1  (DAC LDAC) - configured by other software modules and should be left as inputs during port configuration 

  RD8  - PWM 1 - IC1 - Magnetron Flow
  RD9  - PWM 2 - IC2 - Linac Flow
  RD10 - PWM 3 - IC3 - HV Tank Flow
  RD11 - PWM 4 - IC4 - Unused
  RD12 - PWM 5 - IC5 - Unused
  RD13 - PWM 6 - IC6 - Unused 
  RD14 - Digital in 1
  RD15 - Digital in 2

  RF0 CAN 1
  RF1 CAN 1

  RG2 I2C
  RG3 I2C
  RG6 SPI 1
  RG7 SPI 1
  RG8 SPI 1
  RG14 - Reset Detect
  RG15 (DAC CS/LD) - configured by other software modules and should be left as inputs during port configuration 



  

*/

//   ------------------  Digital Output Pins ---------------
/*
  
  RD11 - Lamdba Voltage Select
  RD0 - Lambda Inhibit (This is also Output Compare 1 - If we want to use that module to generate Inhibit signal)
  RD1 - Lambda Enable


  RA7 - LED Operational
  RB8 - Test Point E
  RB9 - Test Point F
  RF4 - Test Point A
  RF5 - Test Point B
  RG0 - Test Point C
  RG1 - Test Point D
  RG12 - LED A RED
  RG13 - LED B GREEN   // This is reserved for the can module to indicate can activity
  
*/


#define A36746_TRISA_VALUE 0b0000011000000000
#define A36746_TRISB_VALUE 0b1111111111111011
#define A36746_TRISC_VALUE 0b0000000000000010 
#define A36746_TRISD_VALUE 0b1111111100000000 
#define A36746_TRISF_VALUE 0b0000000000000011 
#define A36746_TRISG_VALUE 0b1100000111001100



// -------- Digital Input Pins ----------//
#define PIN_FLOW_PWM_1_MAGNETRON              _RD8      // This is input capture 1 
#define PIN_FLOW_PWM_2_LINAC                  _RD9      // This is input capture 2
#define PIN_FLOW_PWM_3_HV_TANK                _RD10     // This is input capture 3
#define PIN_FLOW_PWM_4_UNUSED                 _RD11     // This is input capture 4
#define PIN_FLOW_PWM_5_UNUSED                 _RD12     // This is input capture 5
#define PIN_FLOW_PWM_6_UNUSED                 _RD13     // This is input capture 6

#define PIN_RESET_DETECT                      _RG14

#define PIN_DIGITAL_INPUT_1_CABINET_TEMP_SWITCH  _RD14    // This is the air temperature
#define PIN_DIGITAL_INPUT_2_COOLANT_TEMP_SWITCH  _RD15    // This is the coolant temperature

#define ILL_TEMP_SWITCH_OVER_TEMP             0


// ------- Digital Output Pins ---------//
#define PIN_RELAY_OUT_SF6_SOLENOID            _LATD7    // This is output compare 8
#define PIN_DIGITAL_OUT_1_UNUSED              _LATD6
#define PIN_DIGITAL_OUT_2_UNUSED              _LATD5

#define OLL_CLOSE_SOLENOID                    1

#define PIN_LED_OPERATIONAL_GREEN             _LATA7
#define PIN_LED_A_RED                         _LATG12
#define PIN_LED_B_GREEN                       _LATG13  // This is is configured by the CAN module to flash on CAN Bus activity

#define OLL_LED_ON                            0


#define PIN_OUT_TP_A                          _LATF6
#define PIN_OUT_TP_B                          _LATF7
#define PIN_OUT_TP_C                          _LATF8



// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN3  - Analog 1 Coolant Temperature (Sampled)
   AN4  - Analog 2 Air Temperature     (Sampled)
   AN5  - Analog 3 Unused              (Skipped)
   AN6  - Analog 4 SF6 Pressure        (Sampled)
   
   AN7  - Flow 1 Unused                (Sampled)
   AN8  - Flow 2 Unused                (Sampled)
   AN9  - Flow 3 Unused                (Sampled)
   AN10 - Flow 4 Unused                (Sampled)
   AN11 - FLow 5 Unused                (Sampled)
   AN12 - Flow 6 Unused                (Skipped)
   AN13 - -15V Mon                     (Skipped) (Startup)
   AN14 - 5V Mon                       (Skipped) (Startup)
   AN15 - +15V Mon                     (Skipped) (Startup)
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  8 Samples per Interrupt, use alternating buffers
  Conversion rate of 111KHz (13.875 Khz per Channel), 138 Samples per 10mS interrupt
  Scan Through Selected Inputs (8 selected at any point in time)

  // DPARKER it may be easier to read all the inputs all the time instead of reconfiguring the ADC, we don't need the high speed reading

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA & ENABLE_AN10_ANA & ENABLE_AN11_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)

#define ADCSSL_SETTING_OPERATE  (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN5 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)
#define ADCON3_SETTING_OPERATE  (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)

#define ADCSSL_SETTING_STARTUP  (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 &  SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12)
#define ADCON3_SETTING_STARTUP  (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)


/* 
   TMR1 Configuration
   Timer1 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T1CON_VALUE                    (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define PR1_PERIOD_US                  10000   // 10mS
#define PR1_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR1_PERIOD_US/8)


/* 
   TMR2 Configuration
   Timer2 - PWM frequency Capture
   Period should be set to 10mS
   Maximum Period = 2^16 * 256 / Fcy = 1.6777216  seconds with 10Mhz Clock


*/

#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_SOURCE_INT)
#define PR2_VALUE_MAX                  0xFFFF





// -------------------- A36746 STATUS BIT CONFIGURATION ------------------------ //
#define _STATUS_SF6_SOLENOID_RELAY_CLOSED               _STATUS_0
#define _STATUS_SF6_COOLANT_TOO_LOW                     _STATUS_1
#define _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE           _STATUS_2
#define _STATUS_SF6_FILL_REQUIRED                       _STATUS_3
#define _STATUS_SF6_NO_PULSES_AVAILABLE                 _STATUS_4
#define _STATUS_SF6_FILLING                             _STATUS_5

// -------------------- A36746 FAULTS/WARNINGS CONFIGURATION-------------------- //
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_0
#define _FAULT_MAGNETRON_COOLANT_FLOW                   _FAULT_1
#define _FAULT_HVPS_COOLANT_FLOW                        _FAULT_2  // unused
#define _FAULT_CIRCULATOR_COOLANT_FLOW                  _FAULT_3
#define _FAULT_LINAC_COOLANT_FLOW                       _FAULT_4  // unused
#define _FAULT_HV_TANK_COOLANT_FLOW                     _FAULT_5
#define _FAULT_CABINET_TEMP_SWITCH                      _FAULT_6
#define _FAULT_CABINET_OVER_TEMP                        _FAULT_7
#define _FAULT_COOLANT_TEMP_SWITCH                      _FAULT_8
#define _FAULT_COOLANT_OVER_TEMP                        _FAULT_9
#define _FAULT_LINAC_OVER_TEMP                          _FAULT_A  // unused
#define _FAULT_SF6_PRESSURE_SWITCH                      _FAULT_B  // unused
#define _FAULT_SF6_UNDER_PRESSURE                       _FAULT_C
#define _FAULT_SENSOR_6_FLOW                            _FAULT_D  // unused

typedef struct {
  unsigned int period_array[256];
  unsigned int current_index;
  unsigned int previous_timer_reading;
  unsigned int counter_10ms_loop;
  
} PWMReading;



typedef struct {
  AnalogInput analog_input_coolant_temperature;       // 1mV per LSB
  AnalogInput analog_input_cabinet_temperature;       // 1mV per LSB
  AnalogInput analog_input_SF6_pressure;              // .01 PSI per LSB
  AnalogInput analog_input_linac_temperature;         // .01 Def K per LSB -  UNUSED in this system
  unsigned int accumulator_counter;

  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_15v_mon;                   // 1mV per LSB
  AnalogInput analog_input_neg_15v_mon;               // 1mV per LSB

  AnalogOutput analog_output_thermistor_ref;          // 1mV per LSB

  unsigned int control_state;
  unsigned int startup_counter;
  unsigned int test_timer;
  unsigned int fault_active;

  unsigned int filter_pin_digitial_input_1_cabinet_temp_switch;
  unsigned int filter_pin_digitial_input_2_coolant_temp_switch;

  unsigned int coolant_temperature_kelvin;        // 1 Deg K per LSB
  unsigned int cabinet_temperature_kelvin;        // 1 Deg K per LSB

  unsigned int flow_magnetron;
  unsigned int flow_linac;
  unsigned int flow_hv_tank;
  unsigned int flow_hvps;       // unused for now
  unsigned int flow_circulator; // unused for now
  unsigned int flow_spare;      // unused for now

  unsigned int SF6_state;
  unsigned int SF6_fill_threshold;
  unsigned int SF6_low_pressure_override_counter;
  unsigned int SF6_state_charging_counter;
  unsigned int SF6_state_delay_counter;

  // THESE TWO VARIABLES MUST BE ADJACENT TO EACH OTHER IN MEMORY BECAUSE THEY ARE ACCESSED AS A PAIR WHEN READING/WRITING THE EEPROM
  unsigned int SF6_pulses_available;           // This is how many pulses can be sent out without ECB intervention
  unsigned int SF6_bottle_pulses_remaining;    // This is how many pulses are remainging in the bottle
  


  //unsigned int SF6_pulse_counter;
  //unsigned int SF6_bottle_counter;



} CoolingGlobalStruct;

extern CoolingGlobalStruct global_data_A36746;


// State Definitions
#define STATE_STARTUP                10
#define STATE_NOT_READY              20
#define STATE_TESTING                30
#define STATE_READY                  40

#define ETM_EEPROM_PAGE_COOLING_INTERFACE      0x70





#endif
