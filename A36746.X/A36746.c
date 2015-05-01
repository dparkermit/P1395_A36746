#include "A36746.h"
#include "FIRMWARE_VERSION.h"
#include "LTC265X.h"
#include "ETM_EEPROM.h"


_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);

#define THERMISTOR_LOOK_UP_TABLE_VALUES 472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,472,377,376,375,374,373,372,371,370,370,369,368,367,366,366,365,364,363,363,362,361,361,360,360,359,358,358,357,357,356,355,355,354,354,353,353,352,352,351,351,350,350,350,349,349,348,348,347,347,346,346,346,345,345,344,344,344,343,343,343,342,342,341,341,341,340,340,340,339,339,339,338,338,338,337,337,337,336,336,336,336,335,335,335,334,334,334,333,333,333,333,332,332,332,332,331,331,331,330,330,330,330,329,329,329,329,328,328,328,328,327,327,327,327,326,326,326,326,326,325,325,325,325,324,324,324,324,323,323,323,323,323,322,322,322,322,322,321,321,321,321,320,320,320,320,320,319,319,319,319,319,318,318,318,318,318,318,317,317,317,317,317,316,316,316,316,316,315,315,315,315,315,315,314,314,314,314,314,313,313,313,313,313,313,312,312,312,312,312,311,311,311,311,311,311,310,310,310,310,310,310,309,309,309,309,309,309,308,308,308,308,308,308,307,307,307,307,307,307,307,306,306,306,306,306,306,305,305,305,305,305,305,304,304,304,304,304,304,304,303,303,303,303,303,303,302,302,302,302,302,302,302,301,301,301,301,301,301,301,300,300,300,300,300,300,299,299,299,299,299,299,299,298,298,298,298,298,298,298,297,297,297,297,297,297,297,296,296,296,296,296,296,296,295,295,295,295,295,295,295,294,294,294,294,294,294,294,293,293,293,293,293,293,293,292,292,292,292,292,292,292,291,291,291,291,291,291,291,290,290,290,290,290,290,290,289,289,289,289,289,289,288,288,288,288,288,288,288,287,287,287,287,287,287,287,286,286,286,286,286,286,286,285,285,285,285,285,285,285,284,284,284,284,284,284,284,283,283,283,283,283,283,282,282,282,282,282,282,282,281,281,281,281,281,281,281,280,280,280,280,280,280,279,279,279,279,279,279,279,278,278,278,278,278,278,277,277,277,277,277,277,276,276,276,276,276,276,275,275,275,275,275,275,274,274,274,274,274,274,273,273,273,273,273,273,272,272,272,272,272,272,271,271,271,271,271,270,270,270,270,270,270,269,269,269,269,269,268,268,268,268,268,267,267,267,267,267,266,266,266,266,266,265,265,265,265,265,264,264,264,264,263,263,263,263,263,262,262,262,262,261,261,261,261,260,260,260,260,259,259,259,259,258,258,258,258,257,257,257,257,256,256,256,255,255,255,255,254,254,254,253,253,253,252,252,252,251,251,251,250,250,250,249,249,248,248,248,247,247,246,246,246,245,245,244,244,243,243,242,242,241,241,240,240,239,238,238,237,236,236,235,234,233,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,172,172,172,172,172


const unsigned int ThermistorLookupTable[630] = {THERMISTOR_LOOK_UP_TABLE_VALUES};

PWMReading PWM1_IC1_magnetron_flow;
PWMReading PWM2_IC2_linac_flow;
PWMReading PWM3_IC3_hv_tank_flow;

void InputCaptureSavePeriod(unsigned int latest_capture, PWMReading* ptr_pwm);


void InputCaptureWriteMaxPeriod(PWMReading* ptr_pwm);

unsigned int InputCaptureCalculateFrequency(PWMReading* ptr_pwm);

LTC265X U14_LTC2656;
CoolingGlobalStruct global_data_A36746;

void DoSF6Management(void);

#define DIGITAL_FILTER_TEMP_SWITCH_FAULT_COUNT    200        // 2 seconds

#define COOLING_INTERFACE_BOARD_TEST_TIME         100        // 1 second


#define MINIMUM_FLOW_MAGNETRON   4000
#define MINIMUM_FLOW_LINAC       6700
#define MINIMUM_FLOW_HV_TANK     8000


#define THERMISTOR_REFERENCE_DAC_SETTING       0xD055        // This will create 10V thermistor reference

#define STARTUP_LED_FLASH_TIME       400       // 4 Seconds


#define FLOW_METER_ML_PER_HZ      81
#define FLOW_METER_CONSTANT       841

#define PERIOD_MAX_FREQUENCY 70  // 558 Hz
#define FLOW_METER_MIN_FREQUENCY  15



#define SF6_STATE_TEST       10
#define SF6_STATE_CHARGING   20
#define SF6_STATE_DELAY      30

#define MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT  292   // 292 Deg K / 20 Deg C
#define MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT      3000  // 30 PSI   

#define SF6_MINIMUM_TARGET_PRESSURE              4000  // 40 PSI
#define SF6_MAXIMUM_TARGET_PRESSURE              4200  // 42 PSI

#define SF6_TIME_CHARGING                        500 // 5 Seconds
#define SF6_TIME_DELAY                           500 // 5 Seconds



//#define COOLANT_TRIP_TEMPERATURE        308 // This is in Deg K units
//#define CABINET_TRIP_TEMPERATURE        318 // This is in Deg K units

#define COOLANT_TRIP_THERMISTOR_VOLTAGE       3936  // This is the voltage (in millivolts) that indicates a temerperature of 36 Deg C.  If the thermistor voltage is less than this it should trip
#define CABINET_TRIP_THERMISTOR_VOLTAGE       3040  // This is the voltage (in millivolts) that indicates a temerperature of 46 Deg C.  If the thermistor voltage is less than this it should trip

#define TEMPERATURE_SENSOR_FIXED_SCALE         .15625

#define ANALOG_TEMPERATURE_TRIP_TIME    1000  // 10 Seconds



#define SF6_SENSOR_FIXED_SCALE                 .19629   // DPARKER NEED TO TEST
#define SF6_SENSOR_FIXED_OFFSET                -12736    // Calculated 4mA Offset
#define SF6_UNDER_PRESSURE_TRIP                3700     // 37 PSI DPARKER NEED TO TEST
#define SF6_PRESSURE_TRIP_TIME                 1000     // 10 seconds




void DoStateMachine(void);
void DoA36746(void);
void InitializeA36746(void);
void FlashLEDs(void);



int main(void) {
  
  global_data_A36746.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A36746.control_state) {
    
  case STATE_STARTUP:
    InitializeA36746();
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    global_data_A36746.startup_counter = 0;
    while (global_data_A36746.control_state == STATE_STARTUP) {
      DoA36746();
      FlashLEDs();
      if (global_data_A36746.startup_counter >= STARTUP_LED_FLASH_TIME) {
	global_data_A36746.control_state = STATE_NOT_READY;	
      }
    }
    break;

    
  case STATE_NOT_READY:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    _CONTROL_NOT_READY = 1;
    while (global_data_A36746.control_state == STATE_NOT_READY) {
      DoA36746();
      if (_SYNC_CONTROL_RESET_ENABLE) {
	global_data_A36746.control_state = STATE_TESTING;
      }
    }
    break;


  case STATE_TESTING:
    _CONTROL_NOT_READY = 1;
    global_data_A36746.test_timer = 0;
    while (global_data_A36746.control_state == STATE_TESTING) {
      DoA36746();
      if (global_data_A36746.test_timer >= COOLING_INTERFACE_BOARD_TEST_TIME) {
	global_data_A36746.control_state = STATE_READY;
      }
      if (global_data_A36746.fault_active) {
	global_data_A36746.control_state = STATE_NOT_READY;
      }
    }
    break;


  case STATE_READY:
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36746.control_state == STATE_READY) {
      DoA36746();
      if (global_data_A36746.fault_active) {
	global_data_A36746.control_state = STATE_NOT_READY;
      }

    }
    break;

  default:
    global_data_A36746.control_state = STATE_STARTUP;
    break;
  }
}




void DoA36746(void) {
  unsigned int timer_reading;
  unsigned int frequency;
  unsigned int coolant_thermistor_index;
  unsigned int cabinet_thermistor_index;
  ETMCanSlaveDoCan();


  // Update the period register for PWM1_IC1_magnetron_flow
  while(IC1CONbits.ICBNE) {
    timer_reading = IC1BUF;
    InputCaptureSavePeriod(timer_reading, &PWM1_IC1_magnetron_flow);
  }

  // Update the period register for PWM2_IC2_linac_flow
  while(IC2CONbits.ICBNE) {
    timer_reading = IC2BUF;
    InputCaptureSavePeriod(timer_reading, &PWM2_IC2_linac_flow);
  }
  

  // Update the period register for PWM3_IC3_hv_tank_flow
  while(IC3CONbits.ICBNE) {
    timer_reading = IC3BUF;
    InputCaptureSavePeriod(timer_reading, &PWM3_IC3_hv_tank_flow);
  }


  if (_T1IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms)
    _T1IF = 0;

    DoSF6Management();
    
    local_debug_data.debug_3 = PWM1_IC1_magnetron_flow.period_array[PWM1_IC1_magnetron_flow.current_index] + PWM1_IC1_magnetron_flow.period_array[(PWM1_IC1_magnetron_flow.current_index - 1) & 0xFF];
    local_debug_data.debug_4 = PWM2_IC2_linac_flow.period_array[PWM2_IC2_linac_flow.current_index] + PWM2_IC2_linac_flow.period_array[(PWM2_IC2_linac_flow.current_index - 1) & 0xFF];
    local_debug_data.debug_5 = PWM3_IC3_hv_tank_flow.period_array[PWM3_IC3_hv_tank_flow.current_index] + PWM3_IC3_hv_tank_flow.period_array[(PWM3_IC3_hv_tank_flow.current_index - 1) & 0xFF];

    local_debug_data.debug_6 = global_data_A36746.control_state;
    local_debug_data.debug_7 = global_data_A36746.analog_input_coolant_temperature.reading_scaled_and_calibrated;
    local_debug_data.debug_8 = global_data_A36746.cabinet_temperature_kelvin;
    local_debug_data.debug_9 = global_data_A36746.flow_magnetron;
    local_debug_data.debug_A = global_data_A36746.flow_hv_tank;
    local_debug_data.debug_B = global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated;
    local_debug_data.debug_C = global_data_A36746.SF6_state;
    local_debug_data.debug_D = global_data_A36746.SF6_pulses_available;
    local_debug_data.debug_E = global_data_A36746.SF6_bottle_pulses_remaining;
    local_debug_data.debug_F = global_data_A36746.SF6_low_pressure_override_counter;
    
    




    frequency = InputCaptureCalculateFrequency(&PWM1_IC1_magnetron_flow);      
    if (frequency < FLOW_METER_MIN_FREQUENCY) {
      global_data_A36746.flow_magnetron = 0;
    } else {
      global_data_A36746.flow_magnetron = FLOW_METER_ML_PER_HZ*frequency + FLOW_METER_CONSTANT;
    }
    local_debug_data.debug_0 = frequency;
      

    // Calculate flow for PWM2_IC2_linac_flow
    frequency = InputCaptureCalculateFrequency(&PWM2_IC2_linac_flow);      
    if (frequency < FLOW_METER_MIN_FREQUENCY) {
      global_data_A36746.flow_linac = 0;
    } else {
      global_data_A36746.flow_linac = FLOW_METER_ML_PER_HZ*frequency + FLOW_METER_CONSTANT;
    }
    local_debug_data.debug_1 = frequency;


    // Calculate flow for PWM3_IC3_hv_tank_flow
    frequency = InputCaptureCalculateFrequency(&PWM3_IC3_hv_tank_flow);      
    if (frequency < FLOW_METER_MIN_FREQUENCY) {
      global_data_A36746.flow_hv_tank = 0;
    } else {
      global_data_A36746.flow_hv_tank = FLOW_METER_ML_PER_HZ*frequency + FLOW_METER_CONSTANT;
    }
    local_debug_data.debug_2 = frequency;

    
    // Update the flow meter, low speed counters.
    if (PWM1_IC1_magnetron_flow.counter_10ms_loop < 200) {
      PWM1_IC1_magnetron_flow.counter_10ms_loop++;
    } else {
      // There have been no pulses for 2 seconds
      // Time to start writing zero flow values to the period register
      InputCaptureWriteMaxPeriod(&PWM1_IC1_magnetron_flow);
    }
    if (PWM2_IC2_linac_flow.counter_10ms_loop < 200) {
      PWM2_IC2_linac_flow.counter_10ms_loop++;
    } else {
      // There have been no pulses for 2 seconds
      // Time to start writing zero flow values to the period register
      InputCaptureWriteMaxPeriod(&PWM2_IC2_linac_flow);
    }
    if (PWM3_IC3_hv_tank_flow.counter_10ms_loop < 200) {
      PWM3_IC3_hv_tank_flow.counter_10ms_loop++;      
    } else {
      // There have been no pulses for 2 seconds
      // Time to start writing zero flow values to the period register
      InputCaptureWriteMaxPeriod(&PWM3_IC3_hv_tank_flow);
    }



   // If the system is faulted or inhibited set the red LED
    if (_CONTROL_NOT_READY) {
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }


    if (global_data_A36746.control_state == STATE_TESTING) {
      global_data_A36746.test_timer++;
    }
    
    if (global_data_A36746.control_state == STATE_STARTUP) {
      global_data_A36746.startup_counter++;
    }
  

    // Do Math on ADC inputs
    // Scale the ADC readings to engineering units
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_coolant_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_cabinet_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36746.analog_input_SF6_pressure);

    // Convert the temperature reading voltage into temperature
    coolant_thermistor_index = (global_data_A36746.analog_input_coolant_temperature.reading_scaled_and_calibrated >>4);
    if (coolant_thermistor_index > 625) {
      coolant_thermistor_index = 625;
    }
    global_data_A36746.coolant_temperature_kelvin = ThermistorLookupTable[coolant_thermistor_index];

    cabinet_thermistor_index = (global_data_A36746.analog_input_cabinet_temperature.reading_scaled_and_calibrated >>4);
    if (cabinet_thermistor_index > 625) {
      cabinet_thermistor_index = 625;
    }
    global_data_A36746.cabinet_temperature_kelvin = ThermistorLookupTable[cabinet_thermistor_index];
    




    // ------------ CHECK FOR FAULTS ----------- //
    global_data_A36746.fault_active = 0;
    
    if (_CONTROL_CAN_COM_LOSS) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_CAN_COMMUNICATION_LATCHED = 0;
    }

    if (global_data_A36746.flow_magnetron < MINIMUM_FLOW_MAGNETRON) {
      _FAULT_MAGNETRON_COOLANT_FLOW = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_MAGNETRON_COOLANT_FLOW = 0;
    }

    if (global_data_A36746.flow_linac < MINIMUM_FLOW_LINAC) {
      _FAULT_LINAC_COOLANT_FLOW = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_LINAC_COOLANT_FLOW = 0;
    }
    
    if (global_data_A36746.flow_hv_tank < MINIMUM_FLOW_HV_TANK) {
      _FAULT_HV_TANK_COOLANT_FLOW = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_HV_TANK_COOLANT_FLOW = 0;
    }

    if (PIN_DIGITAL_INPUT_1_CABINET_TEMP_SWITCH == ILL_TEMP_SWITCH_OVER_TEMP) {
      global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch++;
    } else {
      if (global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch) {
	global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch--;
      }
    }
    if (global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch >= DIGITAL_FILTER_TEMP_SWITCH_FAULT_COUNT) {
      global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch = DIGITAL_FILTER_TEMP_SWITCH_FAULT_COUNT;
      _FAULT_CABINET_TEMP_SWITCH = 1;
      global_data_A36746.fault_active = 1;
    } else if ((global_data_A36746.filter_pin_digitial_input_1_cabinet_temp_switch == 0) && (_SYNC_CONTROL_RESET_ENABLE)) {
      _FAULT_CABINET_TEMP_SWITCH = 0;
    }


    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_cabinet_temperature)) {
      // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
      _FAULT_CABINET_OVER_TEMP = 1;
      global_data_A36746.fault_active = 1; 
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_CABINET_OVER_TEMP = 0;
    }
    

    /*
    if (PIN_DIGITAL_INPUT_2_COOLANT_TEMP_SWITCH == ILL_TEMP_SWITCH_OVER_TEMP) {
      global_data_A36746.filter_pin_digitial_input_2_coolant_temp_switch++;
      if (global_data_A36746.filter_pin_digitial_input_2_coolant_temp_switch >= DIGITAL_FILTER_TEMP_SWITCH_FAULT_COUNT) {
	global_data_A36746.filter_pin_digitial_input_2_coolant_temp_switch = DIGITAL_FILTER_TEMP_SWITCH_FAULT_COUNT;
	_FAULT_COOLANT_TEMP_SWITCH = 1;
	global_data_A36746.fault_active = 1;
      }
    } else {
      if (global_data_A36746.filter_pin_digitial_input_2_coolant_temp_switch) {
	global_data_A36746.filter_pin_digitial_input_2_coolant_temp_switch--;
      }
    }
    */

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_coolant_temperature)) {
      // We are using the under absolute function because as temperature goes up, resistance (and voltage) go down 
      _FAULT_COOLANT_OVER_TEMP = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_COOLANT_OVER_TEMP = 0;
    }

      
    if (ETMAnalogCheckUnderAbsolute(&global_data_A36746.analog_input_SF6_pressure)) {
      _FAULT_SF6_UNDER_PRESSURE = 1;
      global_data_A36746.fault_active = 1;
    } else if (_SYNC_CONTROL_RESET_ENABLE) {
      _FAULT_SF6_UNDER_PRESSURE = 0;
    }

    // ------------ END CHECK FOR FAULTS ----------- //



    // Write Dac Output
    WriteLTC265X(&U14_LTC2656, LTC265X_WRITE_AND_UPDATE_DAC_G, THERMISTOR_REFERENCE_DAC_SETTING);
  }
}



void InitializeA36746(void) {

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;

  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  

  // Initialize all I/O Registers
  TRISA = A36746_TRISA_VALUE;
  TRISB = A36746_TRISB_VALUE;
  TRISC = A36746_TRISC_VALUE;
  TRISD = A36746_TRISD_VALUE;
  TRISF = A36746_TRISF_VALUE;
  TRISG = A36746_TRISG_VALUE;

  // Initialize TMR2
  PR2 = PR2_VALUE_MAX;
  TMR2 = 0;
  T2CON = T2CON_VALUE;


  // Initialize TMR5
  PR1   = PR1_VALUE_10_MILLISECONDS;
  TMR1  = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;


  // Initialize internal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_OPERATE;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING_OPERATE; //0b0000111100000000;//ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
  _ADIE = 1;
  _ADON = 1;


  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2656, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize the Can module
  ETMCanSlaveInitialize();



  // Initialize the Analog Input & Output Scaling


  ETMAnalogInitializeInput(&global_data_A36746.analog_input_coolant_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_3,
			   NO_OVER_TRIP,
			   COOLANT_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   ANALOG_TEMPERATURE_TRIP_TIME);
  
  ETMAnalogInitializeInput(&global_data_A36746.analog_input_cabinet_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(TEMPERATURE_SENSOR_FIXED_SCALE),
			   OFFSET_ZERO,
			   ANALOG_INPUT_4,
			   NO_OVER_TRIP,
			   CABINET_TRIP_THERMISTOR_VOLTAGE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   ANALOG_TEMPERATURE_TRIP_TIME);


  ETMAnalogInitializeInput(&global_data_A36746.analog_input_SF6_pressure,
			   MACRO_DEC_TO_SCALE_FACTOR_16(SF6_SENSOR_FIXED_SCALE),
			   SF6_SENSOR_FIXED_OFFSET,
			   ANALOG_INPUT_6,
			   NO_OVER_TRIP,
			   SF6_UNDER_PRESSURE_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   SF6_PRESSURE_TRIP_TIME);


  _CONTROL_SELF_CHECK_ERROR = 0;


  global_data_A36746.SF6_state = SF6_STATE_DELAY;
  global_data_A36746.SF6_fill_threshold = SF6_MINIMUM_TARGET_PRESSURE;
  global_data_A36746.SF6_low_pressure_override_counter = 0;
  global_data_A36746.SF6_state_delay_counter = 0;

  ETMEEPromReadPage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A36746.SF6_pulses_available);
  // This reads SF6_pulses_available and SF6_bottle_pulses_remaining from the external EEPROM with a single I2C command
  if (global_data_A36746.SF6_pulses_available > 25) {
    global_data_A36746.SF6_pulses_available = 25;
  }
  if (global_data_A36746.SF6_bottle_pulses_remaining > 700) {
    global_data_A36746.SF6_bottle_pulses_remaining = 700;
  }

#define PWM1_INITITAL_PERIOD 195 // 200 Hz
#define PWM2_INITITAL_PERIOD 391 // 100 Hz
#define PWM3_INITITAL_PERIOD 1302 // 30 Hz 

  PWM1_IC1_magnetron_flow.period_array[PWM1_IC1_magnetron_flow.current_index] = PWM1_INITITAL_PERIOD;
  PWM2_IC2_linac_flow.period_array[PWM2_IC2_linac_flow.current_index] = PWM2_INITITAL_PERIOD;
  PWM3_IC3_hv_tank_flow.period_array[PWM3_IC3_hv_tank_flow.current_index] = PWM3_INITITAL_PERIOD;

  
#define ICXCON_VALUE  (IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_EDGE)

  IC1CON = ICXCON_VALUE;
  IC2CON = ICXCON_VALUE;
  IC3CON = ICXCON_VALUE;
}



void FlashLEDs(void) {
  switch (((global_data_A36746.startup_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = OLL_LED_ON;
    break;
  }
}

#define PWM_MAX_PERIOD 19531 // This is a period of one half second (since we trigger and rising and falling edge a period is actually two of these - one second)


void InputCaptureSavePeriod(unsigned int latest_capture, PWMReading* ptr_pwm) {
  ptr_pwm->current_index++;
  ptr_pwm->current_index &= 0xFF;
  if (ptr_pwm->counter_10ms_loop >= 100) {
    // It has been approx 1 second (or more) between readings
    ptr_pwm->period_array[ptr_pwm->current_index] = PWM_MAX_PERIOD; 
  } else {
    ptr_pwm->period_array[ptr_pwm->current_index] = latest_capture - ptr_pwm->previous_timer_reading;
  }
  ptr_pwm->previous_timer_reading = latest_capture;
  ptr_pwm->counter_10ms_loop = 0;
}


void InputCaptureWriteMaxPeriod(PWMReading* ptr_pwm) {
  ptr_pwm->current_index++;
  ptr_pwm->current_index &= 0xFF;
  ptr_pwm->period_array[ptr_pwm->current_index] = PWM_MAX_PERIOD; 
}

unsigned int InputCaptureCalculateFrequency(PWMReading* ptr_pwm) {
  unsigned int temp;
  temp = ptr_pwm->period_array[ptr_pwm->current_index];
  temp += ptr_pwm->period_array[((ptr_pwm->current_index-1) & 0xFF)];
  if (temp <= PERIOD_MAX_FREQUENCY) {
    temp = PERIOD_MAX_FREQUENCY;
  }
  temp = 39062 / temp;
  return temp;
}




void DoSF6Management(void) {
  switch (global_data_A36746.SF6_state) {
    
  case SF6_STATE_TEST:
    _STATUS_SF6_FILLING = 0;

    if (!_SYNC_CONTROL_RESET_ENABLE) {
      break;
    }

    if (global_data_A36746.coolant_temperature_kelvin < MINIMUM_COOLANT_TEMP_FOR_SF6_MANAGEMENT) {
      _STATUS_SF6_COOLANT_TOO_LOW = 1;
      break;
    }
    _STATUS_SF6_COOLANT_TOO_LOW = 0;

    if ((global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated < MINIMUM_PRESSURE_FOR_SF6_MANAGEMENT) && (global_data_A36746.SF6_low_pressure_override_counter == 0)) {
      _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 1;
      break;
    }
    _STATUS_SF6_PRESSURE_TO_LOW_TO_MANAGE = 0;

    if (global_data_A36746.analog_input_SF6_pressure.reading_scaled_and_calibrated > global_data_A36746.SF6_fill_threshold) {
      _STATUS_SF6_FILL_REQUIRED = 0;
      break;
    }
    _STATUS_SF6_FILL_REQUIRED = 1;

    if (!global_data_A36746.SF6_pulses_available) {
      _STATUS_SF6_NO_PULSES_AVAILABLE = 1;
      break;
    }
    _STATUS_SF6_NO_PULSES_AVAILABLE = 0;

    

    if (global_data_A36746.SF6_pulses_available) {
      global_data_A36746.SF6_pulses_available--;
    }

    if (global_data_A36746.SF6_low_pressure_override_counter) {
      global_data_A36746.SF6_low_pressure_override_counter--;
    }

    if (global_data_A36746.SF6_bottle_pulses_remaining) {
      global_data_A36746.SF6_bottle_pulses_remaining--;
    }
    

    ETMEEPromWritePage(ETM_EEPROM_PAGE_COOLING_INTERFACE, 2, &global_data_A36746.SF6_pulses_available);
    // This writes SF6_pulses_available and SF6_bottle_pulses_remaining to the external EEPROM with a single I2C command (and single FLASH write cycle)

    _STATUS_SF6_FILL_REQUIRED = 1;
    global_data_A36746.SF6_fill_threshold = SF6_MAXIMUM_TARGET_PRESSURE;
    global_data_A36746.SF6_state_charging_counter = 0;
    global_data_A36746.SF6_state = SF6_STATE_CHARGING;

    break;


  case SF6_STATE_CHARGING:
    global_data_A36746.SF6_state_charging_counter++;
    PIN_RELAY_OUT_SF6_SOLENOID = OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 1;
    _STATUS_SF6_FILLING = 1;
    if (global_data_A36746.SF6_state_charging_counter >= SF6_TIME_CHARGING) {
      global_data_A36746.SF6_state_delay_counter = 0;
      global_data_A36746.SF6_state = SF6_STATE_DELAY;
    }
    break;

  case SF6_STATE_DELAY:
    PIN_RELAY_OUT_SF6_SOLENOID = !OLL_CLOSE_SOLENOID;
    _STATUS_SF6_SOLENOID_RELAY_CLOSED = 0;
    _STATUS_SF6_FILLING = 1;
    global_data_A36746.SF6_state_delay_counter++;
    if (global_data_A36746.SF6_state_delay_counter >= SF6_TIME_DELAY) {
      _STATUS_SF6_FILLING = 0;
      global_data_A36746.SF6_state = SF6_STATE_TEST;

    }
    break;

  default:
    global_data_A36746.SF6_state = SF6_STATE_DELAY;
    break;
  }
}




void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator += ADCBUF0;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator += ADCBUF1;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator        += ADCBUF2;
    // Flow 1 Unused in ADCBUF3;
    // Flow 2 Unused in ADCBUF4;
    // Flow 3 Unused in ADCBUF5;
    // Flow 4 Unused in ADCBUF6;
    // Flow 5 Unused in ADCBUF7;
  } else {
    // read ADCBUF 8-15
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator += ADCBUF8;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator += ADCBUF9;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator        += ADCBUFA;
    // Flow 1 Unused in ADCBUFB;
    // Flow 2 Unused in ADCBUFC;
    // Flow 3 Unused in ADCBUFD;
    // Flow 4 Unused in ADCBUFE;
    // Flow 5 Unused in ADCBUFF;
  }
  
  global_data_A36746.accumulator_counter += 1;
  
  if (global_data_A36746.accumulator_counter >= 128) {
    global_data_A36746.accumulator_counter = 0;    
    
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_coolant_temperature.filtered_adc_reading = global_data_A36746.analog_input_coolant_temperature.adc_accumulator;
    global_data_A36746.analog_input_coolant_temperature.adc_accumulator = 0;

    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_cabinet_temperature.filtered_adc_reading = global_data_A36746.analog_input_cabinet_temperature.adc_accumulator;
    global_data_A36746.analog_input_cabinet_temperature.adc_accumulator = 0;

    global_data_A36746.analog_input_SF6_pressure.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
    global_data_A36746.analog_input_SF6_pressure.filtered_adc_reading = global_data_A36746.analog_input_SF6_pressure.adc_accumulator;
    global_data_A36746.analog_input_SF6_pressure.adc_accumulator = 0;

  }
}




void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}
