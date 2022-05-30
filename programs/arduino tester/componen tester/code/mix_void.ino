//******************************************************************
// output of flux voltage for 1-2 diodes in row 2
// bcdnum = Numbers of both Diodes:
// higher 4 Bit  number of first Diode
// lower 4 Bit  number of second Diode (Structure diodes[nn])
// if number >= 3  no output is done
void UfOutput(uint8_t bcdnum) {
  lcd_line2();        // 2 row
  lcd_fix_string(Uf_str);   // "Uf="
  mVOutput(bcdnum >> 4);
  mVOutput(bcdnum & 0x0f);
}

void mVOutput(uint8_t nn) {
  if (nn < 3) {
    // Output in mV units
    DisplayValue(diodes[nn].Voltage, -3, 'V', 3);
    lcd_space();
  }
}

void RvalOut(uint8_t ii) {
  // output of resistor value

#if FLASHEND > 0x1fff
  uint16_t rr;
  if ((resis[ii].rx < 100) && (resis[0].lx == 0)) {
    rr = GetESR(resis[ii].ra, resis[ii].rb);
    DisplayValue(rr, -2, LCD_CHAR_OMEGA, 3);
  } else {
    DisplayValue(resis[ii].rx, -1, LCD_CHAR_OMEGA, 4);
  }
#else
  DisplayValue(resis[ii].rx, -1, LCD_CHAR_OMEGA, 4);
#endif

  lcd_space();
}

//******************************************************************

void ChargePin10ms(uint8_t PinToCharge, uint8_t ChargeDirection) {
  // Load the specified pin to the specified direction with 680 Ohm for 10ms.
  // Will be used by discharge of MOSFET Gates or to load big capacities.
  // Parameters:
  // PinToCharge: specifies the pin as mask for R-Port
  // ChargeDirection: 0 = switch to GND (N-Kanal-FET), 1= switch to VCC(P-Kanal-FET)

  if (ChargeDirection & 1) {
    R_PORT |= PinToCharge;  // R_PORT to 1 (VCC)
  } else {
    R_PORT &= ~PinToCharge;   // or 0 (GND)
  }

  R_DDR |= PinToCharge;     // switch Pin to output, across R to GND or VCC
  wait_about10ms();     // wait about 10ms
  // switch back Input, no current
  R_DDR &= ~PinToCharge;    // switch back to input
  R_PORT &= ~PinToCharge;   // no Pull up
}


// first discharge any charge of capacitors
void EntladePins() {
  uint8_t adc_gnd;    // Mask of ADC-outputs, which can be directly connected to GND
  unsigned int adcmv[3];  // voltages of 3 Pins in mV
  unsigned int clr_cnt;   // Clear Counter
  uint8_t lop_cnt;    // loop counter

  // max. time of discharge in ms  (10000/20) == 10s
#define MAX_ENTLADE_ZEIT  (10000/20)

  for (lop_cnt = 0; lop_cnt < 10; lop_cnt++) {
    adc_gnd = TXD_MSK;    // put all ADC to Input
    ADC_DDR = adc_gnd;
    ADC_PORT = TXD_VAL;   // ADC-outputs auf 0
    R_PORT = 0;     // R-outputs auf 0
    R_DDR = (2 << (TP3 * 2)) | (2 << (TP2 * 2)) | (2 << (TP1 * 2)); // R_H for all Pins to GND

    adcmv[0] = W5msReadADC(TP1);  // which voltage has Pin 1?
    adcmv[1] = ReadADC(TP2);    // which voltage has Pin 2?
    adcmv[2] = ReadADC(TP3);    // which voltage has Pin 3?

    if ((PartFound == PART_CELL) || (adcmv[0] < CAP_EMPTY_LEVEL)
        & (adcmv[1] < CAP_EMPTY_LEVEL)
        & (adcmv[2] < CAP_EMPTY_LEVEL)) {
      ADC_DDR = TXD_MSK;    // switch all ADC-Pins to input
      R_DDR = 0;      // switch all R_L Ports (and R_H) to input
      return;       // all is discharged
    }

    // all Pins with voltage lower than 1V can be connected directly to GND (ADC-Port)
    if (adcmv[0] < 1000) {
      adc_gnd |= (1 << TP1); // Pin 1 directly to GND
    }
    if (adcmv[1] < 1000) {
      adc_gnd |= (1 << TP2); // Pin 2 directly to GND
    }
    if (adcmv[2] < 1000) {
      adc_gnd |= (1 << TP3); // Pin 3 directly to  GND
    }
    ADC_DDR = adc_gnd;    // switch all selected ADC-Ports at the same time

    // additionally switch the leaving Ports with R_L to GND.
    // since there is no disadvantage for the already directly switched pins, we can
    // simply switch all  R_L resistors to GND
    R_DDR = (1 << (TP3 * 2)) | (1 << (TP2 * 2)) | (1 << (TP1 * 2)); // Pins across R_L resistors to GND

    for (clr_cnt = 0; clr_cnt < MAX_ENTLADE_ZEIT; clr_cnt++) {
      wdt_reset();
      adcmv[0] = W20msReadADC(TP1); // which voltage has Pin 1?
      adcmv[1] = ReadADC(TP2);    // which voltage has Pin 2?
      adcmv[2] = ReadADC(TP3);    // which voltage has Pin 3?

      if (adcmv[0] < 1300) {
        ADC_DDR |= (1 << TP1); // below 1.3V , switch directly with ADC-Port to GND
      }
      if (adcmv[1] < 1300) {
        ADC_DDR |= (1 << TP2); // below 1.3V, switch directly with ADC-Port to GND
      }
      if (adcmv[2] < 1300) {
        ADC_DDR |= (1 << TP3); // below 1.3V, switch directly with ADC-Port to GND
      }
      if ((adcmv[0] < (CAP_EMPTY_LEVEL + 2)) && (adcmv[1] < (CAP_EMPTY_LEVEL + 2)) && (adcmv[2] < (CAP_EMPTY_LEVEL + 2))) {
        break;
      }
    }

    if (clr_cnt == MAX_ENTLADE_ZEIT) {
      PartFound = PART_CELL;    // mark as Battery
      // there is charge on capacitor, warn later!
    }

    for (adcmv[0] = 0; adcmv[0] < clr_cnt; adcmv[0]++) {
      // for safety, discharge 5% of discharge  time
      wait1ms();
    }
  }  // end for lop_cnt
}

#ifdef AUTO_RH
void RefVoltage(void) {
  // RefVoltage interpolates table RHtab corresponding to voltage ref_mv .
  // RHtab contain the factors to get capacity from load time with 470k for
  // different Band gab reference voltages.
  // for remember:
  // resistor     470000 Ohm      1000 1050 1100 1150 1200 1250 1300 1350 1400  mV
  // uint16_t RHTAB[] MEM_TEXT = { 954, 903, 856, 814, 775, 740, 707, 676, 648};

#define Ref_Tab_Abstand 50    // displacement of table is 50mV
#define Ref_Tab_Beginn 1000   // begin of table is 1000mV

  unsigned int referenz;
  unsigned int y1, y2;
  uint8_t tabind;
  uint8_t tabres;

#ifdef AUTO_CAL
  referenz = ref_mv + (int16_t)eeprom_read_word((uint16_t *)(&ref_offset));
#else
  referenz = ref_mv + REF_C_KORR;
#endif

  if (referenz >= Ref_Tab_Beginn) {
    referenz -= Ref_Tab_Beginn;
  } else {
    referenz = 0; // limit to begin of table
  }

  tabind = referenz / Ref_Tab_Abstand;
  tabres = referenz % Ref_Tab_Abstand;
  tabres = Ref_Tab_Abstand - tabres;

  if (tabind > 7) {
    tabind = 7;   // limit to end of table
  }

  // interpolate the table of factors
  y1 = pgm_read_word(&RHtab[tabind]);
  y2 = pgm_read_word(&RHtab[tabind + 1]);
  // RHmultip is the interpolated factor to compute capacity from load time with 470k
  RHmultip = ((y1 - y2) * tabres + (Ref_Tab_Abstand / 2)) / Ref_Tab_Abstand + y2;
}
#endif

#ifdef LCD_CLEAR
void lcd_clear_line(void) {
  // writes 20 spaces to LCD-Display, Cursor must be positioned to first column
  unsigned char ll;
  for (ll = 0; ll < 20; ll++) {
    lcd_space();
  }
}
#endif

#ifndef INHIBIT_SLEEP_MODE
// set the processor to sleep state
// wake up will be done with compare match interrupt of counter 2
void sleep_5ms(uint16_t pause) {
  // pause is the delay in 5ms units
  uint8_t t2_offset;

#define RESTART_DELAY_US (RESTART_DELAY_TICS/(F_CPU/1000000UL))
  // for 8 MHz crystal the Restart delay is 16384/8 = 2048us

  while (pause > 0) {

#if 3000 > RESTART_DELAY_US
    if (pause > 1) {
      // Startup time is too long with 1MHz Clock!!!!
      t2_offset = (10000 - RESTART_DELAY_US) / T2_PERIOD; // set to 10ms above the actual counter
      pause -= 2;
    } else {
      t2_offset = (5000 - RESTART_DELAY_US) / T2_PERIOD;  // set to 5ms above the actual counter
      pause = 0;
    }

    OCR2A = TCNT2 + t2_offset;      // set the compare value
    TIMSK2 = (0 << OCIE2B) | (1 << OCIE2A) | (0 << TOIE2); // enable output compare match A interrupt

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    //set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
    // wake up after output compare match interrupt
#else
    // restart delay ist too long, use normal delay of 5ms
    wait5ms();
#endif

    wdt_reset();
  }

  TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2); // disable output compare match A interrupt
}
#endif

// show the Pin Layout of the device
void PinLayout(char pin1, char pin2, char pin3) {
// pin1-3 is EBC or SGD or CGA
#ifndef EBC_STYLE
  // Layout with 123= style
  lcd_fix_string(N123_str);     // " 123="
  for (ii = 0; ii < 3; ii++) {
    if (ii == trans.e)  lcd_data(pin1); // Output Character in right order
    if (ii == trans.b)  lcd_data(pin2);
    if (ii == trans.c)  lcd_data(pin3);
  }
#else
#if EBC_STYLE == 321
  // Layout with 321= style
  lcd_fix_string(N321_str);     // " 321="
  ii = 3;
  while (ii != 0) {
    ii--;
    if (ii == trans.e)  lcd_data(pin1); // Output Character in right order
    if (ii == trans.b)  lcd_data(pin2);
    if (ii == trans.c)  lcd_data(pin3);
  }
#else
  // Layout with EBC= style
  lcd_space();
  lcd_data(pin1);
  lcd_data(pin2);
  lcd_data(pin3);
  lcd_data('=');
  lcd_testpin(trans.e);
  lcd_testpin(trans.b);
  lcd_testpin(trans.c);
#endif
#endif
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// Get residual current in reverse direction of a diode
//=================================================================
void GetIr(uint8_t hipin, uint8_t lopin) {
  unsigned int u_res;     // reverse voltage at 470k
  unsigned int ir_nano;
  //unsigned int ir_micro;
  uint8_t LoPinR_L;
  uint8_t HiADC;

  HiADC = pgm_read_byte(&PinADCtab[hipin]);
  ADC_PORT = HiADC | TXD_VAL;     // switch ADC port to high level
  ADC_DDR = HiADC | TXD_MSK;      // switch High Pin direct to VCC
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);   // R_L mask for LowPin R_L load
  R_PORT = 0;         // switch R-Port to GND
  R_DDR = LoPinR_L + LoPinR_L;      // switch R_H port for LowPin to output (GND)

  u_res = W5msReadADC(lopin);   // read voltage
  if (u_res == 0) return;   // no Output, if no current in reverse direction

#if defined(NOK5110) || defined(OLED096)
  lcd_line4();
#endif

  lcd_fix_string(Ir_str);   // output text "  Ir="

#ifdef WITH_IRMICRO
  if (u_res < 2500) {
#endif

    // R_H_VAL has units of 10 Ohm, u_res has units of mV, ir_nano has units of nA
    ir_nano = (unsigned long)(u_res * 100000UL) / R_H_VAL;
    DisplayValue(ir_nano, -9, 'A', 2); // output two digits of current with nA units

#ifdef WITH_IRMICRO
  } else {
    R_DDR = LoPinR_L;     // switch R_L port for LowPin to output (GND)
    u_res = W5msReadADC(lopin); // read voltage
    ir_nano = 0xffff;     // set to max
    // RR680MI has units of 0.1 Ohm, u_res has units of mV, ir_micro has units of uA
    ir_micro = (unsigned long)(u_res * 10000UL) / RR680MI;
    DisplayValue(ir_micro, -6, 'A', 2); // output two digits of current in uA units
  }
#endif

  ADC_DDR = TXD_MSK;      // switch off
  ADC_PORT = TXD_VAL;     // switch off
  R_DDR = 0;        // switch off current

  return;
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

/*
  extern struct ADCconfig_t{
  uint8_t Samples;              // number of ADC samples to take
  uint8_t RefFlag;              // save Reference type VCC of IntRef
  uint16_t U_Bandgap;           // Reference Voltage in mV
  uint16_t U_AVCC;    // Voltage of AVCC
  } ADCconfig;
*/

#ifdef INHIBIT_SLEEP_MODE
//#define StartADCwait() ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; /* enable ADC and start */
#define StartADCwait() ADCSRA = StartADCmsk; /* Start conversion */\
  while (ADCSRA & (1 << ADSC))  /* wait until conversion is done */
#else
#define StartADCwait() ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV; /* enable ADC and Interrupt */\
  set_sleep_mode(SLEEP_MODE_ADC);\
  sleep_mode(); /* Start ADC, return, if ADC has finished */
#endif

unsigned int ReadADC (uint8_t Probe) {
  unsigned int U;     // return value (mV)
  uint8_t Samples;    // loop counter
  unsigned long Value;    // ADC value
  Probe |= (1 << REFS0);  // use internal reference anyway

#ifdef AUTOSCALE_ADC
sample:
#endif

  ADMUX = Probe;  // set input channel and U reference

#ifdef AUTOSCALE_ADC
  // if voltage reference changes, wait for voltage stabilization
  if ((Probe & (1 << REFS1)) != 0) {
    // switch to 1.1V Reference
#ifdef NO_AREF_CAP
    wait100us();    // time for voltage stabilization
#else
    wait_about10ms();   // time for voltage stabilization
#endif
  }
#endif

  // allways do one dummy read of ADC, 112us
  StartADCwait();   // start ADC and wait

  // sample ADC readings
  Value = 0UL;      // reset sampling variable
  Samples = 0;      // number of samples to take

  while (Samples < ADCconfig.Samples) {   // take samples
    StartADCwait();       // start ADC and wait
    Value += ADCW;        // add ADC reading

#ifdef AUTOSCALE_ADC
    // auto-switch voltage reference for low readings
    if ((Samples == 4) && (ADCconfig.U_Bandgap > 255) && ((uint16_t)Value < 1024) && !(Probe & (1 << REFS1))) {
      Probe |= (1 << REFS1);    // select internal bandgap reference

#if PROCESSOR_TYP == 1280
      Probe &= ~(1 << REFS0); // ATmega640/1280/2560 1.1V Reference with REFS0=0
#endif

      goto sample;  // re-run sampling
    }
#endif

    Samples++;    // one more done
  }

#ifdef AUTOSCALE_ADC
  // convert ADC reading to voltage - single sample: U = ADC reading * U_ref / 1024
  // get voltage of reference used
  if (Probe & (1 << REFS1)) U = ADCconfig.U_Bandgap;  // bandgap reference
  else U = ADCconfig.U_AVCC;  // Vcc reference
#else
  U = ADCconfig.U_AVCC;   // Vcc reference
#endif

  // convert to voltage
  Value *= U;     // ADC readings * U_ref
  Value /= 1023;  // / 1024 for 10bit ADC

  // de-sample to get average voltage
  Value /= ADCconfig.Samples;
  U = (unsigned int)Value;
  return U;
  //return ((unsigned int)(Value / (1023 * (unsigned long)ADCconfig.Samples)));
}

unsigned int W5msReadADC (uint8_t Probe) {
  wait_about5ms();
  return (ReadADC(Probe));
}

unsigned int W10msReadADC (uint8_t Probe) {
  wait_about10ms();
  return (ReadADC(Probe));
}

unsigned int W20msReadADC (uint8_t Probe) {
  wait_about20ms();
  return (ReadADC(Probe));
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */
