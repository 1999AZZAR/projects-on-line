// new code by K.-H. Kubbeler
// ReadCapacity tries to find the value of a capacitor by measuring the load time.
// first of all the capacitor is discharged.
// Then a series of up to 500 load pulses with 10ms duration each is done across the R_L (680Ohm)
// resistor.
// After each load pulse the voltage of the capacitor is measured without any load current.
// If voltage reaches a value of more than 300mV and is below 1.3V, the capacity can be
// computed from load time and voltage by a interpolating a build in table.
// If the voltage reaches a value of more than 1.3V with only one load pulse,
// another measurement methode is used:
// The build in 16bit counter can save the counter value at external events.
// One of these events can be the output change of a build in comparator.
// The comparator can compare the voltage of any of the ADC input pins with the voltage
// of the internal reference (1.3V or 1.1V).
// After setting up the comparator and counter properly, the load of capacitor is started
// with connecting the positive pin with the R_H resistor (470kOhm) to VCC and immediately
// the counter is started. By counting the overflow Events of the 16bit counter and watching
// the counter event flag  the total load time of the capacitor until reaching the internal
// reference voltage can be measured.
// If any of the tries to measure the load time is successful,
// the following variables are set:
// cap.cval = value of the capacitor
// cap.cval_uncorrected = value of the capacitor uncorrected
// cap.esr = serial resistance of capacitor,  0.01 Ohm units
// cap.cpre = units of cap.cval (-12==pF, -9=nF, -6=uF)
// ca   = Pin number (0-2) of the LowPin
// cb   = Pin number (0-2) of the HighPin

//=================================================================
void ReadCapacity(uint8_t HighPin, uint8_t LowPin) {
  // check if capacitor and measure the capacity value
  unsigned int tmpint;
  unsigned int adcv[4];

#ifdef INHIBIT_SLEEP_MODE
  unsigned int ovcnt16;
#endif

  uint8_t HiPinR_L, HiPinR_H;
  uint8_t LoADC;
  uint8_t ii;

#if FLASHEND > 0x1fff
  unsigned int vloss;   // lost voltage after load pulse in 0.1%
#endif

#ifdef AUTO_CAL
  pin_combination = (HighPin * 3) + LowPin - 1; // coded Pin combination for capacity zero offset
#endif

  LoADC = pgm_read_byte(&PinADCtab[LowPin]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[HighPin]);   // R_L mask for HighPin R_L load
  HiPinR_H = HiPinR_L + HiPinR_L;     // double for HighPin R_H load

#if DebugOut == 10
  lcd_line3();
  lcd_clear_line();
  lcd_line3();
  lcd_testpin(LowPin);
  lcd_data('C');
  lcd_testpin(HighPin);
  lcd_space();
#endif

  if (PartFound == PART_RESISTOR) {

#if DebugOut == 10
    lcd_data('R');
    wait_about2s();
#endif

    return; // We have found a resistor already
  }

  for (ii = 0; ii < NumOfDiodes; ii++) {
    if ((diodes[ii].Cathode == LowPin) && (diodes[ii].Anode == HighPin) && (diodes[ii].Voltage < 1500)) {

#if DebugOut == 10
      lcd_data('D');
      wait_about2s();
#endif

      return;
    }
  }

#if FLASHEND > 0x1fff
  cap.esr = 0;      // set ESR of capacitor to zero
  vloss = 0;        // set lost voltage to zero
#endif

  cap.cval = 0;       // set capacity value to zero
  cap.cpre = -12;     // default unit is pF
  EntladePins();      // discharge capacitor

  ADC_PORT = TXD_VAL;     // switch ADC-Port to GND
  R_PORT = 0;       // switch R-Port to GND
  ADC_DDR = LoADC;      // switch Low-Pin to output (GND)
  R_DDR = HiPinR_L;     // switch R_L port for HighPin to output (GND)
  adcv[0] = ReadADC(HighPin);   // voltage before any load

  // ******** should adcv[0] be measured without current???
  adcv[2] = adcv[0];      // preset to prevent compiler warning

  for (ovcnt16 = 0; ovcnt16 < 500; ovcnt16++) {
    R_PORT = HiPinR_L;      // R_L to 1 (VCC)
    R_DDR = HiPinR_L;     // switch Pin to output, across R to GND or VCC
    wait10ms();       // wait exactly 10ms, do not sleep

    R_DDR = 0;        // switch back to input
    R_PORT = 0;       // no Pull up
    wait500us();      // wait a little time

    wdt_reset();

    // read voltage without current, is already charged enough?
    adcv[2] = ReadADC(HighPin);

    if (adcv[2] > adcv[0]) {
      adcv[2] -= adcv[0];   // difference to beginning voltage
    } else {
      adcv[2] = 0;      // voltage is lower or same as beginning voltage
    }

    if ((ovcnt16 == 126) && (adcv[2] < 75)) {
      // 300mV can not be reached well-timed
      break;    // don't try to load any more
    }

    if (adcv[2] > 300) {
      break;    // probably 100mF can be charged well-timed
    }
  }

  // wait 5ms and read voltage again, does the capacitor keep the voltage?
  //adcv[1] = W5msReadADC(HighPin) - adcv[0];
  //wdt_reset();

#if DebugOut == 10
  DisplayValue(ovcnt16, 0, ' ', 4);
  DisplayValue(adcv[2], -3, 'V', 4);
#endif

  if (adcv[2] < 301) {

#if DebugOut == 10
    lcd_data('K');
    lcd_space();
    wait1s();
#endif

    //if (NumOfDiodes != 0) goto messe_mit_rh;
    goto keinC;   // was never charged enough, >100mF or shorted
  }

  // voltage is rised properly and keeps the voltage enough
  if ((ovcnt16 == 0 ) && (adcv[2] > 1300)) {
    goto messe_mit_rh;    // Voltage of more than 1300mV is reached in one pulse, too fast loaded
  }

  // Capacity is more than about 50uF

#ifdef NO_CAP_HOLD_TIME
  ChargePin10ms(HiPinR_H, 0);   // switch HighPin with R_H 10ms auf GND, then currentless
  adcv[3] = ReadADC(HighPin) - adcv[0];  // read voltage again, is discharged only a little bit ?

  if (adcv[3] > adcv[0]) {
    adcv[3] -= adcv[0];   // difference to beginning voltage
  } else {
    adcv[3] = 0;      // voltage is lower to beginning voltage
  }

#if DebugOut == 10
  lcd_data('U');
  lcd_data('3');
  lcd_data(':');
  lcd_string(utoa(adcv[3], outval, 10));
  lcd_space();
  wait_about2s();
#endif

  if ((adcv[3] + adcv[3]) < adcv[2]) {

#if DebugOut == 10
    lcd_data('H');
    lcd_space();
    wait_about1s();
#endif

    if (ovcnt16 == 0 )  {
      goto messe_mit_rh;    // Voltage of more than 1300mV is reached in one pulse, but not hold
    }

    goto keinC;  // implausible, not yet the half voltage
  }

  cap.cval_uncorrected.dw = ovcnt16 + 1;
  cap.cval_uncorrected.dw *= getRLmultip(adcv[2]);  // get factor to convert time to capacity from table

#else
  // wait the half the time which was required for loading
  adcv[3] = adcv[2];      // preset to prevent compiler warning

  for (tmpint = 0; tmpint <= ovcnt16; tmpint++) {
    wait5ms();
    adcv[3] = ReadADC(HighPin); // read voltage again, is discharged only a little bit ?
    wdt_reset();
  }

  if (adcv[3] > adcv[0]) {
    adcv[3] -= adcv[0];   // difference to beginning voltage
  } else {
    adcv[3] = 0;      // voltage is lower or same as beginning voltage
  }

  if (adcv[2] > adcv[3]) {
    // build difference to load voltage
    adcv[3] = adcv[2] - adcv[3];  // lost voltage during load time wait
  } else {
    adcv[3] = 0;      // no lost voltage
  }

#if FLASHEND > 0x1fff
  // compute equivalent parallel resistance from voltage drop
  if (adcv[3] > 0) {
    // there is any voltage drop (adcv[3]) !
    // adcv[2] is the loaded voltage.
    vloss = (unsigned long)(adcv[3] * 1000UL) / adcv[2];
  }
#endif

  if (adcv[3] > 100) {
    // more than 100mV is lost during load time

#if DebugOut == 10
    lcd_data('L');
    lcd_space();
    wait_about1s();
#endif

    if (ovcnt16 == 0 )  {
      goto messe_mit_rh;    // Voltage of more than 1300mV is reached in one pulse, but not hold
    }

    goto keinC;     // capacitor does not keep the voltage about 5ms
  }

  cap.cval_uncorrected.dw = ovcnt16 + 1;
  // compute factor with load voltage + lost voltage during the voltage load time
  cap.cval_uncorrected.dw *= getRLmultip(adcv[2] + adcv[3]); // get factor to convert time to capacity from table
#endif

  cap.cval = cap.cval_uncorrected.dw; // set result to uncorrected
  cap.cpre = -9;      // switch units to nF
  Scale_C_with_vcc();

  // cap.cval for this type is at least 40000nF, so the last digit will be never shown
  cap.cval *= (1000 - C_H_KORR);  // correct with C_H_KORR with 0.1% resolution, but prevent overflow
  cap.cval /= 100;

#if DebugOut == 10
  lcd_line3();
  lcd_clear_line();
  lcd_line3();
  lcd_testpin(LowPin);
  lcd_data('C');
  lcd_testpin(HighPin);
  lcd_space();
  DisplayValue(cap.cval, cap.cpre, 'F', 4);
  lcd_space();
  lcd_string(utoa(ovcnt16, outval, 10));
  wait_about3s();
#endif

  goto checkDiodes;

  //==================================================================================
  // Measurement of little capacity values
messe_mit_rh:
  // little capacity value, about  < 50 uF
  EntladePins();      // discharge capacitor

  // measure with the R_H (470kOhm) resistor
  R_PORT = 0;   // R_DDR ist HiPinR_L
  ADC_DDR = (1 << TP1) | (1 << TP2) | (1 << TP3) | (1 << TxD); // switch all Pins to output
  ADC_PORT = TXD_VAL;   // switch all ADC Pins to GND
  R_DDR = HiPinR_H;       // switch R_H resistor port for HighPin to output (GND)

  // setup Analog Comparator
  ADC_COMP_CONTROL = (1 << ACME);   // enable Analog Comparator Multiplexer
  ACSR =  (1 << ACBG) | (1 << ACI)  | (1 << ACIC); // enable, 1.3V, no Interrupt, Connect to Timer1
  ADMUX = (1 << REFS0) | HighPin;   // switch Mux to High-Pin
  ADCSRA = (1 << ADIF) | AUTO_CLOCK_DIV;  // disable ADC
  wait200us();          // wait for bandgap to start up

  // setup Counter1
  ovcnt16 = 0;
  TCCR1A = 0;     // set Counter1 to normal Mode
  TCNT1 = 0;      // set Counter to 0
  TI1_INT_FLAGS = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1); // clear interrupt flags

#ifndef INHIBIT_SLEEP_MODE
  TIMSK1 = (1 << TOIE1) | (1 << ICIE1); // enable Timer overflow interrupt and input capture interrupt
  unfinished = 1;
#endif

  R_PORT = HiPinR_H;            // switch R_H resistor port for HighPin to VCC

  if (PartFound == PART_FET) {
    // charge capacitor with R_H resistor
    TCCR1B = (1 << CS10); //Start counter 1MHz or 8MHz
    ADC_DDR = (((1 << TP1) | (1 << TP2) | (1 << TP3) | TXD_MSK) & ~(1 << HighPin)); // release only HighPin ADC port
  } else {
    TCCR1B =  (1 << CS10); // start counter 1MHz or 8MHz
    ADC_DDR = LoADC;    // stay LoADC Pin switched to GND, charge capacitor with R_H slowly
  }

  //******************************
#ifdef INHIBIT_SLEEP_MODE
  while (1) {
    // Wait, until  Input Capture is set
    ii = TI1_INT_FLAGS; // read Timer flags

    if (ii & (1 << ICF1))  {
      break;
    }

    if ((ii & (1 << TOV1))) { // counter overflow, 65.536 ms @ 1MHz, 8.192ms @ 8MHz
      TI1_INT_FLAGS = (1 << TOV1); // Reset OV Flag
      wdt_reset();
      ovcnt16++;

      if (ovcnt16 == (F_CPU / 5000)) {
        break;      // Timeout for Charging, above 12 s
      }
    }
  }

  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << CS10); // stop counter
  TI1_INT_FLAGS = (1 << ICF1);  // Reset Input Capture
  tmpint = ICR1;      // get previous Input Capture Counter flag

  // check actual counter, if an additional overflow must be added
  if ((TCNT1 > tmpint) && (ii & (1 << TOV1))) {
    // this OV was not counted, but was before the Input Capture
    TI1_INT_FLAGS = (1 << TOV1); // Reset OV Flag
    ovcnt16++;
  }

#else
  while (unfinished) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();         // wait for interrupt
    wdt_reset();

    if (ovcnt16 == (F_CPU / 5000)) {
      break;        // Timeout for Charging, above 12 s
    }
  }

  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << CS10); // stop counter
  tmpint = ICR1;          // get previous Input Capture Counter flag
  TIMSK1 = (0 << TOIE1) | (0 << ICIE1); // disable Timer overflow interrupt and input capture interrupt

  if (TCNT1 < tmpint) {
    ovcnt16--;      // one ov to much
  }
#endif

  //------------------------------------------------------------
  ADCSRA = (1 << ADEN) | (1 << ADIF) | AUTO_CLOCK_DIV; // enable ADC
  R_DDR = 0;          // switch R_H resistor port for input
  R_PORT = 0;         // switch R_H resistor port pull up for HighPin off
  adcv[2] = ReadADC(HighPin);       // get loaded voltage
  load_diff = adcv[2] + REF_C_KORR - ref_mv;  // build difference of capacitor voltage to Reference Voltage
  //------------------------------------------------------------

  if (ovcnt16 >= (F_CPU / 10000)) {

#if DebugOut == 10
    lcd_data('k');
    wait_about1s();
#endif

    goto keinC; // no normal end
  }

  //cap.cval_uncorrected = CombineII2Long(ovcnt16, tmpint);
  cap.cval_uncorrected.w[1] = ovcnt16;
  cap.cval_uncorrected.w[0] = tmpint;

  cap.cpre = -12;     // cap.cval unit is pF
  if (ovcnt16 > 65) {
    cap.cval_uncorrected.dw /= 100; // switch to next unit
    cap.cpre += 2;      // set unit, prevent overflow
  }

  cap.cval_uncorrected.dw *= RHmultip;    // 708
  cap.cval_uncorrected.dw /= (F_CPU / 10000); // divide by 100 (@ 1MHz clock), 800 (@ 8MHz clock)
  cap.cval = cap.cval_uncorrected.dw;   // set the corrected cap.cval
  Scale_C_with_vcc();

  if (cap.cpre == -12) {
#if COMP_SLEW1 > COMP_SLEW2
    if (cap.cval < COMP_SLEW1) {
      // add slew rate dependent offset
      cap.cval += (COMP_SLEW1 / (cap.cval + COMP_SLEW2 ));
    }
#endif

#ifdef AUTO_CAL
    // auto calibration mode, cap_null can be updated in selftest section
    tmpint = eeprom_read_byte(&c_zero_tab[pin_combination]);  // read zero offset

    if (cap.cval > tmpint) {
      cap.cval -= tmpint;   // subtract zero offset (pF)
    } else {
      cap.cval = 0;     // unsigned long may not reach negativ value
    }

#else
    if (HighPin == TP2) cap.cval += TP2_CAP_OFFSET; // measurements with TP2 have 2pF less capacity

    if (cap.cval > C_NULL) {
      cap.cval -= C_NULL;   // subtract constant offset (pF)
    } else {
      cap.cval = 0;     // unsigned long may not reach negativ value
    }
#endif
  }

#if DebugOut == 10
  R_DDR = 0;      // switch all resistor ports to input
  lcd_line4();
  lcd_clear_line();
  lcd_line4();
  lcd_testpin(LowPin);
  lcd_data('c');
  lcd_testpin(HighPin);
  lcd_space();
  DisplayValue(cap.cval, cap.cpre, 'F', 4);
  wait_about3s();
#endif

  R_DDR = HiPinR_L;     // switch R_L for High-Pin to GND

#if F_CPU < 2000001
  if (cap.cval < 50)
#else
  if (cap.cval < 25)
#endif
  {
    // cap.cval can only be so little in pF unit, cap.cpre must not be testet!

#if DebugOut == 10
    lcd_data('<');
    lcd_space();
    wait_about1s();
#endif

    goto keinC;  // capacity to low, < 50pF @1MHz (25pF @8MHz)
  }

  // end low capacity

checkDiodes:

  if ((NumOfDiodes > 0)  && (PartFound != PART_FET)) {

#if DebugOut == 10
    lcd_data('D');
    lcd_space();
    wait_about1s();
#endif

    // nearly shure, that there is one or more diodes in reverse direction,
    // which would be wrongly detected as capacitor
  } else {
    PartFound = PART_CAPACITOR;   // capacitor is found

    if ((cap.cpre > cap.cpre_max) || ((cap.cpre == cap.cpre_max) && (cap.cval > cap.cval_max))) {
      // we have found a greater one
      cap.cval_max = cap.cval;
      cap.cpre_max = cap.cpre;

#if FLASHEND > 0x1fff
      cap.v_loss = vloss;   // lost voltage in 0.01%
#endif

      cap.ca = LowPin;    // save LowPin
      cap.cb = HighPin;   // save HighPin
    }
  }

keinC:

  // discharge capacitor again
  //EntladePins();    // discharge capacitors
  // ready
  // switch all ports to input
  ADC_DDR =  TXD_MSK;   // switch all ADC ports to input
  ADC_PORT = TXD_VAL;   // switch all ADC outputs to GND, no pull up
  R_DDR = 0;      // switch all resistor ports to input
  R_PORT = 0;       // switch all resistor outputs to GND, no pull up

  return;
}  // end ReadCapacity()


unsigned int getRLmultip(unsigned int cvolt) {

  // interpolate table RLtab corresponding to voltage cvolt
  // Widerstand 680 Ohm          300   325   350   375   400   425   450   475   500   525   550   575   600   625   650   675   700   725   750   775   800   825   850   875   900   925   950   975  1000  1025  1050  1075  1100  1125  1150  1175  1200  1225  1250  1275  1300  1325  1350  1375  1400  mV
  //uint16_t RLtab[] MEM_TEXT = {22447,20665,19138,17815,16657,15635,14727,13914,13182,12520,11918,11369,10865,10401, 9973, 9577, 9209, 8866, 8546, 8247, 7966, 7702, 7454, 7220, 6999, 6789, 6591, 6403, 6224, 6054, 5892, 5738, 5590, 5449, 5314, 5185, 5061, 4942, 4828, 4718, 4613, 4511, 4413, 4319, 4228};

#define RL_Tab_Abstand 25     // displacement of table 25mV
#define RL_Tab_Beginn 300     // begin of table ist 300mV
#define RL_Tab_Length 1100    // length of table is 1400-300

  unsigned int uvolt;
  unsigned int y1, y2;
  uint8_t tabind;
  uint8_t tabres;

  if (cvolt >= RL_Tab_Beginn) {
    uvolt = cvolt - RL_Tab_Beginn;
  } else {
    uvolt = 0;      // limit to begin of table
  }

  tabind = uvolt / RL_Tab_Abstand;
  tabres = uvolt % RL_Tab_Abstand;
  tabres = RL_Tab_Abstand - tabres;

  if (tabind > (RL_Tab_Length / RL_Tab_Abstand)) {
    tabind = (RL_Tab_Length / RL_Tab_Abstand); // limit to end of table
  }

  y1 = MEM_read_word(&RLtab[tabind]);
  y2 = MEM_read_word(&RLtab[tabind + 1]);
  return ( ((y1 - y2) * tabres + (RL_Tab_Abstand / 2)) / RL_Tab_Abstand + y2); // interpolate table
}

void Scale_C_with_vcc(void) {

  while (cap.cval > 100000) {
    cap.cval /= 10;
    cap.cpre ++;      // prevent overflow
  }

  cap.cval *= ADCconfig.U_AVCC;   // scale with measured voltage
  cap.cval /= U_VCC;      // Factors are computed for U_VCC
}

#ifndef INHIBIT_SLEEP_MODE
// Interrupt Service Routine for timer1 Overflow
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
  ovcnt16++;        // count overflow
}

// Interrupt Service Routine for timer1 capture event (Comparator)
ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
  unfinished = 0;     // clear unfinished flag
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// new code by K.-H. Kubbeler
// The 680 Ohm resistor (R_L_VAL) at the Lowpin will be used as current sensor
// The current with a coil will with (1 - e**(-t*R/L)), where R is
// the sum of Pin_RM , R_L_VAL , Resistance of coil and Pin_RP.
// L in the inductance of the coil.

//=================================================================
void ReadInductance(void) {
#if FLASHEND > 0x1fff

  // check if inductor and measure the inductance value
  unsigned int tmpint;
  unsigned int umax;
  unsigned int total_r;   // total resistance of current loop
  unsigned int mess_r;    // value of resistor used for current measurement
  unsigned long inductance[4];  // four inductance values for different measurements

  union t_combi {
    unsigned long dw;     // time_constant
    uint16_t w[2];
  } timeconstant;

  uint16_t per_ref1, per_ref2; // percentage
  uint8_t LoPinR_L; // Mask for switching R_L resistor of low pin
  uint8_t HiADC;  // Mask for switching the high pin direct to VCC
  uint8_t ii;
  uint8_t count;  // counter for the different measurements

  //uint8_t found;  // variable used for searching resistors
#define found 0

  uint8_t cnt_diff;     // resistance dependent offset
  uint8_t LowPin; // number of pin with low voltage
  uint8_t HighPin;  // number of pin with high voltage
  int8_t ukorr;   // correction of comparator voltage
  uint8_t nr_pol1;  // number of successfull inductance measurement with polarity 1
  uint8_t nr_pol2;  // number of successfull inductance measurement with polarity 2

  if (PartFound != PART_RESISTOR) {
    return; // We have found no resistor
  }

  if (ResistorsFound != 1) {
    return; // do not search for inductance, more than 1 resistor
  }

  //for (found=0;found<ResistorsFound;found++) {
  //  if (resis[found].rx > 21000) continue;

  if (resis[found].rx > 21000) return;
  // we can check for Inductance, if resistance is below 2100 Ohm

  for (count = 0; count < 4; count++) {
    // Try four times (different direction and with delayed counter start)

    if (count < 2) {
      // first and second pass, direction 1
      LowPin = resis[found].ra;
      HighPin = resis[found].rb;
    } else {
      // third and fourth pass, direction 2
      LowPin = resis[found].rb;
      HighPin = resis[found].ra;
    }

    HiADC = pgm_read_byte(&PinADCtab[HighPin]);
    LoPinR_L = pgm_read_byte(&PinRLtab[LowPin]);  // R_L mask for HighPin R_L load

    //==================================================================================
    // Measurement of Inductance values
    R_PORT = 0;     // switch R port to GND
    ADC_PORT = TXD_VAL;   // switch ADC-Port to GND

    if ((resis[found].rx < 240) && ((count & 0x01) == 0)) {
      // we can use PinR_L for measurement
      mess_r = RR680MI - R_L_VAL;     // use only pin output resistance
      ADC_DDR = HiADC | (1 << LowPin) | TXD_MSK;  // switch HiADC and Low Pin to GND,
    } else {
      R_DDR = LoPinR_L;       // switch R_L resistor port for LowPin to output (GND)
      ADC_DDR = HiADC | TXD_MSK;  // switch HiADC Pin to GND
      mess_r = RR680MI;     // use 680 Ohm and PinR_L for current measurement
    }

    // Look, if we can detect any current
    for (ii = 0; ii < 20; ii++) {
      // wait for current is near zero
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);
      if ((umax < 2) && (total_r < 2)) break; // low current detected
    }

    // setup Analog Comparator
    ADC_COMP_CONTROL = (1 << ACME);   // enable Analog Comparator Multiplexer
    ACSR =  (1 << ACBG) | (1 << ACI)  | (1 << ACIC); // enable, 1.3V, no Interrupt, Connect to Timer1
    ADMUX = (1 << REFS0) | LowPin;    // switch Mux to Low-Pin
    ADCSRA = (1 << ADIF) | AUTO_CLOCK_DIV;  // disable ADC

    // setup Counter1
    timeconstant.w[1] = 0;  // set ov counter to 0
    TCCR1A = 0;     // set Counter1 to normal Mode
    TCNT1 = 0;      // set Counter to 0
    TI1_INT_FLAGS = (1 << ICF1) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1); // reset TIFR or TIFR1
    HiADC |= TXD_VAL;
    wait200us();    // wait for bandgap to start up

    if ((count & 0x01) == 0 ) {
      // first start counter, then start current
      TCCR1B =  (1 << ICNC1) | (0 << ICES1) | (1 << CS10); // start counter 1MHz or 8MHz
      ADC_PORT = HiADC;         // switch ADC-Port to VCC
    } else {
      // first start current, then start counter with delay
      // parasitic capacity of coil can cause high current at the beginning
      ADC_PORT = HiADC;   // switch ADC-Port to VCC

#if F_CPU >= 8000000UL
      wait3us();    // ignore current peak from capacity
#else
      wdt_reset();    // delay
      wdt_reset();    // delay
#endif

      TI1_INT_FLAGS = (1 << ICF1);    // Reset Input Capture
      TCCR1B =  (1 << ICNC1) | (0 << ICES1) | (1 << CS10); // start counter 1MHz or 8MHz
    }

    //******************************
    while (1) {
      // Wait, until  Input Capture is set
      ii = TI1_INT_FLAGS;   // read Timer flags

      if (ii & (1 << ICF1))  {
        break;
      }

      if ((ii & (1 << TOV1))) { // counter overflow, 65.536 ms @ 1MHz, 8.192ms @ 8MHz
        TI1_INT_FLAGS = (1 << TOV1); // Reset OV Flag
        wdt_reset();
        timeconstant.w[1]++;    // count one OV

        if (timeconstant.w[1] == (F_CPU / 100000UL)) {
          break;      // Timeout for Charging, above 0.13 s
        }
      }
    }

    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << CS10); // stop counter
    TI1_INT_FLAGS = (1 << ICF1);      // Reset Input Capture
    timeconstant.w[0] = ICR1;   // get previous Input Capture Counter flag

    // check actual counter, if an additional overflow must be added
    if ((TCNT1 > timeconstant.w[0]) && (ii & (1 << TOV1))) {
      // this OV was not counted, but was before the Input Capture
      TI1_INT_FLAGS = (1 << TOV1);  // Reset OV Flag
      timeconstant.w[1]++;      // count one additional OV
    }

    ADC_PORT = TXD_VAL;         // switch ADC-Port to GND
    ADCSRA = (1 << ADEN) | (1 << ADIF) | AUTO_CLOCK_DIV; // enable ADC

    for (ii = 0; ii < 20; ii++) {
      // wait for current is near zero
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);

      if ((umax < 2) && (total_r < 2)) break; // low current detected
    }

#define CNT_ZERO_42 6
#define CNT_ZERO_720 7

    //#if F_CPU == 16000000UL
    //  #undef CNT_ZERO_42
    //  #undef CNT_ZERO_720
    //  #define CNT_ZERO_42 7
    //  #define CNT_ZERO_720 10
    //#endif

    total_r = (mess_r + resis[found].rx + RRpinMI);

    //cnt_diff = 0;
    //if (total_r > 7000) cnt_diff = 1;
    //if (total_r > 14000) cnt_diff = 2;

    cnt_diff = total_r / ((14000UL * 8) / (F_CPU / 1000000UL));
    // Voltage of comparator in % of umax

#ifdef AUTO_CAL
    tmpint = (ref_mv + (int16_t)eeprom_read_word((uint16_t *)(&ref_offset))) ;
#else
    tmpint = (ref_mv + REF_C_KORR);
#endif

    if (mess_r < R_L_VAL) {
      // measurement without 680 Ohm
      cnt_diff = CNT_ZERO_42;

      if (timeconstant.dw < 225) {
        ukorr = (timeconstant.w[0] / 5) - 20;
      } else {
        ukorr = 25;
      }

      tmpint -= (((REF_L_KORR * 10) / 10) + ukorr);
    } else {
      // measurement with 680 Ohm resistor
      // if 680 Ohm resistor is used, use REF_L_KORR for correction
      cnt_diff += CNT_ZERO_720;
      tmpint += REF_L_KORR;
    }

    if (timeconstant.dw > cnt_diff) timeconstant.dw -= cnt_diff;
    else timeconstant.dw = 0;

    if ((count & 0x01) == 1) {
      // second pass with delayed counter start
      timeconstant.dw += (3 * (F_CPU / 1000000UL)) + 10;
    }

    if (timeconstant.w[1] >= (F_CPU / 100000UL)) timeconstant.dw = 0; // no transition found

    if (timeconstant.dw > 10) {
      timeconstant.dw -= 1;
    }

    // compute the maximum Voltage umax with the Resistor of the coil
    umax = ((unsigned long)mess_r * (unsigned long)ADCconfig.U_AVCC) / total_r;
    per_ref1 = ((unsigned long)tmpint * 1000) / umax;
    //per_ref2 = (uint8_t)MEM2_read_byte(&LogTab[per_ref1]);  // -log(1 - per_ref1/100)
    per_ref2 = get_log(per_ref1);       // -log(1 - per_ref1/1000)

    //*********************************************************
#if 0
    if (count == 0) {
      lcd_line3();
      DisplayValue(count, 0, ' ', 4);
      DisplayValue(timeconstant.dw, 0, '+', 4);
      DisplayValue(cnt_diff, 0, ' ', 4);
      DisplayValue(total_r, -1, 'r', 4);
      lcd_space();
      DisplayValue(per_ref1, -1, '%', 4);
      lcd_line4();
      DisplayValue(tmpint, -3, 'V', 4);
      lcd_space();
      DisplayValue(umax, -3, 'V', 4);
      lcd_space();
      DisplayValue(per_ref2, -1, '%', 4);
      wait_about4s();
      wait_about2s();
    }
#endif

    //*********************************************************
    // lx in 0.01mH units, L = Tau * R
    per_ref1 = ((per_ref2 * (F_CPU / 1000000UL)) + 5) / 10;
    inductance[count] = (timeconstant.dw * total_r ) / per_ref1;

    if (((count & 0x01) == 0) && (timeconstant.dw > ((F_CPU / 1000000UL) + 3))) {
      // transition is found, measurement with delayed counter start is not necessary
      inductance[count + 1] = inductance[count]; // set delayed measurement to same value
      count++;          // skip the delayed measurement
    }

    wdt_reset();
  }  // end for count

  ADC_PORT = TXD_VAL;   // switch ADC Port to GND
  wait_about20ms();

#if 0
  if (inductance[1] > inductance[0]) {
    resis[found].lx = inductance[1];    // use value found with delayed counter start
  } else {
    resis[found].lx = inductance[0];
  }

  if (inductance[3] > inductance[2]) inductance[2] = inductance[3];   // other polarity, delayed start

  if (inductance[2] < resis[found].lx) resis[found].lx = inductance[2]; // use the other polarity

#else
  nr_pol1 = 0;
  if (inductance[1] > inductance[0]) {
    nr_pol1 = 1;
  }

  nr_pol2 = 2;
  if (inductance[3] > inductance[2]) {
    nr_pol2 = 3;
  }

  if (inductance[nr_pol2] < inductance[nr_pol1]) nr_pol1 = nr_pol2;

  resis[found].lx = inductance[nr_pol1];
  resis[found].lpre = -5;         // 10 uH units

  if (((nr_pol1 & 1) == 1) || (resis[found].rx >= 240)) {
    // with 680 Ohm resistor total_r is more than 7460
    resis[found].lpre = -4;         // 100 uH units
    resis[found].lx = (resis[found].lx + 5) / 10;
  }
#endif

  //} // end loop for all resistors

  // switch all ports to input
  ADC_DDR =  TXD_MSK;   // switch all ADC ports to input
  R_DDR = 0;      // switch all resistor ports to input

#endif
  return;
}  // end ReadInductance()


#if FLASHEND > 0x1fff
// get_log interpolate a table with the function -log(1 - (permil/1000))
uint16_t get_log(uint16_t permil) {
  // for remember:
  // uint16_t LogTab[] PROGMEM = {0, 20, 41, 62, 83, 105, 128, 151, 174, 198, 223, 248, 274, 301, 329, 357, 386, 416, 446, 478, 511, 545, 580, 616, 654, 693, 734, 777, 821, 868, 916, 968, 1022, 1079, 1139, 1204, 1273, 1347, 1427, 1514, 1609, 1715, 1833, 1966, 2120, 2303, 2526 };

#define Log_Tab_Distance 20           // displacement of table is 20 mil

  uint16_t y1, y2;      // table values
  uint16_t result;      // result of interpolation
  uint8_t tabind;     // index to table value
  uint8_t tabres;     // distance to lower table value, fraction of Log_Tab_Distance

  tabind = permil / Log_Tab_Distance; // index to table
  tabres = permil % Log_Tab_Distance; // fraction of table distance

  // interpolate the table of factors
  y1 = pgm_read_word(&LogTab[tabind]);    // get the lower table value
  y2 = pgm_read_word(&LogTab[tabind + 1]); // get the higher table value

  result = ((y2 - y1) * tabres ) / Log_Tab_Distance + y1;  // interpolate
  return (result);
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#define MAX_CNT 255

/* The sleep mode for ADC can be used. It is implemented for 8MHz and 16MHz operation */
/* But the ESR result is allways higher than the results with wait mode. */
/* The time of ESR measurement is higher with the sleep mode (checked with oszilloscope) */
/* The reason for the different time is unknown, the start of the next ADC measurement */
/* should be initiated before the next ADC-clock (8 us). One ADC takes 13 ADC clock + 1 clock setup. */
/* The setting to sleep mode takes 10 clock tics, the wakeup takes about 24 clock tics, but 8us are 64 clock tics. */
/* I have found no reason, why a reset of the ADC clock divider should occur during ESR measurement. */
//#define ADC_Sleep_Mode

//#define ESR_DEBUG

#ifdef ADC_Sleep_Mode
//#define StartADCwait() ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV; /* enable ADC and Interrupt */
//#define StartADCwait() set_sleep_mode(SLEEP_MODE_ADC);
//sleep_mode()    /* Start ADC, return if ADC has finished */
#define StartADCwait() sleep_cpu()
#else
//#define StartADCwait() ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV; /* enable ADC and start */
#define StartADCwait() ADCSRA = StartADCmsk; /* Start conversion */\
  while (ADCSRA & (1 << ADSC))  /* wait until conversion is done */
#endif

/************************************************************************/
/* Predefine the wait time for switch off the load current for big caps */
/************************************************************************/
//         wdt_reset();   // with wdt_reset the timing can be adjusted,
// when time is too short, voltage is down before SH of ADC
// when time is too long, capacitor will be overloaded.
// That will cause too high voltage without current.

#ifdef ADC_Sleep_Mode
// Interrupt mode, big cap
#if F_CPU == 8000000UL
#define DelayBigCap() wait10us(); /* 2.5 ADC clocks = 20us */ \
  wait5us();    /*  */ \
  wait2us();  /* with only 17 us delay the voltage goes down before SH */ \
  /* delay 17us + 3 clock tics (CALL instead of RCALL) = 17.375 us @ 8 MHz */ \
  /* + 21 clock tics delay from interrupt return, +2.625us = 20.0 */  \
  wdt_reset();  /* 20.125 us  */ \
  wdt_reset()   /* 20.250 us  */
#endif
#if F_CPU == 16000000UL
#define DelayBigCap() us500delay(18); /* 2.5 ADC clocks = 20us */ \
  /* with only 18 us delay the voltage goes down before SH */ \
  /* delay 18us 500ns + 1 clock tics (CALL instead of RCALL) = 18.5625 us */ \
  /* + 21 clock tics delay from interrupt return, +1.3125us = 19.8750 */ \
  wdt_reset();  /* 19.9375 us  */ \
  wdt_reset();  /* 20.0000 us  */ \
  wdt_reset();  /* 20.0625 us  */ \
  wdt_reset();  /* 20.1250 us  */ \
  wdt_reset();  /* 20.1875 us  */ \
  wdt_reset()   /* 20.2500 us  */
#endif
#else
// Polling mode, big cap
#if F_CPU == 8000000UL
#define DelayBigCap() wait10us(); /* 2.5 ADC clocks = 20us */ \
  wait5us();    /*  */ \
  wait4us();  /* pulse length 19.375 us */
/* delay 19us + 3 clock tics (CALL instead of RCALL) = 19.375 us @ 8 MHz */
/* + 7 clock tics delay from while loop, +0.875us  = 20.250 */
//            wdt_reset() /* 20.375 us + */
#endif
#if F_CPU == 16000000UL
#define DelayBigCap() delayMicroseconds(20)
//    #define DelayBigCap() us500delay(19); /* 2.5 ADC clocks = 20us */ \
//            /* with only 18 us delay the voltage goes down before SH */ \
//            /* delay 19us 500ns + 1 clock tics (CALL instead of RCALL) = 19.5625 us */ \
//            /* + 7 clock tics delay from "while (ADCSRA&(1<<ADSC))" loop = 20.0000 */ \
//            wdt_reset();  /* 20.0625 us  */ \
//            wdt_reset();  /* 20.1250 us  */ \
//            wdt_reset();  /* 20.1875 us  */ \
//            wdt_reset()   /* 20.2500 us  */
#endif
#endif

/**************************************************************************/
/* Predefine the wait time for switch off the load current for small caps */
/**************************************************************************/
// SH at 2.5 ADC clocks behind start = 5 us
#ifdef ADC_Sleep_Mode
// Interrupt mode, small cap
#if F_CPU == 8000000UL
#define DelaySmallCap() wait2us();  /* with only 4 us delay the voltage goes down before SH */ \
  /* delay 2us + 1 clock tics (CALL instead of RCALL) = 2.125 us @ 8 MHz */ \
  /* + 21 clock tics delay from interrupt return, +2.625us = 4.75 */ \
  wdt_reset();  /* 4.875 us   */ \
  wdt_reset();  /* 5.000 us   */ \
  wdt_reset()   /* 5.125 us   */
#endif
#if F_CPU == 16000000UL
#define DelaySmallCap() us500delay(3);  /* with only 18 us delay the voltage goes down before SH */ \
  /* delay 3us 500ns + 1 clock tics (CALL instead of RCALL) = 3.5625 us */ \
  /* + 21 clock tics delay from interrupt return, +1.3125us = 4.875 */ \
  wdt_reset();  /* 4.9375 us  */ \
  wdt_reset();  /* 5.0000 us  */ \
  wdt_reset();  /* 5.0625 us  */ \
  wdt_reset()   /* 5.1250 us  */
#endif
#else
// Polling mode, small cap
#if F_CPU == 8000000UL
#define DelaySmallCap() wait4us();  /* with only 4 us delay the voltage goes down before SH */ \
  /* delay 4us + 1 clock tics (CALL instead of RCALL) = 4.125 us @ 8 MHz */ \
  /* + 7 clock tics delay from while loop, +0.875us  = 5.000 */ \
  wdt_reset()   /* 5.125 us   */
#endif
#if F_CPU == 16000000UL
#define DelaySmallCap() us500delay(4);  /* with only 4 us delay the voltage goes down before SH */ \
  /* delay 4us 500ns + 1 clock tics (CALL instead of RCALL) = 4.5625 us */ \
  /* + 7 clock tics delay from "while (ADCSRA&(1<<ADSC))" loop, +0.4375 = 5.0000 */ \
  wdt_reset();  /* 5.0625 us  */ \
  wdt_reset()   /* 5.1250 us  */
#endif
#endif

//=================================================================
uint16_t GetESR(uint8_t hipin, uint8_t lopin) {
#if FLASHEND > 0x1fff
  // measure the ESR value of capacitor
  unsigned int adcv[4];   // array for 4 ADC readings
  unsigned long sumvolt[4]; // array for 3 sums of ADC readings
  unsigned long cap_val_nF;
  uint16_t esrvalue;
  uint8_t HiPinR_L;   // used to switch 680 Ohm to HighPin
  uint8_t HiADC;    // used to switch Highpin directly to GND or VCC
  uint8_t LoPinR_L;   // used to switch 680 Ohm to LowPin
  uint8_t LoADC;    // used to switch Lowpin directly to GND or VCC
  uint8_t ii, jj;   // tempory values
  uint8_t StartADCmsk;    // Bit mask to start the ADC
  uint8_t SelectLowPin, SelectHighPin;
  uint8_t big_cap;
  int8_t esr0;      // used for ESR zero correction
  big_cap = 1;

  if (PartFound == PART_CAPACITOR) {
    ii = cap.cpre_max;
    cap_val_nF = cap.cval_max;

    while (ii < -9) {     // set cval to nF unit
      cap_val_nF /= 10;   // reduce value by factor ten
      ii++;     // take next decimal prefix
    }

    if (cap_val_nF < (1800 / 18)) return (0xffff); // capacity lower than 1.8 uF
    //if (cap_val_nF > (1800/18)) {

    // normal ADC-speed, ADC-Clock 8us
#ifdef ADC_Sleep_Mode
    StartADCmsk = (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | AUTO_CLOCK_DIV; // enable ADC and Interrupt
    ADCSRA = StartADCmsk;   // enable ADC and Interrupt
#else
    StartADCmsk =  (1 << ADSC) | (1 << ADEN) | (1 << ADIF) | AUTO_CLOCK_DIV; // enable and start ADC
#endif

    //} else {

    // fast ADC-speed, ADC-Clock 2us
#ifdef ADC_Sleep_Mode
    //StartADCmsk = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | FAST_CLOCK_DIV;  // enable ADC and Interrupt
    //ADCSRA = StartADCmsk;   // enable ADC and Interrupt
    //SMCR = (1 << SM0) | (1 <<SE); // set ADC Noise Reduction and Sleep Enable
#else
    //StartADCmsk =  (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | FAST_CLOCK_DIV;  // enable and start ADC
#endif

    //big_cap = 0;
    //}
  }

  LoADC = pgm_read_byte(&PinADCtab[lopin]) | TXD_MSK;
  HiADC = pgm_read_byte(&PinADCtab[hipin]) | TXD_MSK;
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);   // R_L mask for LowPin R_L load
  HiPinR_L = pgm_read_byte(&PinRLtab[hipin]);   // R_L mask for HighPin R_L load

#if PROCESSOR_TYP == 1280
  // ATmega640/1280/2560 1.1V Reference with REFS0=0
  SelectLowPin = (lopin | (1 << REFS1) | (0 << REFS0)); // switch ADC to LowPin, Internal Ref.
  SelectHighPin = (hipin | (1 << REFS1) | (0 << REFS0)); // switch ADC to HighPin, Internal Ref.
#else
  SelectLowPin = (lopin | (1 << REFS1) | (1 << REFS0)); // switch ADC to LowPin, Internal Ref.
  SelectHighPin = (hipin | (1 << REFS1) | (1 << REFS0)); // switch ADC to HighPin, Internal Ref.
#endif

  // Measurement of ESR of capacitors AC Mode
  sumvolt[0] = 1;   // set sum of LowPin voltage to 1 to prevent divide by zero
  sumvolt[2] = 1;   // clear sum of HighPin voltage with current
  // offset is about (x*10*200)/34000 in 0.01 Ohm units
  sumvolt[1] = 0;   // clear sum of HighPin voltage without current
  sumvolt[3] = 0;   // clear sum of HighPin voltage without current
  EntladePins();    // discharge capacitor
  ADC_PORT = TXD_VAL;   // switch ADC-Port to GND
  ADMUX = SelectLowPin;   // set Mux input and Voltage Reference to internal 1.1V

#ifdef NO_AREF_CAP
  wait100us();    // time for voltage stabilization
#else
  wait_about10ms();   // time for voltage stabilization with 100nF
#endif

  // start voltage must be negativ
  ADC_DDR = HiADC;      // switch High Pin to GND
  R_PORT = LoPinR_L;      // switch R-Port to VCC
  R_DDR = LoPinR_L;     // switch R_L port for HighPin to output (VCC)
  wait10us();
  wait2us();
  R_DDR = 0;        // switch off current
  R_PORT = 0;
  StartADCwait();     // set ADCSRA Interrupt Mode, sleep

  // Measurement frequency is given by sum of ADC-Reads < 680 Hz for normal ADC speed.
  // For fast ADC mode the frequency is below 2720 Hz (used for capacity value below 3.6 uF).
  // ADC Sample and Hold (SH) is done 1.5 ADC clock number after real start of conversion.
  // Real ADC-conversion is started with the next ADC-Clock (125kHz) after setting the ADSC bit.

  for (ii = 0; ii < MAX_CNT; ii++) {
    ADC_DDR = LoADC;      // switch Low-Pin to output (GND)
    R_PORT = LoPinR_L;      // switch R-Port to VCC
    R_DDR = LoPinR_L;     // switch R_L port for LowPin to output (VCC)
    ADMUX = SelectLowPin;
    StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    adcv[0] = ADCW;     // Voltage LowPin with current
    ADMUX = SelectHighPin;

    //if (big_cap != 0) {

    StartADCwait();     // ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV;
    ADCSRA = (1 << ADSC) | (1 << ADEN) | (1 << ADIF) | AUTO_CLOCK_DIV; // enable ADC and start with ADSC
    wait4us();
    R_PORT = HiPinR_L;      // switch R-Port to VCC
    R_DDR = HiPinR_L;     // switch R_L port for HighPin to output (VCC)
    DelayBigCap();      // wait predefined time

    //} else {
    //  StartADCwait();     // ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV;
    //  R_PORT = HiPinR_L;    // switch R-Port to VCC
    //  R_DDR = HiPinR_L;   // switch R_L port for HighPin to output (VCC)
    //  ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | FAST_CLOCK_DIV;  // enable ADC and start with ADSC
    //                      // SH at 2.5 ADC clocks behind start = 5 us
    //  DelaySmallCap();    // wait predefined time
    //}

    R_DDR = 0;        // switch current off,  SH is 1.5 ADC clock behind real start
    R_PORT = 0;
    while (ADCSRA & (1 << ADSC)); // wait for conversion finished
    adcv[1] = ADCW;     // Voltage HighPin with current

#ifdef ADC_Sleep_Mode
    ADCSRA = StartADCmsk;   // enable ADC and Interrupt
#endif

    wdt_reset();

    // ******** Reverse direction, connect High side with GND ********
    ADC_DDR = HiADC;      // switch High Pin to GND
    R_PORT = HiPinR_L;    // switch R-Port to VCC
    R_DDR = HiPinR_L;     // switch R_L port for HighPin to output (VCC)
    wdt_reset();
    ADMUX = SelectHighPin;
    StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    adcv[2] = ADCW;     // Voltage HighPin with current
    ADMUX = SelectLowPin;

    //if (big_cap != 0) {

    StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    ADCSRA = (1 << ADSC) | (1 << ADEN) | (1 << ADIF) | AUTO_CLOCK_DIV; // enable ADC and start with ADSC
    wait4us();
    R_PORT = LoPinR_L;
    R_DDR = LoPinR_L;     // switch LowPin with 680 Ohm to VCC
    DelayBigCap();        // wait predefined time

    //} else {
    //  StartADCwait();     // set ADCSRA Interrupt Mode, sleep
    //  R_PORT = LoPinR_L;
    //  R_DDR = LoPinR_L;   // switch LowPin with 680 Ohm to VCC
    //  ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | FAST_CLOCK_DIV;  // enable ADC and start with ADSC
    //                      // 2.5 ADC clocks = 5 us
    //   DelaySmallCap();   // wait predefined time
    //}

    R_DDR = 0;        // switch current off
    R_PORT = 0;

    while (ADCSRA & (1 << ADSC)); // wait for conversion finished
    adcv[3] = ADCW;     // Voltage LowPin with current

#ifdef ADC_Sleep_Mode
    ADCSRA = StartADCmsk;   // enable ADC and Interrupt
#endif

    sumvolt[0] += adcv[0];    // add sum of both LowPin voltages with current
    sumvolt[1] += adcv[1];    // add  HighPin voltages with current
    sumvolt[2] += adcv[2];    // add  LowPin voltages with current
    sumvolt[3] += adcv[3];    // add  HighPin voltages with current
  } // end for

  sumvolt[0] += sumvolt[2];

#ifdef ESR_DEBUG
  lcd_testpin(hipin);
  lcd_testpin(lopin);
  lcd_data(' ');
  DisplayValue(sumvolt[0], 0, 'L', 4); // LowPin 1
  lcd_line3();
  DisplayValue(sumvolt[1], 0, 'h', 4); // HighPin 1
  lcd_data(' ');
  DisplayValue(sumvolt[3], 0, 'H', 4); // LowPin 2
  lcd_line4();
#endif

  if ((sumvolt[1] + sumvolt[3]) > sumvolt[0]) {
    sumvolt[2] = (sumvolt[1] + sumvolt[3]) - sumvolt[0];  // difference HighPin - LowPin Voltage with current
  } else {
    sumvolt[2] = 0;
  }

  if (PartFound == PART_CAPACITOR) {
    sumvolt[2] -= (1745098UL * MAX_CNT) / (cap_val_nF * (cap_val_nF + 19));
  }

#ifdef ESR_DEBUG
  DisplayValue(sumvolt[2], 0, 'd', 4); // HighPin - LowPin
  lcd_data(' ');
#endif

  esrvalue = (sumvolt[2] * 10 * (unsigned long)RRpinMI) / (sumvolt[0] + sumvolt[2]);
  esrvalue += esrvalue / 14;          // esrvalue + 7%
  esr0 = (int8_t)pgm_read_byte(&EE_ESR_ZEROtab[hipin + lopin]);

  if (esrvalue > esr0) {
    esrvalue -= esr0;
  } else {
    esrvalue = 0;
  }

#ifdef ADC_Sleep_Mode
  SMCR = (0 << SM0) | (0 << SE);  // clear ADC Noise Reduction and Sleep Enable
#endif

  return (esrvalue);
#else
  return (0);
#endif
}

void us500delay(unsigned int us)  // = delayMicroseconds(us) + 500ns
{
#if F_CPU >= 20000000L
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop");              // just waiting 2 cycles
  if (--us == 0) return;
  us = (us << 2) + us;   // x5 us

#elif F_CPU >= 16000000L
  if (--us == 0) return;
  us <<= 2;
#else
  if (--us == 0) return;
  if (--us == 0) return;
  us <<= 1;
#endif
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t"            // 2 cycles
    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
  );
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// new code by K.-H. Kubbeler
// ca   = Pin number (0-2) of the LowPin
// cb   = Pin number (0-2) of the HighPin

//=================================================================
void GetVloss() {
#if FLASHEND > 0x1fff
  // measure voltage drop after load pulse
  unsigned int tmpint;
  unsigned int adcv[4];

  union t_combi {
    unsigned long dw;     // capacity value  in 100nF units
    uint16_t w[2];
  } lval;

  uint8_t ii;
  uint8_t HiPinR_L;
  uint8_t LoADC;

  if (cap.v_loss > 0) return;   // Voltage loss is already known

  LoADC = pgm_read_byte(&PinADCtab[cap.ca]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[cap.cb]);    // R_L mask for HighPin R_L load

  EntladePins();      // discharge capacitor
  ADC_PORT = TXD_VAL;     // switch ADC-Port to GND
  R_PORT = 0;       // switch R-Port to GND
  ADC_DDR = LoADC;      // switch Low-Pin to output (GND)
  R_DDR = HiPinR_L;     // switch R_L port for HighPin to output (GND)
  adcv[0] = ReadADC(cap.cb);    // voltage before any load

  // ******** should adcv[0] be measured without current???
  if (cap.cpre_max > -9) return;  // too much capacity

  lval.dw = cap.cval_max;
  //for (ii=cap.cpre_max+12;ii<5;ii++) {
  for (ii = cap.cpre_max + 12; ii < 4; ii++) {
    lval.dw = (lval.dw + 5) / 10;
  }

  //if ((lval.dw == 0) || (lval.dw > 500)) {
  if ((lval.dw == 0) || (lval.dw > 5000)) {
    // capacity more than 50uF, Voltage loss is already measured
    return;
  }

  R_PORT = HiPinR_L;  // R_L to 1 (VCC)
  R_DDR = HiPinR_L;   // switch Pin to output, across R to GND or VCC

  for (tmpint = 0; tmpint < lval.w[0]; tmpint += 2) {
    //wait50us();   // wait exactly 50us
    wait5us();      // wait exactly 5us
  }

  R_DDR = 0;      // switch back to input
  R_PORT = 0;     // no Pull up
  //wait10us();     // wait a little time
  wdt_reset();

  // read voltage without current
  ADCconfig.Samples = 5;  // set ADC to only 5 samples
  adcv[2] = ReadADC(cap.cb);
  if (adcv[2] > adcv[0]) {
    adcv[2] -= adcv[0];   // difference to beginning voltage
  } else {
    adcv[2] = 0;    // voltage is lower or same as beginning voltage
  }

  // wait 2x the time which was required for loading
  for (tmpint = 0; tmpint < lval.w[0]; tmpint++) {
    //wait50us();
    wait5us();
  }

  adcv[3] = ReadADC(cap.cb);    // read voltage again, is discharged only a little bit ?
  ADCconfig.Samples = ANZ_MESS;   // set ADC back to configured No. of samples
  wdt_reset();

  if (adcv[3] > adcv[0]) {
    adcv[3] -= adcv[0];     // difference to beginning voltage
  } else {
    adcv[3] = 0;      // voltage is lower or same as beginning voltage
  }

  if (adcv[2] > adcv[3]) {
    // build difference to load voltage
    adcv[1] = adcv[2] - adcv[3];  // lost voltage during load time wait
  } else {
    adcv[1] = 0;      // no lost voltage
  }

  // compute voltage drop as part from loaded voltage
  if (adcv[1] > 0) {
    // there is any voltage drop (adcv[1]) !
    // adcv[2] is the loaded voltage.
    cap.v_loss = (unsigned long)(adcv[1] * 500UL) / adcv[2];
  }

#if 0
  lcd_line3();
  DisplayValue(adcv[2], 0, ' ', 4);
  DisplayValue(adcv[1], 0, ' ', 4);
  lcd_line4();
  DisplayValue(lval.w[0], 0, 'x', 4);
#endif

  // discharge capacitor again
  EntladePins();    // discharge capacitors
  // ready
  // switch all ports to input

  ADC_DDR =  TXD_MSK;   // switch all ADC ports to input
  ADC_PORT = TXD_VAL;   // switch all ADC outputs to GND, no pull up
  R_DDR = 0;      // switch all resistor ports to input
  R_PORT = 0;       // switch all resistor outputs to GND, no pull up

#endif
  return;
}  // end GetVloss()

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

void Calibrate_UR(void) {
  // get reference voltage, calibrate VCC with external 2.5V and
  // get the port output resistance

#ifdef AUTO_CAL
  uint16_t sum_rm;  // sum of 3 Pin voltages with 680 Ohm load
  uint16_t sum_rp;  // sum of 3 Pin voltages with 680 Ohm load
  uint16_t u680;  // 3 * (Voltage at 680 Ohm)
#endif

  //--------------------------------------------
  ADCconfig.U_AVCC = U_VCC;     // set initial VCC Voltage
  ADCconfig.Samples = 190;  // set number of ADC reads near to maximum

#if FLASHEND > 0x1fff
  ADC_PORT = TXD_VAL;                 // switch to 0V
  ADC_DDR = (1 << TPREF) | TXD_MSK;   // switch pin with 2.5V reference to GND
  wait1ms();
  ADC_DDR =  TXD_MSK;       // switch pin with reference back to input
  trans.uBE[1] = W5msReadADC(TPREF);  // read voltage of 2.5V precision reference

  if ((trans.uBE[1] > 2250) && (trans.uBE[1] < 2750)) {
    // precision voltage reference connected, update U_AVCC
    WithReference = 1;
    ADCconfig.U_AVCC = (unsigned long)((unsigned long)ADCconfig.U_AVCC * 2495) / trans.uBE[1];
  }
#endif

#ifdef WITH_AUTO_REF
  (void) ReadADC(MUX_INT_REF);  // read reference voltage
  ref_mv = W5msReadADC(MUX_INT_REF);  // read reference voltage
  RefVoltage();     // compute RHmultip = f(reference voltage)
#else
  ref_mv = DEFAULT_BAND_GAP;      // set to default Reference Voltage
#endif

  ADCconfig.U_Bandgap = ADC_internal_reference; // set internal reference voltage for ADC

  //--------------------------------------------

#ifdef AUTO_CAL
  // measurement of internal resistance of the ADC port outputs switched to GND
  ADC_DDR = 1 << TP1 | TXD_MSK; // ADC-Pin  1 to output 0V
  R_PORT = 1 << (TP1 * 2); // R_L-PORT 1 to VCC
  R_DDR = 1 << (TP1 * 2); // Pin 1 to output and over R_L to VCC
  sum_rm = W5msReadADC(TP1);

  ADC_DDR = 1 << TP2 | TXD_MSK; // ADC-Pin 2 to output 0V
  R_PORT =  1 << (TP2 * 2); // R_L-PORT 2 to VCC
  R_DDR = 1 << (TP2 * 2); // Pin 2 to output and over R_L to VCC
  sum_rm += W5msReadADC(TP2);

  ADC_DDR = 1 << TP3 | TXD_MSK; // ADC-Pin 3 to output 0V
  R_PORT =  1 << (TP3 * 2); // R_L-PORT 3 to VCC
  R_DDR = 1 << (TP3 * 2); // Pin 3 to output and over R_L to VCC
  sum_rm += W5msReadADC(TP3);   // add all three values

  // measurement of internal resistance of the ADC port output switched to VCC
  R_PORT = 0;       // R-Ports to GND
  ADC_PORT = 1 << TP1 | TXD_VAL; // ADC-Port 1 to VCC
  ADC_DDR = 1 << TP1 | TXD_MSK; // ADC-Pin  1 to output 0V
  R_DDR = 1 << (TP1 * 2); // Pin 1 to output and over R_L to GND
  sum_rp = ADCconfig.U_AVCC - W5msReadADC(TP1);

  ADC_PORT = 1 << TP2 | TXD_VAL; // ADC-Port 2 to VCC
  ADC_DDR = 1 << TP2 | TXD_MSK; // ADC-Pin  2 to output 0V
  R_DDR = 1 << (TP2 * 2); // Pin 2 to output and over R_L to GND
  sum_rp += ADCconfig.U_AVCC - W5msReadADC(TP2);

  ADC_PORT = 1 << TP3 | TXD_VAL; // ADC-Port 3 to VCC
  ADC_DDR = 1 << TP3 | TXD_MSK; // ADC-Pin  3 to output 0V
  R_DDR = 1 << (TP3 * 2); // Pin 3 to output and over R_L to GND
  sum_rp += ADCconfig.U_AVCC - W5msReadADC(TP3);

  u680 = ((ADCconfig.U_AVCC * 3) - sum_rm - sum_rp);  // three times the voltage at the 680 Ohm
  pin_rmi = (unsigned long)((unsigned long)sum_rm * (unsigned long)R_L_VAL) / (unsigned long)u680;
  //adcmv[2] = pin_rm;  // for last output in row 2
  pin_rpl = (unsigned long)((unsigned long)sum_rp * (unsigned long)R_L_VAL) / (unsigned long)u680;
  resis680pl = pin_rpl + R_L_VAL;
  resis680mi = pin_rmi + R_L_VAL;
#endif

  ADCconfig.Samples = ANZ_MESS;   // set to configured number of ADC samples
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// Interfacing a HD44780 compatible LCD with 4-Bit-Interface mode

#ifdef STRIP_GRID_BOARD
#warning "strip-grid-board layout selected!"
#endif

void lcd_set_cursor(uint8_t row, uint8_t col)
{
#ifdef LCD1602
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row >= 2 ) {
    row = 1;
  }
  lcd.command(CMD_SetDDRAMAddress | (col + row_offsets[row]));
#endif

#ifdef NOK5110
  lcd.setCursor(6 * col, 10 * row);
#endif

#ifdef OLED096
  display.setCursor(6 * col, 10 * row);
#endif

  uart_newline();
}

void lcd_string(char *data) {
  while (*data) {
    lcd_data(*data);
    data++;
  }
}

void lcd_pgm_string(const unsigned char *data) {
  unsigned char cc;
  while (1) {
    cc = pgm_read_byte(data);
    if ((cc == 0) || (cc == 128)) return;
    lcd_data(cc);
    data++;
  }
}

void lcd_pgm_custom_char(uint8_t location, const unsigned char *chardata) {
#ifdef LCD1602
  location &= 0x7;
  lcd.command(CMD_SetCGRAMAddress | (location << 3));
  for (uint8_t i = 0; i < 8; i++) {
    lcd.write(pgm_read_byte(chardata));
    chardata++;
  }
#endif
}

// sends numeric character (Pin Number) to the LCD
// from binary 0 we send ASCII 1
void lcd_testpin(unsigned char temp) {
  lcd_data(temp + '1');
}

// send space character to LCD
void lcd_space(void) {
  lcd_data(' ');
}

void lcd_fix_string(const unsigned char *data) {
  unsigned char cc;
  while (1) {
    cc = MEM_read_byte(data);
    if ((cc == 0) || (cc == 128)) return;
    lcd_data(cc);
    data++;
  }
}

// sends data byte to the LCD
void lcd_data(unsigned char temp1) {
#ifdef LCD1602
  lcd.write(temp1);
#endif

#ifdef NOK5110
  lcd.write(temp1);
#endif

#ifdef OLED096
  display.write(temp1);
#endif

  switch (temp1) {
    case LCD_CHAR_DIODE1: {
        uart_putc('>'); uart_putc('|'); break;
      }
    case LCD_CHAR_DIODE2: {
        uart_putc('|'); uart_putc('<'); break;
      }
    case LCD_CHAR_CAP: {
        uart_putc('|'); uart_putc('|'); break;
      }
    case LCD_CHAR_RESIS1: {
        uart_putc('['); uart_putc('='); break;
      }
    case LCD_CHAR_RESIS2: {
        uart_putc(']'); break;
      }
    case LCD_CHAR_U: {    // micro
        uart_putc('u');   // "u"
        break;
      }
    case LCD_CHAR_OMEGA: {  // omega
        uart_putc('o');     // "ohm"
        uart_putc('h');
        uart_putc('m'); break;
      }
    default: {
        uart_putc(temp1);
      }
  }
}

void lcd_clear(void) {
#ifdef LCD1602
  lcd.clear();
#endif

#ifdef NOK5110
  lcd.clearDisplay();
#endif

#ifdef OLED096
  display.clearDisplay();
#endif

  uart_newline();
}

void uart_putc(uint8_t data) {
  Serial.write(data);
  delay(2);
}
