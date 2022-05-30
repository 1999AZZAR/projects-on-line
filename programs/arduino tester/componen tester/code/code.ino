/*                      \\\|///
                      \\  - -  //
                       (  @ @  )
  /--------------------oOOo-(_)-oOOo---------------------\
  |                                                      |
  |                                                      |
  |    Transistor Tester for Arduino (version 1.08a)     |
  |                                                      |
  |  based on code: Karl-Heinz Kubbeler (version 1.08k)  |
  |                                                      |
  |                                                      |
  |                            Oooo                      |
  \--------------------oooO----(   )---------------------/
                   (   )    ) /
                      \ (    (_/
                       \_)                            */

#include "config.h"

void setup()
{
  Serial.begin(9600);

  pinMode(TestKeyPin, INPUT);

#ifdef LCD1602
#ifdef LCD_I2C
  lcd.begin();
#else
  lcd.begin(16, 2);
#endif

  lcd_pgm_custom_char(LCD_CHAR_DIODE1, DiodeIcon1);  // Custom-Character Diode symbol >|
  lcd_pgm_custom_char(LCD_CHAR_DIODE2, DiodeIcon2);  // Custom-Character Diode symbol |<
  lcd_pgm_custom_char(LCD_CHAR_CAP,    CapIcon);     // Custom-Character Capacitor symbol ||
  lcd_pgm_custom_char(LCD_CHAR_RESIS1, ResIcon1);    // Custom-Character Resistor symbol [
  lcd_pgm_custom_char(LCD_CHAR_RESIS2, ResIcon2);    // Custom-Character Resistor symbol ]
  lcd_pgm_custom_char(LCD_CHAR_OMEGA,  OmegaIcon);   // load Omega as Custom-Character
  lcd_pgm_custom_char(LCD_CHAR_U,      MicroIcon);   // load Micro as Custom-Character
  lcd.home();

  lcd_string("Component Tester");
  lcd_set_cursor(1, 0);
  lcd_string("forArduino 1.08a");
#endif

#ifdef NOK5110
  lcd.begin();
  lcd.cp437(true);
  lcd.setContrast(40);
  lcd.clearDisplay();
#endif

#ifdef OLED096
#ifdef OLED_I2C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
#else
  display.begin(SSD1306_SWITCHCAPVCC);
#endif

  display.cp437(true);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
#endif

#if defined(NOK5110) || defined(OLED096)
  lcd_string("Component");
  lcd_set_cursor(1, 0);
  lcd_string("Tester");
  lcd_set_cursor(2, 0);
  lcd_string("for Arduino");
  lcd_set_cursor(3, 0);
  lcd_string("1.08.004");
#endif

  //ON_DDR = 0;
  //ON_PORT = 0;

  /*
    // switch on
    ON_DDR = (1<<ON_PIN);     // switch to output
    #ifdef PULLUP_DISABLE
      ON_PORT = (1<<ON_PIN);    // switch power on
    #else
      ON_PORT = (1<<ON_PIN)|(1<<RST_PIN);   // switch power on , enable internal Pullup for Start-Pin
    #endif
  */

  // ADC-Init
  ADCSRA = (1 << ADEN) | AUTO_CLOCK_DIV; // prescaler=8 or 64 (if 8Mhz clock)

#ifdef __AVR_ATmega8__
  //#define WDRF_HOME MCU_STATUS_REG
#define WDRF_HOME MCUCSR
#else
#define WDRF_HOME MCUSR
#endif

  /*
    tmp = (WDRF_HOME & (1<<WDRF));  // save Watch Dog Flag
    WDRF_HOME &= ~(1<<WDRF);    // reset Watch Dog flag
    wdt_disable();      // disable Watch Dog
  */

  /*
    #ifndef INHIBIT_SLEEP_MODE
      // switch off unused Parts
      PRR = (1<<PRTWI) | (1<<PRTIM0) | (1<<PRSPI) | (1<<PRUSART0);
      DIDR0 = (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D);
      TCCR2A = (0<<WGM21) | (0<<WGM20);     // Counter 2 normal mode

      #if F_CPU <= 1000000UL
        TCCR2B = (1<<CS22) | (0<<CS21) | (1<<CS20); // prescaler 128, 128us @ 1MHz
        #define T2_PERIOD 128
      #endif
      #if F_CPU == 2000000UL
        TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20); // prescaler 256, 128us @ 2MHz
        #define T2_PERIOD 128
      #endif
      #if F_CPU == 4000000UL
        TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20); // prescaler 256, 64us @ 2MHz
        #define T2_PERIOD 64
      #endif
      #if F_CPU >= 8000000UL
        TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // prescaler 1024, 128us @ 8MHz, 64us @ 16MHz
        #define T2_PERIOD (1024 / (F_CPU / 1000000UL)); // set to 128 or 64 us
      #endif

      sei();  // enable interrupts
    #endif
  */

#define T2_PERIOD (1024 / (F_CPU / 1000000UL)); // set to 128 or 64 us 

  //ADC_PORT = TXD_VAL;
  //ADC_DDR = TXD_MSK;

  if (tmp) {
    // check if Watchdog-Event
    // this happens, if the Watchdog is not reset for 2s
    // can happen, if any loop in the Program doen't finish.
    lcd_line1();
    lcd_fix_string(TestTimedOut); // Output Timeout
    wait_about3s();     // wait for 3 s
    //ON_PORT = 0;      // shut off!
    //ON_DDR = (1<<ON_PIN);   // switch to GND
    //return;
  }

#ifdef PULLUP_DISABLE
#ifdef __AVR_ATmega8__
  SFIOR = (1 << PUD); // disable Pull-Up Resistors mega8
#else
  MCUCR = (1 << PUD); // disable Pull-Up Resistors mega168 family
#endif
#endif

  //DIDR0 = 0x3f;   // disable all Input register of ADC

  /*
    #if POWER_OFF+0 > 1
      // tester display time selection
      display_time = OFF_WAIT_TIME;   // LONG_WAIT_TIME for single mode, else SHORT_WAIT_TIME
      if (!(ON_PIN_REG & (1<<RST_PIN))) {
        // if power button is pressed ...
        wait_about300ms();      // wait to catch a long key press
        if (!(ON_PIN_REG & (1<<RST_PIN))) {
          // check if power button is still pressed
          display_time = LONG_WAIT_TIME;    // ... set long time display anyway
        }
      }
    #else
      #define display_time OFF_WAIT_TIME
    #endif
  */

#define display_time OFF_WAIT_TIME

  empty_count = 0;
  mess_count = 0;
}

void loop()
{
  // Entry: if start key is pressed before shut down
start:

#ifdef NOK5110
  lcd.display();
#endif

#ifdef OLED096
  display.display();
#endif

  TestKey = 1;
  while (TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  while (!TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  lcd_clear();
  delay(100);

  PartFound = PART_NONE;  // no part found
  NumOfDiodes = 0;    // Number of diodes = 0
  PartReady = 0;
  PartMode = 0;
  WithReference = 0;    // no precision reference voltage
  ADC_DDR = TXD_MSK;    // activate Software-UART
  ResistorsFound = 0;   // no resistors found
  cap.ca = 0;
  cap.cb = 0;

#ifdef WITH_UART
  uart_newline();   // start of new measurement
#endif

  ADCconfig.RefFlag = 0;
  Calibrate_UR();   // get Ref Voltages and Pin resistance
  lcd_line1();    // 1 row

  ADCconfig.U_Bandgap = ADC_internal_reference;  // set internal reference voltage for ADC

#ifdef BAT_CHECK
  // Battery check is selected
  ReadADC(TPBAT);     // Dummy-Readout
  trans.uBE[0] = W5msReadADC(TPBAT);  // with 5V reference
  lcd_fix_string(Bat_str);    // output: "Bat. "

#ifdef BAT_OUT
  // display Battery voltage
  // The divisor to get the voltage in 0.01V units is ((10*33)/133) witch is about 2.4812
  // A good result can be get with multiply by 4 and divide by 10 (about 0.75%).
  //cap.cval = (trans.uBE[0]*4)/10+((BAT_OUT+5)/10); // usually output only 2 digits
  //DisplayValue(cap.cval,-2,'V',2);    // Display 2 Digits of this 10mV units
  cap.cval = (trans.uBE[0] * 4) + BAT_OUT; // usually output only 2 digits
  DisplayValue(cap.cval, -3, 'V', 2);   // Display 2 Digits of this 10mV units
  lcd_space();
#endif

#if (BAT_POOR > 12000)
#warning "Battery POOR level is set very high!"
#endif
#if (BAT_POOR < 2500)
#warning "Battery POOR level is set very low!"
#endif

#if (BAT_POOR > 5300)
  // use .8 V difference to Warn-Level
#define WARN_LEVEL (((unsigned long)(BAT_POOR+800)*(unsigned long)33)/133)
#elif (BAT_POOR > 3249)
  // less than 5.4 V only .4V difference to Warn-Level
#define WARN_LEVEL (((unsigned long)(BAT_POOR+400)*(unsigned long)33)/133)
#elif (BAT_POOR > 1299)
  // less than 2.9 V only .2V difference to Warn-Level
#define WARN_LEVEL (((unsigned long)(BAT_POOR+200)*(unsigned long)33)/133)
#else
  // less than 1.3 V only .1V difference to Warn-Level
#define WARN_LEVEL (((unsigned long)(BAT_POOR+100)*(unsigned long)33)/133)
#endif

#define POOR_LEVEL (((unsigned long)(BAT_POOR)*(unsigned long)33)/133)

  // check the battery voltage
  if (trans.uBE[0] <  WARN_LEVEL) {

    // Vcc < 7,3V; show Warning
    if (trans.uBE[0] < POOR_LEVEL) {
      // Vcc <6,3V; no proper operation is possible
      lcd_fix_string(BatEmpty); // Battery empty!
      wait_about2s();
      PORTD = 0;      // switch power off
      return;
    }

    lcd_fix_string(BatWeak);    // Battery weak
  } else {                            // Battery-voltage OK
    lcd_fix_string(OK_str);     // "OK"
  }

#else
  lcd_fix2_string(VERSION_str); // if no Battery check, Version .. in row 1
#endif

#ifdef WDT_enabled
  //wdt_enable(WDTO_2S);    // Watchdog on
#endif

  //wait_about1s();     // add more time for reading batterie voltage

  // begin tests

#ifdef AUTO_RH
  RefVoltage();     // compute RHmultip = f(reference voltage)
#endif

#if FLASHEND > 0x1fff
  if (WithReference) {
    // 2.5V precision reference is checked OK
    if ((mess_count == 0) && (empty_count == 0)) {
      // display VCC= only first time
      lcd_line2();
      lcd_fix_string(VCC_str);      // VCC=
      DisplayValue(ADCconfig.U_AVCC, -3, 'V', 3); // Display 3 Digits of this mV units
      //lcd_space();
      //DisplayValue(RRpinMI,-1,LCD_CHAR_OMEGA,4);
      wait_about1s();
    }
  }
#endif

#ifdef WITH_VEXT
  // show the external voltage
  while (!(ON_PIN_REG & (1 << RST_PIN))) {
    lcd_line2();
    lcd_clear_line();
    lcd_line2();
    lcd_fix_string(Vext_str);     // Vext=
    ADC_DDR = 0;        // deactivate Software-UART
    trans.uBE[1] = W5msReadADC(TPext);  // read external voltage
    ADC_DDR = TXD_MSK;      // activate Software-UART

#ifdef WITH_UART
    uart_newline();   // start of new measurement
#endif

    DisplayValue(trans.uBE[1] * 10, -3, 'V', 3); // Display 3 Digits of this mV units
    wait_about300ms();
  }
#endif

  lcd_line2();      // LCD position row 2, column 1
  lcd_fix_string(TestRunning);  // String: testing...

#ifndef DebugOut
  lcd_line2();    // LCD position row 2, column 1
#endif

#ifdef NOK5110
  lcd.display();
#endif

#ifdef OLED096
  display.display();
  display.setCursor(0, 0);
#endif

  delay(5);

  EntladePins();    // discharge all capacitors!

  if (PartFound == PART_CELL) {
    lcd_clear();
    lcd_fix_string(Cell_str); // display "Cell!"
    goto end2;
  }

#ifdef CHECK_CALL
  AutoCheck();    // check, if selftest should be done
#endif

  // check all 6 combinations for the 3 pins
  //       High  Low  Tri
  CheckPins(TP1, TP2, TP3);
  CheckPins(TP2, TP1, TP3);
  CheckPins(TP1, TP3, TP2);
  CheckPins(TP3, TP1, TP2);
  CheckPins(TP2, TP3, TP1);
  CheckPins(TP3, TP2, TP1);

  // separate check if is is a capacitor
  if (((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) ) {
    EntladePins();    // discharge capacities
    // measurement of capacities in all 3 combinations
    cap.cval_max = 0;   // set max to zero
    cap.cpre_max = -12;   // set max to pF unit

    ReadCapacity(TP3, TP1);
    ReadCapacity(TP3, TP2);
    ReadCapacity(TP2, TP1);

#if FLASHEND > 0x1fff
    ReadInductance();   // measure inductance
#endif
  }

  // All checks are done, output result to display
  lcd_clear();

  if (PartFound == PART_DIODE) {
    if (NumOfDiodes == 1) {   // single Diode
      //lcd_fix_string(Diode);    // "Diode: "

#if FLASHEND > 0x1fff
      // enough memory to sort the pins

#if EBC_STYLE == 321
      // the higher test pin number is left side
      if (diodes[0].Anode > diodes[0].Cathode) {
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
      } else {
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[0].Anode);
      }
#else
      // the higher test pin number is right side
      if (diodes[0].Anode < diodes[0].Cathode) {
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
      } else {
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[0].Anode);
      }
#endif

#else
      // too less memory to sort the pins
      lcd_testpin(diodes[0].Anode);
      lcd_fix_string(AnKat);          // "->|-"
      lcd_testpin(diodes[0].Cathode);
#endif

#if FLASHEND > 0x1fff
      GetIr(diodes[0].Cathode, diodes[0].Anode);
#endif

      UfOutput(0x70);

#if defined(NOK5110) || defined(OLED096)
      lcd_line3();
#endif

      // load current of capacity is (5V-1.1V)/(470000 Ohm) = 8298nA
      lcd_fix_string(GateCap_str);      // "C="
      ReadCapacity(diodes[0].Cathode, diodes[0].Anode); // Capacity opposite flow direction
      DisplayValue(cap.cval, cap.cpre, 'F', 3);
      goto end;

    } else if (NumOfDiodes == 2) {  // double diode
      lcd_data('2');
      lcd_fix_string(Diodes);   // "diodes "

      if (diodes[0].Anode == diodes[1].Anode) { //Common Anode
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;

      } else if (diodes[0].Cathode == diodes[1].Cathode) { //Common Cathode
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);          // "-|<-"
        lcd_testpin(diodes[1].Anode);
        UfOutput(0x01);
        goto end;

      } else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
        // Antiparallel
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(AnKat);          // "->|-"
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;
      }

    } else if (NumOfDiodes == 3) {
      // Serial of 2 Diodes; was detected as 3 Diodes
      trans.b = 3;
      trans.c = 3;

      // Check for any constallation of 2 serial diodes:
      // Only once the pin No of anyone Cathode is identical of another anode.
      // two diodes in series is additionally detected as third big diode.

      if (diodes[0].Cathode == diodes[1].Anode) {
        trans.b = 0;
        trans.c = 1;
      }
      if (diodes[0].Anode == diodes[1].Cathode) {
        trans.b = 1;
        trans.c = 0;
      }
      if (diodes[0].Cathode == diodes[2].Anode) {
        trans.b = 0;
        trans.c = 2;
      }
      if (diodes[0].Anode == diodes[2].Cathode) {
        trans.b = 2;
        trans.c = 0;
      }
      if (diodes[1].Cathode == diodes[2].Anode) {
        trans.b = 1;
        trans.c = 2;
      }
      if (diodes[1].Anode == diodes[2].Cathode) {
        trans.b = 2;
        trans.c = 1;
      }

#if DebugOut == 4
      lcd_line3();
      lcd_testpin(diodes[0].Anode);
      lcd_data(':');
      lcd_testpin(diodes[0].Cathode);
      lcd_space();
      lcd_string(utoa(diodes[0].Voltage, outval, 10));
      lcd_space();
      lcd_testpin(diodes[1].Anode);
      lcd_data(':');
      lcd_testpin(diodes[1].Cathode);
      lcd_space();
      lcd_string(utoa(diodes[1].Voltage, outval, 10));
      lcd_line4();
      lcd_testpin(diodes[2].Anode);
      lcd_data(':');
      lcd_testpin(diodes[2].Cathode);
      lcd_space();
      lcd_string(utoa(diodes[2].Voltage, outval, 10));
      lcd_line1();
#endif

      if ((trans.b < 3) && (trans.c < 3)) {
        lcd_data('3');
        lcd_fix_string(Diodes);     // "Diodes "
        lcd_testpin(diodes[trans.b].Anode);
        lcd_fix_string(AnKat);      // "->|-"
        lcd_testpin(diodes[trans.b].Cathode);
        lcd_fix_string(AnKat);      // "->|-"
        lcd_testpin(diodes[trans.c].Cathode);
        UfOutput( (trans.b << 4) | trans.c);
        goto end;
      }
    }
    // end (PartFound == PART_DIODE)

  } else if (PartFound == PART_TRANSISTOR) {
    if (PartReady != 0) {
      if ((trans.hfe[0] > trans.hfe[1])) {
        // if the amplification factor was higher at first testr: swap C and E !
        tmp = trans.c;
        trans.c = trans.e;
        trans.e = tmp;
      } else {
        trans.hfe[0] = trans.hfe[1];
        trans.uBE[0] = trans.uBE[1];
      }
    }

    if (PartMode == PART_MODE_NPN) {
      lcd_fix_string(NPN_str);    // "NPN "
    } else {
      lcd_fix_string(PNP_str);    // "PNP "
    }

    if ( NumOfDiodes > 2) { // Transistor with protection diode

#ifdef EBC_STYLE
#if EBC_STYLE == 321
      // Layout with 321= style
      if (((PartMode == PART_MODE_NPN) && (trans.c < trans.e)) || ((PartMode != PART_MODE_NPN) && (trans.c > trans.e)))
#else
      // Layout with EBC= style
      if (PartMode == PART_MODE_NPN)
#endif
#else
      // Layout with 123= style
      if (((PartMode == PART_MODE_NPN) && (trans.c > trans.e)) || ((PartMode != PART_MODE_NPN) && (trans.c < trans.e)))
#endif
      {
        lcd_fix_string(AnKat);  // "->|-"
      } else {
        lcd_fix_string(KatAn);  // "-|<-"
      }
    }

#if defined(NOK5110) || defined(OLED096)
    lcd_line2();
#endif

    PinLayout('E', 'B', 'C');   // EBC= or 123=...

#if defined(NOK5110) || defined(OLED096)
    lcd_line3();
#else
    lcd_line2();  // 2 row
#endif

    lcd_fix_string(hfe_str);    // "B="  (hFE)
    DisplayValue(trans.hfe[0], 0, 0, 3);
    lcd_space();

#if defined(NOK5110) || defined(OLED096)
    lcd_line4();
#endif

    lcd_fix_string(Uf_str);   // "Uf="
    DisplayValue(trans.uBE[0], -3, 'V', 3);
    goto end;

    // end (PartFound == PART_TRANSISTOR)

  } else if (PartFound == PART_FET) { // JFET or MOSFET

    if (PartMode & 1) {
      lcd_data('P');      // P-channel
    } else {
      lcd_data('N');      // N-channel
    }
    lcd_data('-');

    tmp = PartMode / 2;
    if (tmp == (PART_MODE_N_D_MOS / 2)) {
      lcd_data('D');      // N-D
    }
    if (tmp == (PART_MODE_N_E_MOS / 2)) {
      lcd_data('E');      // N-E
    }

    if (tmp == (PART_MODE_N_JFET / 2)) {
      lcd_fix_string(jfet_str);         // "JFET"
    } else {
      lcd_fix_string(mosfet_str);       // "-MOS "
    }

#if defined(NOK5110) || defined(OLED096)
    lcd_line2();
#endif

    PinLayout('S', 'G', 'D');   // SGD= or 123=...

    if ((NumOfDiodes > 0) && (PartMode < PART_MODE_N_D_MOS)) {
      // MOSFET with protection diode; only with enhancement-FETs

#ifdef EBC_STYLE
#if EBC_STYLE == 321
      // layout with 321= style
      if (((PartMode & 1) && (trans.c > trans.e)) || ((!(PartMode & 1)) && (trans.c < trans.e)))
#else
      // Layout with SGD= style
      if (PartMode & 1) // N or P MOS
#endif
#else
      // layout with 123= style
      if (((PartMode & 1) && (trans.c < trans.e)) || ((!(PartMode & 1)) && (trans.c > trans.e)))
#endif
      {
        lcd_data(LCD_CHAR_DIODE1);  // show Diode symbol >|
      } else {
        lcd_data(LCD_CHAR_DIODE2);  // show Diode symbol |<
      }
    }

#if defined(NOK5110) || defined(OLED096)
    lcd_line3();
#else
    lcd_line2();  // 2 row
#endif

    if (PartMode < PART_MODE_N_D_MOS) {   // enhancement-MOSFET
      // Gate capacity
      lcd_fix_string(GateCap_str);    // "C="
      ReadCapacity(trans.b, trans.e);   // measure capacity
      DisplayValue(cap.cval, cap.cpre, 'F', 3);

#if defined(NOK5110) || defined(OLED096)
      lcd_line4();
#endif

      lcd_fix_string(vt_str);     // "Vt="
    } else {
      lcd_data('I');
      lcd_data('=');
      DisplayValue(trans.uBE[1], -5, 'A', 2);

#if defined(NOK5110) || defined(OLED096)
      lcd_line4();
#endif

      lcd_fix_string(Vgs_str);      // " Vgs="
    }

    // Gate-threshold voltage
    DisplayValue(gthvoltage, -3, 'V', 2);
    goto end;

    // end (PartFound == PART_FET)

  } else if (PartFound == PART_THYRISTOR) {
    lcd_fix_string(Thyristor);      // "Thyristor"
    goto gakOutput;

  } else if (PartFound == PART_TRIAC) {
    lcd_fix_string(Triac);      // "Triac"
    goto gakOutput;

  } else if (PartFound == PART_RESISTOR) {
    if (ResistorsFound == 1) {      // single resistor
      lcd_testpin(resis[0].rb);     // Pin-number 1
      lcd_fix_string(Resistor_str);
      lcd_testpin(resis[0].ra);     // Pin-number 2
    } else {          // R-Max suchen
      ii = 0;
      if (resis[1].rx > resis[0].rx)
        ii = 1;

      if (ResistorsFound == 2) {
        ii = 2;
      } else {
        if (resis[2].rx > resis[ii].rx)
          ii = 2;
      }

      char x = '1';
      char y = '3';
      char z = '2';

      if (ii == 1) {
        //x = '1';
        y = '2';
        z = '3';
      }

      if (ii == 2) {
        x = '2';
        y = '1';
        z = '3';
      }

      lcd_data(x);
      lcd_fix_string(Resistor_str);    // "-[=]-"
      lcd_data(y);
      lcd_fix_string(Resistor_str);    // "-[=]-"
      lcd_data(z);
    }

    lcd_line2();  // 2 row

    if (ResistorsFound == 1) {
      RvalOut(0);

#if FLASHEND > 0x1fff
      if (resis[0].lx != 0) {
        // resistor have also Inductance

#if defined(NOK5110) || defined(OLED096)
        lcd_line3();
#endif

        lcd_fix_string(Lis_str);        // "L="
        DisplayValue(resis[0].lx, resis[0].lpre, 'H', 3); // output inductance
      }
#endif

    } else {
      // output resistor values in right order
      if (ii == 0) {
        RvalOut(1);
        RvalOut(2);
      }
      if (ii == 1) {
        RvalOut(0);
        RvalOut(2);
      }
      if (ii == 2) {
        RvalOut(0);
        RvalOut(1);
      }
    }
    goto end;

    // end (PartFound == PART_RESISTOR)

    // capacity measurement is wanted
  } else if (PartFound == PART_CAPACITOR) {
    //lcd_fix_string(Capacitor);
    lcd_testpin(cap.ca);    // Pin number 1
    lcd_fix_string(CapZeich);   // capacitor sign
    lcd_testpin(cap.cb);    // Pin number 2

#if FLASHEND > 0x1fff
    GetVloss();     // get Voltage loss of capacitor
    if (cap.v_loss != 0) {

#if defined(NOK5110) || defined(OLED096)
      lcd_line4();
#endif

      lcd_fix_string(VLOSS_str);  // "  Vloss="
      DisplayValue(cap.v_loss, -1, '%', 2);
    }
#endif

    lcd_line2();  // 2 row
    DisplayValue(cap.cval_max, cap.cpre_max, 'F', 4);

#if FLASHEND > 0x1fff
    cap.esr = GetESR(cap.cb, cap.ca);   // get ESR of capacitor
    if (cap.esr < 65530) {

#if defined(NOK5110) || defined(OLED096)
      lcd_line3();
#endif

      lcd_fix_string(ESR_str);
      DisplayValue(cap.esr, -2, LCD_CHAR_OMEGA, 2);
    }
#endif

    goto end;
  }

  if (NumOfDiodes == 0) {   // no diodes are found
    lcd_fix_string(TestFailed1);  // "No, unknown, or"
    lcd_line2();      // 2 row
    lcd_fix_string(TestFailed2);  // "damaged "
    lcd_fix_string(Component);    // "part"
  } else {
    lcd_fix_string(Component);    // "part"
    lcd_fix_string(Unknown);    // " unknown"
    lcd_line2();      // 2 row
    lcd_fix_string(OrBroken);     // "or damaged "
    lcd_data(NumOfDiodes + '0');
    lcd_fix_string(AnKat);    // "->|-"
  }

  empty_count++;
  mess_count = 0;
  goto end2;

gakOutput:
  lcd_line2();  // 2 row
  PinLayout(Cathode_char, 'G', 'A'); // CGA= or 123=...

  //- - - - - - - - - - - - - - - - - - - - - - - - - - - -
end:
  empty_count = 0;    // reset counter, if part is found
  mess_count++;     // count measurements

end2:
  //ADC_DDR = (1<<TPREF) | TXD_MSK;  // switch pin with reference to GND, release relay
  ADC_DDR = TXD_MSK;                 // switch pin with reference to GND, release relay
  goto start;

  while (!(ON_PIN_REG & (1 << RST_PIN))); // wait ,until button is released
  wait_about200ms();
  // wait 14 seconds or 5 seconds (if repeat function)

  for (gthvoltage = 0; gthvoltage < display_time; gthvoltage += 10) {
    if (!(ON_PIN_REG & (1 << RST_PIN))) {
      // If the key is pressed again...
      // goto start of measurement
      goto start;
    }
    wdt_reset();
    wait_about10ms();
  }

#ifdef POWER_OFF
#if POWER_OFF > 127
#define POWER2_OFF 255
#else
#define POWER2_OFF POWER_OFF*2
#endif

#if POWER_OFF+0 > 1
  if ((empty_count < POWER_OFF) && (mess_count < POWER2_OFF)) {
    goto start;     // repeat measurement POWER_OFF times
  }
#endif

  // only one Measurement requested, shut off
  //MCUSR = 0;
  ON_PORT &= ~(1 << ON_PIN);  // switch off power
  // never ending loop
  while (1) {
    if (!(ON_PIN_REG & (1 << RST_PIN))) {
      // The statement is only reached if no auto off equipment is installed
      goto start;
    }
    wdt_reset();
    wait_about10ms();
  }

#else
  goto start;  // POWER_OFF not selected, repeat measurement
#endif

  return;
} // end main
