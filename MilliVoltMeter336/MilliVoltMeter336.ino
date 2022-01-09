/* SCULLCOM HOBBY ELECTRONICS
 * MILLIVOLT METER USING LTC2400 24bit ADC CHIP
 * Using the Version 2.x PCB designed by Barbouri (Greg Christenson)
 * https://www.barbouri.com/2019/08/07/millivolt-meter-version-2/
 * 
 * Software version 3.36
 * 4.096 volt precision reference (ADR4540)
 * 
 * ****************************************************************
 * ****************************************************************
 * Items to modify in this code for your individual Millivolt Meter:
 * 
 * float adc_ref_volts - This is the actual value of the external voltage reference for battery calibration
 * batt_voltage = 9.5 - If using battery monitoring comment out this line in code and un-comment line above it
 * ****************************************************************
 * ****************************************************************
 *
 * Changes by dbldutch: 
 *  - added code to display the sw version in the welcome screens 
 *  - added code to display a real micro symbol instead of uV 
 *  - added code to monitor the charging level of a NiCAD/NiMH cell 
 *  - changed code to display decimal digits based on volts measured. 
 *  
 *  - Version 2.00:
 *  - stripped the 4LSB in the Read_ADC, before averaging. They add no value here.
 *    Calibration uses a larger sample size. Added a loop counter to show progress.
 *    
 *  - Version 3.00 : 
 *  - Changed from average based sampling to an Infinite Input Response (IIR) filter design.
 *    Details found here: https://github.com/arduino/Arduino/issues/3934
 *    and here : https://en.m.wikipedia.org/wiki/Infinite_impulse_response
 *  - Created a dynamic ADC conversion delay for the LTC2400 to wring out some idling time.
 *  - Changed to EEPROM.put & .get to also store floats. Eliminated the previous functions.
 *  - With the IIR filtering, there is no need for reduced decimals. Eliminated the code.
 *  - Added a calibration to a voltage reference to tweak the accuracy.
 *  
 *  - Version 3.10:
 *  - Added a dynamic filter weight algorithm to the IIR filter.
 *  
 *  - Version 3.11:
 *  - Changed the conversion delay to (re)start at the end of the LCD update cycle, such that the LTC
 *    has 165 mSec of "quiet" time on the power and data lines to do the sampling.
 *  - Fixed a compounded rounding error of the IIR filter calculation.
 *  - Added the filter weight exponent multiplier to the display.
 *  
 *  - Version 3.12:
 *  - Added some tweaks and changes to allow a precise calibration of the reference voltage. This
 *    largely determines the linearity of the meter.
 *
 *  - Version 3.14 (Paul V):
 *  - Added remote logging of the measured values.
 *
 *  - Version 3.x - 3.34:
 *  - Modified May 2016, December 2018 by Barbouri
 *  - for I2C display and pushbuttons 
 *  - Added includes Wire.h, Adafruit_MCP23017.h, Adafruit_RGBLCDShield.h 
 *  - https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library 
 *  - remapped buttons from processor to I2C port expander
 *  - Compilled using Arduino 1.8.2
 *  - Added COTO relay for 0-4 V measurement range on board version 2.11
 *  - Changed code to support 2-point calibration for both ranges.
 *  - Added range switch to front panel and connected to I2C port expander, with code to support.
 *  - Added overvoltage range switching to 40V on 4V scale using voltage > 4.01 volts or EXR (bit 28) status bit.
 *
 *  - Version 3.36:
 *  - Modified January 2022 by johanh
 *  - Added 0V calibration and removed v_ref
 *  - Using three curves from calibration points to calculate voltage
 *  
 *   Software version:
 */
String SW_VERSION = " Version 3.36";

/*
 * Battery indicator level:
 * - Analog pin A0 (via a 20K:10K divider) [10K is minimum value for the A2D]
 */
 
#include <SPI.h>                         // include SPI library (Serial Peripheral Interface)
                                         //
#include <Wire.h>                        // include I2C library
                                         //
#include <Adafruit_RGBLCDShield.h>       // include MODIFIED Adafruit RGB I2C Display to include 6th MENU button
                                         // 
#include <EEPROM.h>                      // include EEPROM library

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#define BLACK 0x0                        // For use during Vref six week burn-in

//---- LT 2400 ADC convertor
const int LTC_CS = 10;                   // set ADC chip select pin !CS (!SS) to Arduino pin D10
                                         // SPI SLCK is D13, SDO (MISO) is D12
long adcread;                            // reading from the ADC (LTC2400)
int ct = 165;                            // ADC converstion time is 160mS typical, +/-3.2 mS (data sheet)
unsigned long ct_start;                  // seed the conversion start timestamp
unsigned long ct_chk;                    // the entry timestamp, used to dynamically create the delay
float volt;                              // voltage reading from ADC
byte x;                                  // ADC status register (bits 31 to 25) 2 (0010) in range, 3 (0011) out of range high

//---- Range relay control
const int RELAY = 4;                     // set Relay pin PD4 to Arduino pin D4
int range = 1;                           // 1 is 0.000 to 40.095V, 0 is 0.000 to 4.010 input voltage

//---- Calibration Factors
float CA_0;                              // Calibration actual 0 --- averaged reading with shorted input --- stored in EEPROM
float CA_1;                              // Calibration actual 1 --- averaged reading with 0.4096 volt input (10%) --- stored in EEPROM
float CA_2;                              // Calibration actual 2 --- averaged reading with 3.6864 volt input (90%) --- stored in EEPROM
float M_1;                               // Gradient of curve from 0V to 0.4096V
float M_2;                               // Gradient of curve from 0.4096V to 3.6864V
long CA_0_address = 0L;                  // EEPROM address 0..3
long CA_1_address = 5L;                  // EEPROM address 5..8
long CA_2_address = 10L;                 // EEPROM address 10..14
long M_1_address = 15L;                  // EEPROM address 15..18
long M_2_address = 20L;                  // EEPROM address 20..24

float CA_3;                              // Calibration actual 1 --- averaged reading with 4.096 volt input (10%) --- stored in EEPROM
float CA_4;                              // Calibration actual 2 --- averaged reading with 36.864 volt input (90%) --- stored in EEPROM
float M_3;                               // Gradient of curve from 4.096V to 36.864V
long CA_3_address = 25L;                 // EEPROM address 25..28
long CA_4_address = 30L;                 // EEPROM address 30..33
long M_3_address = 35L;                  // EEPROM address 35..38

//---- IIR Low Pass Filtering with a dynamic filter weight algorithm
float average;                           // holds the result of the filter
int filterWeight = 64;                   // from 4..64 Higher numbers = heavier filtering
int fw_multiplier = 1;                   // multiplier for the filterWeight calculation 16777216
int noise_level = 48;                    // 96 is +/- 58.6 uV input voltage differential 40V range
int noise_level4 = 480;                  // 480 is +/- 58.6 uV input voltage differential 4V range

const int cal_adj_samples = 256;         // number of samples

//---- Display the result to the LCD
int DecPlaces = 0;                       // limit to the number of decimal places on display
String v;                                // string to hold the V, mV or uV string data for the display
String micro;                            // string to hold the real micro character
int dec_digit = 6;                       // integer that holds number of decimal places displayed on LCD
int dTV = 6;                             // default is 10.00000 V for 10 Volt and above
int dV = 6;                              // default is 1.000000 V for 1-9 Volt range
int dmV = 3;                             // default is 100,000 mV for MilliVolt range
int duV = 0;                             // default is 1000000 uV for MicroVolt range

//---- Arduino ADC Battery check
int batt = A0;                           // the 9V NiMH or NiCAD battery
int check_v = 0;                         // check the battery voltage
int loop_cnt = 365;                      // check it approx. every minute (loop time is 0.165 Sec)
float adc_cal = 0.0;                     // ADC calibration value
float adc_ref_volts = 5.01;              // external volt reference (VCC) -> measured
float adc_res = 1024;                    // 10 bit ADC resolution
const int adc_samples = 4;               // set number of sample readings to average the result

// Create a set of new symbols that can be displayed on the LCD
byte batt_full[8] =                      // full battery symbol, displayed at 9V
  {
  B01110,B11111,B11111,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_7[8] =                       // 8.7V level
  {
  B00000,B01110,B11111,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_3[8] =                       // 8.3V level
  {
  B00000,B00000,B01110,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_0[8] =                       // 8.0V level
  {
  B00000,B00000,B00000,B01110,B11111,B11111,B11111,B11111
  };
byte batt_7_7[8] =                       // 7.7V level
  {
  B00000,B00000,B00000,B00000,B01110,B11111,B11111,B11111
  };
byte batt_7_5[8] =                       // 7.5V level
  {
  B00000,B00000,B00000,B00000,B00000,B01110,B11111,B11111
  };
byte batt_empty[8] =                     // <7.3V Empty
  {
  B00100,B00100,B00100,B00000,B00100,B00000,B01110,B11111
  };
byte batt_charging[8] =                  // >10V Charging symbol
  {
  B00010,B00100,B01000,B11111,B00010,B10100,B11000,B11100
  };
  
//---- initialize the i2c/LCD library
  Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// **********************************************************************
// ******** Initialization routine, runs only at start or reboot ********
// **********************************************************************
void setup() {
  
  Serial.begin(9600);                    // ==>> activate for debug and test only

  micro = char(228);                     // real micro symbol is char(b11100100)

  pinMode (RELAY, OUTPUT);               // set Relay (pin D4 on Arduino Nano) to OUTPUT mode
  digitalWrite(RELAY, HIGH);             // set Relay pin HIGH to divide input by 10, 5V relay energized
  delay(500);                            // delay for 0.5 seconds for LCD startup
  pinMode (LTC_CS, OUTPUT);              // set LTC_CS (pin D10 on Arduino Nano) to OUTPUT mode
  digitalWrite(LTC_CS, HIGH);            // set LCT2400 chip select pin HIGH to disable
                                         // initialize digital pin LED_BUILTIN as an output.
                                            
  SPI.begin();                           // initialise SPI bus
  SPI.setBitOrder(MSBFIRST);             // Sets the order of bits shifted out and in to SPI bus, MSBFIRST (most-significant bit first)
  SPI.setDataMode(SPI_MODE0);            // set SPI to Mode 0 (MOSI read on rising edge (CPLI=0) and SCK idle low (CPOL=0))
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // divide Arduino clock by 16 to gave a 1 MHz SPI clock

  lcd.begin(16, 2);                      // set up the LCD's number of columns and rows
  lcd.setBacklight(BLUE);                // set LCD backlight color to Blue
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("MilliVolt Meter ");         // print MilliVolt Meter name to display
  lcd.setCursor(0,1);                    // set LCD cursor to column 0, row 1 (start of second line)
  lcd.print(SW_VERSION);                 // print software version to display
  delay(2500);                           // display for 2.5 seconds
  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)

  EEPROM.get(CA_0_address, CA_0);        // get the 4V low cal actual value
  EEPROM.get(CA_1_address, CA_1);        // get the 4V low cal actual value
  EEPROM.get(CA_2_address, CA_2);        // get the 4V hi cal actual value
  EEPROM.get(CA_3_address, CA_3);        // get the 40V low cal actual value
  EEPROM.get(CA_4_address, CA_4);        // get the 40V hi cal actual value
  EEPROM.get(M_1_address, M_1);          // get the first curve gradient from EEPROM
  EEPROM.get(M_2_address, M_2);          // get the second curve gradient from EEPROM
  EEPROM.get(M_3_address, M_3);          // get the third curve gradient from EEPROM


  // ==>> for debugging & Testing, comment out for faster startup
  /*
  lcd.clear();                           // clear dislay
  lcd.setBacklight(YELLOW);              // set LCD backlight color to Yellow
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Cal Actual 0V Low");
  Serial.print ("Cal Actual 0V Low: ");
  lcd.setCursor(0,1);
  lcd.print(CA_0, 6);                    // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (CA_0, 6);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Cal Actual 4V Low");
  Serial.print ("Cal Actual 4V Low: ");
  lcd.setCursor(0,1);
  lcd.print(CA_1, 6);                    // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (CA_1, 6);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Cal Actual 4V High");
  Serial.print ("Cal Actual 4V High: ");
  lcd.setCursor(0,1);
  lcd.print(CA_2);                       // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (CA_2);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("M_1 4V Range  ");
  Serial.print ("M_1 4V range: ");
  lcd.setCursor(0,1);
  lcd.print(M_1, 6);                     // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (M_1, 6);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("M_2 4V Range");
  Serial.print ("M_2 4V range: ");
  lcd.setCursor(0,1);
  lcd.print(M_2);                        // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (M_2);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Cal Actual 40V Low");
  Serial.print ("Cal Actual 40V Low: ");
  lcd.setCursor(0,1);
  lcd.print(CA_3);                       // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (CA_3);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Cal Actual 40V High");
  Serial.print ("Cal Actual 40V High: ");
  lcd.setCursor(0,1);
  lcd.print(CA_4);                       // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (CA_4);
  delay(1000);

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("M_3 40V Range ");
  Serial.print ("M_3 40V range: ");
  lcd.setCursor(0,1);
  lcd.print(M_3, 6);                     // Briefly show calibration factor stored in EEPROM at switch on
  Serial.println (M_3, 6);
  delay(2000);
  */

  // Setup display with initial static text 
  lcd.clear();                           // Clear LCD screen for initial setup
  lcd.setBacklight(TEAL);                // set LCD backlight color to Teal (Black during burn-in)
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  if (range == 1) lcd.print("40V ");     // print Range 40V
  if (range == 0) lcd.print("4V  ");     // print Range 4V
  lcd.setCursor(4,0);                    // set LCD cursor to column 4, row O (start of line after range)
  lcd.print("            ");             // print to display "mV Meter" and clear the rest BLANK FOR TEST
  lcd.setCursor(14,1);                   // set LCD cursor to column 12, row 1 (start of "fw=")
  lcd.print("f ");                       // print to display "fw=" and clear last digit
  
  // create a set of special symbols from the battery monitor definitions above
  lcd.createChar (0, batt_full);         // >9V
  lcd.createChar (1, batt_8_7);          // 8.7V
  lcd.createChar (2, batt_8_3);          // 8.3V
  lcd.createChar (3, batt_8_0);          // 8.0V
  lcd.createChar (4, batt_7_7);          // 7.7V
  lcd.createChar (5, batt_7_5);          // 7.5V
  lcd.createChar (6, batt_empty);        // <7.3V measurements can be inaccurate!
  lcd.createChar (7, batt_charging);     // > 10V charging and connected to wall-wart
  analogReference(DEFAULT);              // not needed here, use the external or default (=internal) A2D reference

  Monitor_batt();                        // get the battery level and show it on the display
  
  ct_start = millis();                   // seed the conversion start timestamp for the LTC2400
  for (int i=0;i<5;i++) {                // disregard the first five readings as they seem unstable
    average = Spi_Read();                // and also seed the IIR filter
  }
}


/**************************************************************************************
 * Routine to read the data from the LTC2400 A2D convertor through the SPI interface
 */
long Spi_Read(void){                     // SPI(Serial Peripheral Interface) read sub-routine to read data form the LTC2400 ADC
                                         // and transfer 8 bits (1 byte) at a time - total of 4 bytes.
  long result = 0L;                      // result represents rolling total of the bytes transferred
  long b;                                // b is result of reading ADC output bytes
  
  //calculate the minimum conversion delay dynamically
  ct_chk = millis();
  unsigned int ct_delay = ct_chk - ct_start; // use the time already spent and factor that in
  if (ct_delay < ct){
    delay(ct - ct_delay);                // use the adjusted conversion delay if needed
  }
  
  digitalWrite(LTC_CS, LOW);             // LTC2400 chip select pin taken low to wake up the ADC and enable the SDO (MOSI) output
  delayMicroseconds(1);                  // timing delay but is not really required

  if (!(PINB & (1 << 4))) {              // check for a low !EOC on the MOSI pin D12, if the ADC is ready to transmit new data
                                         // if not, try again later -> this will reduce the number of readings to average
    b = SPI.transfer(0xff);              // transfer first byte most significant bits first.
  // ***********************************************************************************************
    x = b;                               // copy first byte to x   used for OVP range switching
    x &= 0xf0;                           // discard last 4 bits to leave status (bits 31 to 25) in x
    x = x >> 4;                          // shift high nibble (right) to low nibble
  // ***********************************************************************************************
    b &= 0x0f;                           // discard first 4 status bits (bits 31 to 25) mask received data with binary 00001111
    result = b;                          // result after removing first 4 bits (replacing them with 0's)
    result <<= 8;                        // shift first byte left by 8 places
    b = SPI.transfer(0xff);              // transfer second byte most significant bits first.
    result |= b;                         // add second byte to first byte by using the OR function (now 12 bits)
    result = result << 8;                // shift result left by 8 places
    b = SPI.transfer(0xff);              // transfer third byte most significant bits first.
    result |= b;                         // add third byte to result by using the OR function (now 20 bits)
    result = result << 8;                // shift result left by 8 places
    b = SPI.transfer(0xff);              // transfer fourth byte most significant bits first.
    result |= b;                         // add fourth byte to result by using the OR function (now 28 bits)
    result = result >> 4;                // get rid of the 4 LSB bits, Sub-LSB bits - mostly noise
  
    digitalWrite(LTC_CS, HIGH);          // LTC2400 chip enters low power (sleep mode) and disables the ADC output.
    ct_start = millis();                 // start the conversion delay timer (restarted at the end of the main loop)
    return(result);                      // return with result as the 24 bit data representing the voltage
  }
}

/**************************************************************************************
 * Routine to calculate filter weight
 */

int filterWeightCalc(void){

  // Check if the new reading is outside the noise level band of the filtered result
  // and dynamically adjust the filter weight accordingly.
  if (range == 1) {
    if ((adcread > average + noise_level)||(adcread < average - noise_level)){
                                          // reading is outside the noise band
      fw_multiplier--;                    // scale the filterWeight down with powers of 2
      if (fw_multiplier < 1){             // lower limit to 2<<1 = 4
        fw_multiplier = 1;                // bottom-out
        average = adcread;                // and reset the filter
        } 
    }else{                                // the reading is inside the noise band
        fw_multiplier++;                  // scale the filterWeight up with powers of 2
        if (fw_multiplier > 6){           // upper limit is 2<<5 = 128 
            fw_multiplier = 6;
        }
    }
  }
  if (range == 0) {
    if ((adcread > average + noise_level4)||(adcread < average - noise_level4)){
                                          // reading is outside the noise band
      fw_multiplier--;                    // scale the filterWeight down with powers of 2
      if (fw_multiplier < 1){             // lower limit to 2<<1 = 4
        fw_multiplier = 1;                // bottom-out
        average = adcread;                // and reset the filter
        } 
    }else{                                // the reading is inside the noise band
        fw_multiplier++;                  // scale the filterWeight up with powers of 2
        if (fw_multiplier > 6){           // upper limit is 2<<5 = 128 
            fw_multiplier = 6;
        }
    }
  }
}

/**************************************************************************************
 * Routine to run the voltage calibration against a voltage reference.
 * Short the input first to read 0V.
 * The calibration factor is created by connecting a 0.40960V reference 
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Cal4_Low() {
  digitalWrite(RELAY, LOW);     // set Relay pin LOW to divide input by 1
  delay(100);

  // 0V calibration
  lcd.clear();
  lcd.setBacklight(GREEN);                            // set LCD backlight to Green
  lcd.setCursor(0,0);
  lcd.print("=Cal4-0V");
  lcd.setCursor(0,1);
  lcd.print("Short Input");
  delay(10000);

  int cal_counter = cal_adj_samples;                  // total number of readings

  for (int i=0; i < cal_adj_samples; i++) {           // create a moving average with an IIR filter

    adcread = Spi_Read();                             // seed the IIR filter

    // Check if the new reading is outside the noise level band of the filtered result
    // and dynamically adjust the filter weight accordingly.
    filterWeightCalc();
  
    // update the filter weight; ranges from 4..128
    filterWeight = 2 << fw_multiplier;
    
    average = average + (Spi_Read() - average) / (float)filterWeight;
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                            // get rid of the first decimal number               
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }

  CA_0 = average;
  EEPROM.put(CA_0_address, CA_0);     // store the calibration reading in EEPROM
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal4-Low 0V:");
  lcd.setCursor(0,1);
  lcd.print(average, 2);
  delay(5000);
  lcd.clear();


  // 0.40960V calibration
  lcd.clear();
  lcd.setBacklight(GREEN);                            // set LCD backlight to Green
  lcd.setCursor(0,0);
  lcd.print("=Cal4-Low");
  lcd.setCursor(0,1);
  lcd.print("Connect 0.40960");
  delay(10000);

  cal_counter = cal_adj_samples;                  // total number of readings

  for (int i=0; i < cal_adj_samples; i++) {           // create a moving average with an IIR filter

    adcread = Spi_Read();                             // seed the IIR filter

    // Check if the new reading is outside the noise level band of the filtered result
    // and dynamically adjust the filter weight accordingly.
    filterWeightCalc();
  
    // update the filter weight; ranges from 4..128
    filterWeight = 2 << fw_multiplier;
    
    average = average + (Spi_Read() - average) / (float)filterWeight;
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                            // get rid of the first decimal number               
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }

  CA_1 = average;
  EEPROM.put(CA_1_address, CA_1);        // store the calibration reading in EEPROM
  M_1 = (CA_1 - CA_0) / 0.409600;        // calculate gradient of curve
  EEPROM.put(M_1_address, M_1);          // store the curve gradient in EEPROM
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal4-Low Actual:");
  lcd.setCursor(0,1);
  lcd.print(average, 2);
  delay(10000);
  lcd.clear();
  lcd.setBacklight(TEAL);                // set LCD backlight to Teal
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("4V  ");                     // print Range 4V
  lcd.setCursor(4,0);                    // set LCD cursor to column 4, row O (start of line after range)
  lcd.print("mV Meter    ");             // print to display "mV Meter" and clear the rest
  lcd.setCursor(14,1);                   // set LCD cursor to column 12, row 1 (start of "fw=")
  lcd.print("f ");                       // print to display "fw=" and clear last digit
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

/**************************************************************************************
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is created by connecting a 3.686400V reference 
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Cal4_High() {
  digitalWrite(RELAY, LOW);     // set Relay pin LOW to divide input by 1
  delay(100);
  
  lcd.clear();
  lcd.setBacklight(GREEN);                            // set LCD backlight to Green
  lcd.setCursor(0,0);
  lcd.print("=Cal4-High");
  lcd.setCursor(0,1);
  lcd.print("Connect 3.686400");
  delay(10000);

  int cal_counter = cal_adj_samples;                  // total number of readings

  for (int i=0; i < cal_adj_samples; i++) {           // create a moving average with an IIR filter

    adcread = Spi_Read();                             // seed the IIR filter

    // Check if the new reading is outside the noise level band of the filtered result
    // and dynamically adjust the filter weight accordingly.
    filterWeightCalc();
  
    // update the filter weight; ranges from 4..128
    filterWeight = 2 << fw_multiplier;
    
    average = average + (Spi_Read() - average) / (float)filterWeight;
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                            // get rid of the first decimal number               
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }

  // The Cal4_High must be done after the Cal4_Low calibration as we do not have both points before.
  CA_2 = average;
  EEPROM.put(CA_2_address, CA_2);             // store the calibration reading A in EEPROM

  EEPROM.get(CA_1_address, CA_1);             // get the 4V low cal actual value
  M_2 = (CA_2 - CA_1) / 3.276800;             // calculate gradient of curve. 3.6864 - 0.4096 = 3.2768
  EEPROM.put(M_2_address, M_2);               // store the curve gradient in EEPROM
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal4-High Actual:");
  lcd.setCursor(0,1);
  lcd.print(average, 2);
  delay(10000);
  lcd.clear();
  lcd.setBacklight(TEAL);                // set LCD backlight to Teal
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("4V  ");                     // print Range 4V
  lcd.setCursor(4,0);                    // set LCD cursor to column 4, row O (start of line after range)
  lcd.print("mV Meter    ");             // print to display "mV Meter" and clear the rest
  lcd.setCursor(14,1);                   // set LCD cursor to column 12, row 1 (start of "fw=")
  lcd.print("f ");                       // print to display "fw=" and clear last digit
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

/**************************************************************************************
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is created by connecting a 4.09600V reference 
 * to the input. This will enhance the linearity of the conversion.
 * 
 */

void Cal40_Low() {
  digitalWrite(RELAY, HIGH);     // set Relay pin HIGH to divide input by 10
  delay(100);
  
  lcd.clear();
  lcd.setBacklight(GREEN);                            // set LCD backlight to Green
  lcd.setCursor(0,0);
  lcd.print("=Cal40-Low");
  lcd.setCursor(0,1);
  lcd.print("Connect 4.09600");
  delay(10000);

  int cal_counter = cal_adj_samples;                  // total number of readings

  for (int i=0; i < cal_adj_samples; i++) {           // create a moving average with an IIR filter

    adcread = Spi_Read();                             // seed the IIR filter

    // Check if the new reading is outside the noise level band of the filtered result
    // and dynamically adjust the filter weight accordingly.
    filterWeightCalc();
  
    // update the filter weight; ranges from 4..128
    filterWeight = 2 << fw_multiplier;
    
    average = average + (Spi_Read() - average) / (float)filterWeight;
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                            // get rid of the first decimal number               
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }

  CA_3 = average;
  EEPROM.put(CA_3_address, CA_3);        // store the calibration reading A in EEPROM
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal40-Low Actual");
  lcd.setCursor(0,1);
  lcd.print(average, 2);
  delay(10000);
  lcd.clear();
  lcd.setBacklight(TEAL);                // set LCD backlight to Teal
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("40V ");                     // print Range 4V
  lcd.setCursor(4,0);                    // set LCD cursor to column 4, row O (start of line after range)
  lcd.print("mV Meter    ");             // print to display "mV Meter" and clear the rest
  lcd.setCursor(14,1);                   // set LCD cursor to column 12, row 1 (start of "fw=")
  lcd.print("f ");                       // print to display "fw=" and clear last digit
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

/**************************************************************************************
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is created by connecting a 36.8640V reference 
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Cal40_High() {
  digitalWrite(RELAY, HIGH);     // set Relay pin HIGH to divide input by 10
  delay(100);
  
  lcd.clear();
  lcd.setBacklight(GREEN);                          // set LCD backlight to Green
  lcd.setCursor(0,0);
  lcd.print("=Cal40-High");
  lcd.setCursor(0,1);
  lcd.print("Connect 36.8640");
  delay(10000);

  int cal_counter = cal_adj_samples;                // total number of readings

  for (int i=0; i < cal_adj_samples; i++) {         // create a moving average with an IIR filter

    adcread = Spi_Read();                           // seed the IIR filter

    // Check if the new reading is outside the noise level band of the filtered result
    // and dynamically adjust the filter weight accordingly.
    filterWeightCalc();
  
    // update the filter weight; ranges from 4..128
    filterWeight = 2 << fw_multiplier;
    
    average = average + (Spi_Read() - average) / (float)filterWeight;
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                          // get rid of the first decimal number               
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }

  // The Cal4_High must be done after the Cal4_Low calibration as we do not have both points before.
  CA_4 = average;

  EEPROM.put(CA_4_address, CA_4);                   // store the calibration reading A in EEPROM

  EEPROM.get(CA_3_address, CA_3);                   // get the 40V low cal actual value
  M_3 = (CA_4 - CA_3) / 32.768;                     // calculate gradient of curve, 36.864 - 4.096 = 
  EEPROM.put(M_3_address, M_3);                     // store the curve gradient in EEPROM

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cal40-High Act: ");
  lcd.setCursor(0,1);
  lcd.print(average, 2);
  delay(10000);
  lcd.clear();
  lcd.setBacklight(TEAL);                // set LCD backlight to Teal
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("4V  ");                     // print Range 4V
  lcd.setCursor(4,0);                    // set LCD cursor to column 4, row O (start of line after range)
  lcd.print("mV Meter    ");             // print to display "mV Meter" and clear the rest
  lcd.setCursor(14,1);                   // set LCD cursor to column 12, row 1 (start of "fw=")
  lcd.print("f ");                       // print to display "fw=" and clear last digit
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

/**************************************************************************************
 * Routine to check if the button was pressed, and depending on the length, decide what action to take
 */
void Button_press() {
   uint8_t buttons = lcd.readButtons();
                                          // Buttons availible on I2C LCD header: A0=SELECT, A1=RIGHT,
      if (buttons) {                      //  A2=DOWN, A3=UP, A4=LEFT, A5=MENU - Positions: GND,A0,A1,A2,A3,A4,A5
        
        lcd.setCursor(15, 1);             // placeholder for the button press ack
        lcd.print("0");                   // back to default

        if (buttons & BUTTON_SELECT) {
          Cal40_High();
        }
        if (buttons & BUTTON_RIGHT) {     // 0.409600 V Cal Position A1 (A)
            Cal4_Low();
        }
        if (buttons & BUTTON_DOWN) {      // 3.68640 V Cal Position A2 (B)
          Cal4_High();
        }
        if (buttons & BUTTON_UP) {        // 4.09600 V Cal Position A3 (C)
          Cal40_Low();
        }
        if (buttons & BUTTON_LEFT) {      // 36.8640 V Cal Position A4 (D)  // A4 BUTTON_LEFT doesn't work JH 3.1.2022, use BUTTON_SELECT instead
          Cal40_High();
        }
        if (buttons & BUTTON_MENU) {      // Range select Position A5
          if (range == 1) {
            (range = 0);
            digitalWrite(RELAY, LOW);     // set Relay pin LOW to divide input by 1
            lcd.setBacklight(TEAL);                // set LCD backlight to Teal
            lcd.setCursor(0,0);
            lcd.print("4V ");             // print Range 4V and clear next space
            Serial.println("Range: 4V");
            delay(100);
          } else {
            (range = 1);
            digitalWrite(RELAY, HIGH);    // set Relay pin HIGH to divide input by 10
            lcd.setCursor(0,0);
            lcd.print("40V");             // print Range 40V
            Serial.println("Range: 40V");
            delay(100);
          }
        }
      }  
}


/**************************************************************************************
 * The battery level monitor routine
 */
void Monitor_batt() {
  int i;
  long sum = 0;
  int sensorValue;
  for (i=0; i<(adc_samples); i++) {
    sensorValue = analogRead(batt) + adc_cal; // read from A0 and add the calibration factor
    delay(100);
    sum += sensorValue;
  }
  sum = sum / adc_samples;

  float batt_voltage = sum;
  // Convert the analog reading (which goes from 0 - 1023) to a voltage 0 - 12V:
 
  batt_voltage = (sensorValue * (adc_ref_volts / adc_res) * 3) + adc_cal;
  //  batt_voltage = 9.5;  ****** UN-COMMENT line above, and COMMENT out this line to use BATTERY MONITOR ******
  
  /*  note that during the test with a potmeter connected to 5V, the resulting maximum volt level is just below
   *  20V, if you connect to a PC with a USB cable for power.
   *  With the power coming from the 78L12, the maximum level will be 20V, unless you create a voltage divider to
   *  limit the maximum voltage coming from the potmeter to 3V, which is equal to the 20K:10K divider to the cell.
   */

  // print out the "battery" voltage level at the right-hand fields of the first line
  lcd.setCursor(15, 0);                  // start of the batt level field (line 1, last position)

  if (batt_voltage < 6.0) {              // critical batt level for the reference and the ADC is < 5.4V
    lcd.print(char(6));                  // battery is empty!
    } else if (batt_voltage < 6.6 && batt_voltage > 6.0){  // batt getting too low, connect to mains or stop
      lcd.print(char(5));                // stop measuring
    } else if (batt_voltage < 7.2 && batt_voltage > 6.6){  // batt charge is getting critical
      lcd.print(char(4));                //
    } else if (batt_voltage < 8.4 && batt_voltage > 7.2){  // batt charge is OK
      lcd.print(char(3));                //
    } else if (batt_voltage < 8.8 && batt_voltage > 8.4){  // batt charge is OK
      lcd.print(char(2));                //
    } else if (batt_voltage < 8.9 && batt_voltage > 8.8){  // batt charge is OK
      lcd.print(char(1));                //
    } else if (batt_voltage < 9.0 && batt_voltage > 8.9){  // batt is full
      lcd.print(char(0));
    } else if (batt_voltage > 9.0){      // batt is new lithium at 1.8V (10.8V pack)
      lcd.print(char(7));                // New batteries or recovered from extended off
    }
}


/**************************************************************************************
 * The main routine
 */
void loop() {
  
  // The minimum (also normal) looptime is 166 mSec, determined by the aquisition delay of the LTC2400
  
  Button_press();                        // check if the button was pressed

  // Check the battery level approx. every minute
  if (check_v > loop_cnt) {
      check_v = 0;
      Monitor_batt();                    // checking the battery level takes about 810 mSec
  } else {
    check_v ++;
  }

  // Take a new raw LTC2400 reading
  adcread = Spi_Read();
  
  // Check if the new reading is outside the noise level band of the filtered result
  // and dynamically adjust the filter weight accordingly.
  filterWeightCalc();
  
  // update the filter weight; ranges from 4..128
  filterWeight = 2 << fw_multiplier;

  /*  Run the quisition through the IIR filter and take the new reading with a
   *  grain of filterWeight salt. ie. divide the new reading by the filterWeight factor (from 4..64)
   *  Note: average must be a float, otherwise there will be a compounded rounding error in the result.
   */
  average = average + (adcread - average) / (float)filterWeight;

  /* 
   *  Convert the filtered result to volts.
   *  There are three ranges, 0V to 0.4096V, 0.4096V to 3.6864V (extends to 4.01V) and 4.096 to 36.684V (extends to 40V).
   *  Calculation is done in each range using the stored curve gradient.
   */
  if (range == 1) volt = ((average - CA_4) / M_3) + 36.864;  // 40V range
       
  if (range == 0) {
    if (average <= CA_1) {
      volt = ((average - CA_1) / M_1) + 0.4096;              // 0V to 0.4096V
    } else if (average > CA_1) {
      volt = ((average - CA_2) / M_2) + 3.6864;              // 0.4096 to 4.01V
    }
    if (volt > 4.01 || x == 3) {         // detect and switch to 40V range
      (range = 1);
      digitalWrite(RELAY, HIGH);         // set Relay pin HIGH to divide input by 10
      lcd.setBacklight(VIOLET);          // set LCD backlight to Violet to warn of range change
      lcd.setCursor(0,0);
      lcd.print("40V");                  // print Range 40V
    }
  }
  
  // prepare for the display of the data
  
  // ==>> for debugging & Testing, to be analyzed with MS-Excel:
  /*
  Serial.print(adcread);
  Serial.print("\t");
  Serial.print(average);
  Serial.print("\t");
  Serial.println(filterWeight);
  */
  //  ==>> for testing purposes only, read and display the 9V cell instead
  //  volt = (analogRead(A0) * (adc_ref_volts / adc_res) * 2) + adc_cal;

  if (volt <0.001) {                     // check if voltage reading is below 1 milli-Volt    
    volt = volt * 1000000;               // if so multiply reading by 1.000000 and display as micro-Volt
    v = micro + "V " ;                   // use uV on display after voltage reading
    dec_digit = duV;                     // set display to 0 decimal places (1000000 uV) 10d
  
  } else if (volt < 1){                  // check if voltage reading is below 1 volt
    volt = volt * 1000;                  // if below 1 volt multiply by 1.000 and display as Millivolt
    v = "mV ";                           // use mV on display after voltage reading
    dec_digit = dmV;                     // set display to 4 decimal places (100.0000 mV) 11d

  } else if (volt < 10){                 // check if voltage reading is below 10 volt
    v = "V ";                            // use V on display after voltage reading
    dec_digit = dV;                      // set display to 6 decimal places (1.000000 V) 10d
   
  } else {                               // volt is > 10V
    v = "V ";                            // if 10 volt or higher use letter V on display after voltage reading
    dec_digit = dTV;                     // set display to 5 decimal places (10.00000 V) 10d
  }

  lcd.setCursor(11, 1);                  // set LCD cursor to Column 0 and Row 1 (second row of LCD, first column)
  lcd.print(" ");                        // print 3 spaces to clear last reading
  lcd.setCursor(0, 1);                   // set LCD cursor to Column 0 and Row 1 (second row of LCD, first column)
  lcd.print(volt, dec_digit);            // print voltage as floating number with x decimal places
  Serial.print (volt, dec_digit);
  lcd.print(" ");                        // add one blank space after voltage reading
  Serial.print (" ");
  lcd.print(v);                          // print either uV, mV or V to LCD display
  Serial.println (v);
  lcd.print("  ");                       // add two blank spaces
  lcd.setCursor(4,0);        
  lcd.print (average, 0);
  lcd.print("  ");                       // add two blank spaces
  lcd.setCursor(15,1);
  lcd.print(fw_multiplier);              // show the filterweight exponent multiplier

  ct_start = millis();                   // After this noisy display intermezzo, reset the LTC2400 conversion time counter
                                         // such that the ADC has the full 165 mSec to sample a new acquisition
                                         // during a quiet period.
}
