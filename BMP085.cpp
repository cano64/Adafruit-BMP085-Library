/*************************************************** 
  This is a library for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
  
  -------------------------------------------------------------------
  
  Improved by Michal Canecky/Cano 2013-05-01
  -mess clean up
  -optimized calculations
  -calculation of altitude without use of pow() function and floats thus
  -minimized sketch size
  
  
 ****************************************************/

#include "BMP085.h"
#include <util/delay.h>

BMP085::BMP085() {
}


uint8_t BMP085::begin(uint8_t mode) {
  if (mode > BMP085_ULTRAHIGHRES) oversampling = BMP085_ULTRAHIGHRES;
  else oversampling = mode;

  Wire.begin();

  if (read8(0xD0) != 0x55) return false; //not our altimeter

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  ac2 = read16(BMP085_CAL_AC2);
  ac3 = read16(BMP085_CAL_AC3);
  ac4 = read16(BMP085_CAL_AC4);
  ac5 = read16(BMP085_CAL_AC5);
  ac6 = read16(BMP085_CAL_AC6);

  b1 = read16(BMP085_CAL_B1);
  b2 = read16(BMP085_CAL_B2);

  mb = read16(BMP085_CAL_MB);
  mc = read16(BMP085_CAL_MC);
  md = read16(BMP085_CAL_MD);
  return true;
}

uint16_t BMP085::readRawTemperature(void) {
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  _delay_ms(5);
  return read16(BMP085_TEMPDATA);
}

uint32_t BMP085::readRawPressure(void) {
  uint32_t raw;

  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    _delay_ms(5);
  else if (oversampling == BMP085_STANDARD) 
    _delay_ms(8);
  else if (oversampling == BMP085_HIGHRES) 
    _delay_ms(14);
  else 
    _delay_ms(26);

  raw = read16(BMP085_PRESSUREDATA);

  raw <<= 8;
  raw |= read8(BMP085_PRESSUREDATA+2);
  raw >>= (8 - oversampling);

  return raw;
}


int32_t BMP085::readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

  // do temperature calculations
  X1=((UT-(int32_t)(ac6))*((int32_t)(ac5))) >> 15;
  X2=((int32_t)mc << 11)/(X1+(int32_t)md);
  B5=X1 + X2;

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) >> 2;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

  if (B7 < 0x80000000) {
    p = (B7 << 1) / B4;
  } else {
    p = (B7 / B4) << 1;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
  return p;
}

float BMP085::readTemperature(void) {
	return readTemperature10C()/10;
}

int16_t BMP085::readTemperature10C(void) {
  int32_t UT, X1, X2, B5;     // following ds convention

  UT = readRawTemperature();

  X1 = ((UT - (int32_t)ac6) * ((int32_t)ac5)) >> 15;
  X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  B5 = X1 + X2;
  return (B5 + 8) >> 4;
}

float BMP085::readAltitude(int16_t sealevelPressure) {
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}

int16_t BMP085::readAltitudeDM(int16_t sealevelPressure) {
  float altitude;

  float pressure = readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return (int16_t)(altitude *10);
}

/*********************************************************************/

uint8_t BMP085::read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  Wire.endTransmission(); // end transmission

  return ret;
}

uint16_t BMP085::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret <<= 8;
  ret |= Wire.read(); // receive DATA
  Wire.endTransmission(); // end transmission

  return ret;
}

void BMP085::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
  Wire.endTransmission(); // end transmission
}
