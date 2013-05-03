/*************************************************** 
 * This is a library for the BMP085 Barometric Pressure & Temp Sensor
 * 
 * Designed specifically to work with the Adafruit BMP085 Breakout 
 * ----> https://www.adafruit.com/products/391
 * 
 * These displays use I2C to communicate, 2 pins are required to  
 * interface
 * Adafruit invests time and resources providing this open source code, 
 * please support Adafruit and open-source hardware by purchasing 
 * products from Adafruit!
 * 
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 * BSD license, all text above must be included in any redistribution
 * 
 * -------------------------------------------------------------------
 * 
 * Improved by Michal Canecky/Cano 2013-05-01
 * -mess clean up
 * -optimized calculations
 * -calculation of altitude without use of pow() function and floats thus
 * -minimized sketch size
 * 
 * sensor datasheet:
 * http://media.digikey.com/pdf/Data%20Sheets/Bosch/BMP085.pdf
 * 
 * 
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

// returns pressure in Pa
int32_t BMP085::readPressure(void) {
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    UT = readRawTemperature();
    UP = readRawPressure();
    //see manual http://media.digikey.com/pdf/Data%20Sheets/Bosch/BMP085.pdf page 13

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
    } 
    else {
        p = (B7 / B4) << 1;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + (int32_t)3791)>>4);
    return p;
}

//return temperature in C
float BMP085::readTemperature(void) {
    return readTemperature10C()/10;
}

//return temperature in 0.1C
int16_t BMP085::readTemperature10C(void) {
    int32_t UT, X1, X2, B5;     // following ds convention

    UT = readRawTemperature();

    X1 = ((UT - (int32_t)ac6) * ((int32_t)ac5)) >> 15;
    X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
    B5 = X1 + X2;
    return (B5 + 8) >> 4;
}

//return altitude in meters
float BMP085::readAltitude(int16_t sealevelPressure) {
    float altitude;

    int32_t pressure = readPressure();

    altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 1.0 / 5.255));

    return altitude;
}

//return altitude in deci meters, approximation using Taylor series
int16_t BMP085::readAltitudeDM(int16_t sealevelPressure) {
    int16_t altitude;

    //Taylor series for sealevelPressure = 101325
    //-0.0832546 (x-101325)+3.32651x10^-7 (x-101325)^2-1.98043x10^-12 (x-101325)^3+1.37291x10^-17 (x-101325)^4-1.0324x10^-22 (x-101325)^5+8.16767x10^-28 (x-101325)^6+O((x-101325)^7)


    //Taylor Series at x = pressure = 101325, a = sealevelPressure 
    //(44330.-397430. (1/a)^0.190295)-0.746399 (1/a)^0.190295 (x-101325)+2.9823x10^-6 (1/a)^0.190295 (x-101325)^2-1.7755x10^-11 (1/a)^0.190295 (x-101325)^3+1.23085x10^-16 (1/a)^0.190295 (x-101325)^4-9.25572x10^-22 (1/a)^0.190295 (x-101325)^5+O((x-101325)^6)

    //substitute b = (1/a) ^ 0.19029495718363463368220742150333
    //we have Taylor series for that too at x = 101325
    //0.111542-2.09483x10^-7 (x-101325)+1.23043x10^-12 (x-101325)^2-8.86586x10^-18 (x-101325)^3+6.97871x10^-23 (x-101325)^4-5.77209x10^-28 (x-101325)^5+O((x-101325)^6)

    int32_t ax1 = sealevelPressure - 101325;
    int32_t ax2 = ax1 * ax1;
    int32_t ax3 = ax2 * ax1;

    float b = 0.111542 - 2.09483E-7 * ax1 + 1.23043E-12 * ax2 - 8.86586E-18 * ax3;


    int32_t pressure = readPressure();
    int32_t px1 = pressure - 101325;
    int32_t px2 = px1 * px1;
    int32_t px3 = px2 * px1;

    altitude = 44330 -397430 * b - 0.746399 * b * px1 + 2.9823E-6 * b * px2 - 1.7755E-11 * b * px3;

    //  altitude = -0.0832546 * px1 + 3.32651E-7 * px2 - 1.98043E-12 *px3; //estimate for sealevelPressure = 101325

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

