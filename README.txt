This is an Arduino library for BMP085 Barometric Pressure and Temperaure sensor.
Written by Michal Canecky/Cano based on library by Adafruit with some improvements.

This library is calculating altitudes without using pow() function and math library 
thus minimizing sketch size by about 1200 bytes.

Uncalibrated (standard sea level pressure) function for calculating altitude
without using floats -is- will be available as well saving another 1100 bytes of sketch size

int32_t readAltitudeSTDmm() is the function you would normally use, 
but your reading may be off about 100 meters based on weather, whether is sunny or raining

BMP085 requires I2C (two wire) communication.


