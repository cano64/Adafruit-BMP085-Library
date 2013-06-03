This is an Arduino library for BMP085 Barometric Pressure and Temperaure sensor.
Written by Michal Canecky/Cano based on library by Adafruit with some improvements.
Check also my [ATTiny compatible BMP085 library](https://github.com/cano64/ATTiny85-ATTiny84-BMP085-Arduino-Library-FastAltitude)

	This library is calculating altitudes
	without using pow() function, math library
	and floats altogether thus minimizing 
	your sketch size by about 1724 bytes
  magic :)
  
It is done by approximation using Taylor Series, centered at 500m altitude
As you go higher (or lower) the error offset is higher, but still negligible.

Uncalibrated (fixed for standard sea level pressure) function for calculating altitude.

	int16_t readAltitudeSTDdm()

is a small and fast function you will normally use to calculate altitude
using standard sea level pressure as a reference. It will return your altitude in decimeters.
Please note that your reading may be offset up to 100 meters based on weather,
depending whether it's sunny or raining. It's great for calculating relative changes in altitude.

If you need a calibrated measurement and have a method of entering
the current sea level pressure from your local weather report every time you use your device
use the following function to calculate your real altitude

	int32_t readAltitudemm(int32_t sealevelPressure = 101325)


Other functions in the library
	
	int readPressure(); //returns current pressure in Pa
	float readTemperature(); //returns current temperature in C
	int readTemperature10C(); //returns current temperature in tenths of C
	float readAltitude(); //left for compatibility with old library, returns altitude in m
	int readAltitudemm(); //returns current altitude in mm, calibrated for your local SLP
	int readAltitudeSTDmm(); //returns current altitude in mm, fixed for SSLP
	int16_t readAltitudeSTDdm(); //returns current altitude in dm, fixed for SSLP, for -3km to 3km range
	uint16_t readAltitudeSTDdm2(); //returns current altitude in mm, fixed for SSLP, for 0.5km to 6km range
	
