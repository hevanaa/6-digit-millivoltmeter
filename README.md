# 6-digit-millivoltmeter
Arduino sketch for a 6-digit millivolt meter

This code is derived from the code in the blog post here: 
https://www.barbouri.com/2019/08/07/millivolt-meter-version-2/

Older versions of this code is available here:
https://www.barbouri.com/2016/05/26/millivolt-meter/
https://github.com/paulvee/6-digit-milli-voltmeter
http://www.scullcom.uk/category/projects/millivolt-meter/

The hardware is based on Barbouri's version 2 board, with a few changes.

In this code there is also a 0-volt calibration and the reference voltage is ignored.
Instead, three linear curves are calculated. The first one between 0V and 0.4096V, 
the second between 0.4096V and 3.6864V and the last one between 4.096 and 36.864V. 

This repo includes the modified Adafruit_RGBLCDShield library with additional input I/O.
