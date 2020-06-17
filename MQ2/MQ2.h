/*
Author: Leonardo Arcanjo - leonardoarcanjoo@gmail.com
Year: 2020
Version: 1.0
Description: This library was created in order to reduce code in Envoriment Monitoring.ino. All following methods and constants, 
it was obtained from: http://sandboxelectronics.com/?p=165

Support:  Tiequan Shao: support[at]sandboxelectronics.com
 
Licence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
 
Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application.
*/

#ifndef MQ2_H
#define MQ2_H

#include <Arduino.h>
#include <math.h>

#define RO_CLEAN_AIR_FACTOR 9.83 

typedef enum {
	LPG, CO, SMOKE
} subsType;


class MQ2 {
	public:
		MQ2(uint8_t pin);
		void begin();
		float calibration();
		float GetPercentageGas(float ro, subsType subs);
		

	private:
		float ResistanceCalculation (int adc_read);
		float ReadSensor();
		float GetPercentage(float ratio_rs_ro, float *SCurve);
};

#endif
