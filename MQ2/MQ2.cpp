/*
File: MQ2.cpp
Author: Leonardo Arcanjo - leonardoarcanjoo@gmail.com
Year: 2020
Version: 1.0
Description: This library was created in order to reduce code in Envoriment Monitoring.ino. All following methods and constants, 
it was obtained from: http://sandboxelectronics.com/?p=165 
Support:  Tiequan Shao: support[at]sandboxelectronics.com
Licence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application.

Besides a few explanations about how sensor works was taken from: Gas Sensors MQ Series Principle https://forum.arduino.cc/index.php?topic=416377.0 
*/

#include "MQ2.h"

#define HEATING_TIME 60000 // Following the Datasheet, it's necessary a heating time in order to calibrate the sensor. In this case 60 seconds

/*
	All below defines were taken from http://sandboxelectronics.com/?p=165
	PS: The RL_VALUE resistance value is obtained in MQ Sensor Module used. In this case is FC-22 SBX, more info in: Gas Sensors MQ Series Principle
*/
#define RL_VALUE 1  
#define RO_CLEAN_AIR_FACTOR 9.83 
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500
#define READ_SAMPLE_TIMES 5
#define READ_SAMPLE_INTERVAL 50

// Gas curves used to obtain concentration gas (in ppm)
float LPGCurve[3] = {2.3, 0.21, -0.47};
float COCurve[3] = {2.3, 0.72, -0.34};
float SMOKECurve[3] = {2.3, 0.53, -0.44};

int pino;

MQ2::MQ2(uint8_t pin){
	/* Object Constructor - The argument pin is turned INPUT mode and attribuited to private variable _pin*/
	pinMode(pin, INPUT);
	pino = pin;
}


void MQ2::begin(){
	/* begin - this method runs a HEATING_TIME seconds to heat the sensor MQ2*/
	delay(HEATING_TIME);
}



float MQ2::ResistanceCalculation(int adc_read){
/****************** MQResistanceCalculation ****************************************
    Input:   adc_read - value reads from adc, which represents the voltage
    Output:  the calculated sensor resistance
    Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
          across the load resistor and its resistance, the resistance of the sensor
          could be derived.
    PS: The ESP32 ADC has 12 bits resolution, thus there is a change in the expression
        that returns the resistance value
************************************************************************************/
	return (float) ((RL_VALUE * (4095 - adc_read))/adc_read);
}


float MQ2::calibration(){
/***************************** MQCalibration ****************************************
    Output:  Ro of the sensor
    Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
 ************************************************************************************/
	float val = 0;
	int i; 

	for(i = 0; i < CALIBRATION_SAMPLE_TIMES; i++){
		val += ResistanceCalculation(analogRead(pino));
		delay(CALIBRATION_SAMPLE_INTERVAL);
	}

	val = val / CALIBRATION_SAMPLE_TIMES;
	val = val / RO_CLEAN_AIR_FACTOR;

	return val;
}


float MQ2::ReadSensor(){
/*****************************  MQRead *********************************************
    Output:  Rs resistance of the sensor
    Remarks: This function use ResistanceCalculation to calculate the sensor resistence (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
	float rs = 0;
	int i = 0;
	
	for(i = 0; i < READ_SAMPLE_TIMES; i++){
		rs += ResistanceCalculation(analogRead(pino));
		delay(READ_SAMPLE_INTERVAL);
	}
	rs = rs / READ_SAMPLE_TIMES;
	return rs;
}

float MQ2::GetPercentageGas(float ro, subsType subs){
/*****************************  MQGetGasPercentage **********************************
Input:   ro - Ro - Reference Resistance obtained in calibration
         subs - Target Gas (LPG, CO, SMOKE)
Output:  ppm of the target gas
Remarks: This function passes different curves to the GetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
	float rs = ReadSensor();
	float ratio = rs / ro;

	if (subs == LPG){
		return GetPercentage(ratio, LPGCurve);
	} else if (subs == CO){
		return GetPercentage(ratio, COCurve);
	} else if (subs == SMOKE){
		return GetPercentage(ratio, SMOKECurve);
	}
}


float MQ2::GetPercentage(float ratio_rs_ro, float *SCurve){
	/*****************************  GetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
	double gasPercent = ((log(ratio_rs_ro) - SCurve[1]) / SCurve[2]) + SCurve[0];
	return pow(10, gasPercent);
}