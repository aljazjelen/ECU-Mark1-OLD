/*
 * Crankshaft.c
 *
 *  Created on: Oct 4, 2020
 *      Author: aljaz
 */


#include "crankshaft.h"

int Crank_ShaftFreqHz = 0;		// Global Variable of estimated Crankshaft frequency in Hz (filtered)
int Crank_ShaftFreqHzRaw = 0;	// Global Variable of estimated Crankshaft freq in Hz (non filtered, no missing tooth compensation)
uint32_t Crank_RotDelta = 0;	// Global Variable of exact delta time between 2 teeth
uint32_t Crank_PosDiff = 0;		// Global Variable to track Crankshaft Tooth New timing difference
uint32_t Crank_PosDiffOld = 0;	// Global Variable to track Crankshaft Tooth Old timing difference
uint8_t Crank_TeethCount = 0;	// Global Variable of exact Current Tooth Number
uint16_t Crank_Angle = 0;		// Global Variable of estimated Crankshaft Angle
uint32_t Crank_DivAngle = 0;	// Global Variable used to calculate angle between 2 rising edges of the tooth

// TODO move to own files (engine. where all central variables are)
uint16_t Engine_Angle = 0;		// Global Variable of estimated Engine System
// TODO move to own files (cam, where cam related drivers are stored)
uint8_t	Cam_CycleStart = 0;		// Global Variable to trach the start of Camshaft (0) is start, (1) is 2nd half cycle.

// Locals
uint16_t Crank_LastCapturedEdgeTime = 0;	// Global Time Stamp for last detected edge from Hal Sensor
uint8_t Crank_bErrToothJump = 0;			// Global Error flag when counted teeth and total tooth number dont match

// Parameters
uint8_t Crank_TeethNmbr_P = 17;	// Global Parameter for total number of teeth on the Crankshaft
uint8_t Crank_MissingTeethNmbr_P = 1;// Global Parameter for missing number of teeth on the Crankshaft
// TODO add a parameter which varries threshold for missing tooth detection algorithm


/* All Possible Init Stuff */
void Crank_Init(){
	Crank_DivAngle = 360/Crank_TeethNmbr_P;
}

/* Driver for calculating the speed and resetting the counter upon empty slot */
void Crank_HalGeberDriver(TIM_HandleTypeDef *htim){
	uint16_t capturedValue;
	//int Crank_ShaftFreqHzRaw = 0;
	//channelId = PWM_IC_CHANNEL_FLOW_METER;

	// Get CCR register for the specific Timer and Channel
	capturedValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	if( capturedValue > Crank_LastCapturedEdgeTime )
		// Compute the input signal frequency
		Crank_RotDelta = capturedValue - Crank_LastCapturedEdgeTime;
	else
	// Timer counter overflow
		Crank_RotDelta = ( 0xFFFF - Crank_LastCapturedEdgeTime ) + capturedValue;

	// Compute the input signal frequency
	Crank_RotDelta = Crank_RotDelta*(1+htim->Instance->PSC);
	Crank_ShaftFreqHzRaw = HAL_RCC_GetPCLK1Freq()/Crank_RotDelta/(Crank_TeethNmbr_P+1);  // calculate frequency

	// Update the last captured value
	Crank_LastCapturedEdgeTime = capturedValue;
	Crank_PosDiff = Crank_RotDelta;

	// Check for the empty tooth - if the difference between timestamps is bigger than usually.
	if(((Crank_PosDiff-Crank_PosDiffOld) >= 0.7*Crank_PosDiffOld) && (Crank_PosDiff >= Crank_PosDiffOld)){
		Crank_PosDiffOld = Crank_PosDiff;

		Crank_ShaftFreqHz = Crank_ShaftFreqHz; // take the last valid value in case the tooth is missing
		if (Crank_TeethCount >= Crank_TeethNmbr_P-1){
			Crank_bErrToothJump = 1;
		}else
			Crank_bErrToothJump = 0;
		Crank_TeethCounterReset();					// reset the counter due to the larger space between teeth
	}
	else{
		Crank_PosDiffOld = Crank_PosDiff;
		Crank_ShaftFreqHz = 0.5*Crank_ShaftFreqHz + 0.5*Crank_ShaftFreqHzRaw; // filter frequency if the tooth isnt missing
		//Crank_TeethCount++;
	}
}

/* Helper funcion for Teeth Counter reset */
void Crank_TeethCounterReset(){
	Crank_TeethCount = 1;
}

/* Teeth Counter */
void Crank_TeethCounter(){
	if (Crank_TeethCount < Crank_TeethNmbr_P)
		Crank_TeethCount = Crank_TeethCount + 1;
	else
		Crank_TeethCounterReset();
}

/* Calculation of Crankshaft angle */
void Crank_AngleCalc(){
	Crank_TeethCounter();
	Crank_Angle = 360*(Crank_TeethCount-1)/(Crank_TeethNmbr_P+Crank_MissingTeethNmbr_P);
}

/* Used to synchronise angle calculation considering twice slower rotation of camshaft */
void Crank_CamPositionSync(){
	Engine_Angle = Crank_Angle + Cam_CycleStart*360;
}


