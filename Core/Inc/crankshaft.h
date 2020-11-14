/*
 * crankshaft.h
 *
 *  Created on: Oct 4, 2020
 *      Author: aljaz
 */
#include "stm32l0xx_hal.h"

#ifndef INC_CRANKSHAFT_H_
#define INC_CRANKSHAFT_H_


extern int 		Crank_ShaftFreqHz;
extern int 		Crank_ShaftFreqHzRaw;
extern uint32_t Crank_PosDiff;		// Global Variable to track Crankshaft Tooth New timing difference
extern uint32_t Crank_PosDiffOld;	// Global Variable to track Crankshaft Tooth Old timing difference
extern uint8_t 	Crank_TeethCount; 	// Global Variable of exact Current Tooth Number
extern uint32_t Crank_RotDelta;
extern uint16_t Crank_Angle;
extern uint32_t Crank_DivAngle;		// Global Variable used to calculate angle between 2 rising edges of the tooth

extern uint8_t	Cam_CycleStart;

// Locals
extern uint16_t Crank_LastCapturedEdgeTime;
extern uint8_t 	Crank_bErrToothJump;

// Parameters
extern uint8_t 	Crank_TeethNmbr_P;
extern uint8_t 	Crank_MissingTeethNmbr_P;

// Functions
extern void Crank_HalGeberDriver(TIM_HandleTypeDef *htim);
extern void Crank_CamPositionSync();	// TODO used to synchronise with CAM, to see which halfcycle is it
extern void Crank_TeethCounterReset();
extern void Crank_AngleCalc();

#endif /* INC_CRANKSHAFT_H_ */
