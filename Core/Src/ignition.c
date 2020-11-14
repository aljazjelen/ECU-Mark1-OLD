/*
 * ignition.c
 *
 *  Created on: Sep 21, 2020
 *      Author: Aljaž Jelen
 */

#include "ignition.h"
//#include "stm32l0xx_hal.h"
int Ignition_DwellTimeUs = 1000;	// Global Variable for Dwell Time of ignition coils, calculated from maps (1000us to 2000us)
int Ignition_AngleDegree = 5;		// Ignition angle BTDC

uint32_t Ignition_TimeToFireCyl1 = 0;	// Time in us to firing a Cylinder 1
uint32_t Ignition_TimeToFireCyl2 = 0;	// Time in us to firing a Cylinder 2
uint32_t Ignition_TimeToFireCyl3 = 0;	// Time in us to firing a Cylinder 3
uint32_t Ignition_TimeToFireCyl4 = 0;	// Time in us to firing a Cylinder 4

int Ignition_FireToothCyl1 = 1;			// Tooth at which the counter starts to count to align to correct angle for firing Cyl 1
int Ignition_FireToothCyl2 = 20;		// Tooth at which the counter starts to count to align to correct angle for firing Cyl 2
int Ignition_FireToothCyl3 = 9;			// Tooth at which the counter starts to count to align to correct angle for firing Cyl 3
int Ignition_FireToothCyl4 = 40;		// Tooth at which the counter starts to count to align to correct angle for firing Cyl 4

int Ignition_DwellToothCyl1 = 1;		// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 1
int Ignition_DwellToothCyl2 = 20;		// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 2
int Ignition_DwellToothCyl3 = 9;		// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 3
int Ignition_DwellToothCyl4 = 40;		// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 4



uint8_t Ignition_CoilStateCyl1 = IDLE;
uint8_t Ignition_CoilStateCyl2 = IDLE;
uint8_t Ignition_CoilStateCyl3 = IDLE;
uint8_t Ignition_CoilStateCyl4 = IDLE;

int *pAwellTimeUs = &Ignition_DwellTimeUs;
int *pAngleDegree = &Ignition_AngleDegree;


int Ignition_DefineIgnitionTeeth(int AngleOfIgnition,uint8_t CrankTeethNmbr, uint8_t CrankMissingTeethNmbr){
	int triggerTooth = 0;
	triggerTooth = (CrankTeethNmbr+CrankMissingTeethNmbr)*AngleOfIgnition/360+1;
	return triggerTooth;
}

/**
  * @brief Ignition Function which will set the ignition timer and the tooth number at which timer has to start counting, in order to fire at exact angle.
  * @param CylId - Cylinder Number, AngleOfIgnition - Angle (0-360) at which the Coil shall fire, CrankShaftHz - Frequency of Crankshaft, CrankTeethNmbr - Number of teeth on the wheel, CrankMissingTeethNmbr - Number of missing teeth
  * @retval None
  */
void Ignition_SetIgnitionTiming(int CylId, int AngleOfIgnition, int CrankShaftHz, uint8_t CrankTeethNmbr, uint8_t CrankMissingTeethNmbr){
	int TriggerTooth;
	TriggerTooth = Ignition_DefineIgnitionTeeth(AngleOfIgnition,CrankTeethNmbr,CrankMissingTeethNmbr);

	float triggerAngle;
	float delayAngle;
	uint32_t Ignition_TimeToFire;

	// Perform correction if angle is after the missing teeth
	if (TriggerTooth == CrankTeethNmbr){
		TriggerTooth = 0;
		delayAngle = 0;
	}
	else{
		triggerAngle = (TriggerTooth-1)*360/(CrankTeethNmbr+CrankMissingTeethNmbr);
		delayAngle = AngleOfIgnition - triggerAngle;
		Ignition_TimeToFire = Ignition_AngleToUs(CrankShaftHz,delayAngle);
	}

	/* correction in case CCR and ARR dont work with 0
	 *
	 */
	if (Ignition_TimeToFire == 0)
		Ignition_TimeToFire = 1;


	switch (CylId)
	{
	case 1:
		Ignition_FireToothCyl1 = TriggerTooth;
		Ignition_TimeToFireCyl1 = Ignition_TimeToFire;
		break;
	case 2:
		Ignition_FireToothCyl2 = TriggerTooth;
		Ignition_TimeToFireCyl2 = Ignition_TimeToFire;
		break;
	case 3:
		Ignition_FireToothCyl3 = TriggerTooth;
		Ignition_TimeToFireCyl3 = Ignition_TimeToFire;
		break;
	case 4:
		Ignition_FireToothCyl4 = TriggerTooth;
		Ignition_TimeToFireCyl4 = Ignition_TimeToFire;
		break;
	}
}

uint32_t Ignition_AngleToUs(int CrankShaftHz, float AngleDegree){
	return HAL_RCC_GetPCLK1Freq()*AngleDegree/360/CrankShaftHz/TIM22->PSC;
}


void Ignition_StartTimerFireCylinder(int CylId){
	uint32_t fireStartInUs = 0;
	switch (CylId)
	{
	case 1:
		fireStartInUs = Ignition_TimeToFireCyl1;
		Ignition_CoilStateCyl1 = FIRE;
		break;
	case 2:
		fireStartInUs = Ignition_TimeToFireCyl2;
		break;
	case 3:
		fireStartInUs = Ignition_TimeToFireCyl3;
		Ignition_CoilStateCyl3 = FIRE;
		break;
	case 4:
		fireStartInUs = Ignition_TimeToFireCyl4;
		break;
	}
	// set proper GPIO to choose the cylinder (1,2,3,4)?
	// toggle the timer which will start counting to fire dwelling and then spark
	TIM22->ARR = fireStartInUs + 100;
	TIM22->CCR1 = fireStartInUs;
	TIM22->CCR2 = TIM22->ARR+10;//to disable
	TIM22->EGR = 1;
	TIM22->CR1 |= TIM_CR1_CEN;
	 // TODO only for cyl1
	//HAL_TIM_OnePulse_Start(TIM22,1);
}


void Ignition_StartTimerDwellCylinder(int CylId,int dwellStartInUs){

	switch (CylId)
	{
	case 1:
		//fireStartInUs = Ignition_TimeToFireCyl1;
		Ignition_CoilStateCyl1 = DWELL;
		break;
	case 2:
		//fireStartInUs = Ignition_TimeToFireCyl2;
		break;
	case 3:
		//fireStartInUs = Ignition_TimeToFireCyl3;
		Ignition_CoilStateCyl3 = DWELL;
		break;
	case 4:
		//fireStartInUs = Ignition_TimeToFireCyl4;
		break;
	}
	// Dwell selected ignition coil for cylinder
	//TIM22->CCR1 = TIM22->ARR+1;
	//TIM22->CCR2 = 10;
	TIM22->ARR = 100;
	TIM22->CCR1 = 10;
	TIM22->CCR2 = TIM22->ARR+10;
	TIM22->EGR = 1;
	TIM22->CR1 |= TIM_CR1_CEN;
	//Ignition_CoilStateCyl1 = DWELL;
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}


