/*
 * ignition.h
 *
 *  Created on: Sep 21, 2020
 *      Author: Aljaž Jelen
 */
#include "stm32l0xx_hal.h"

#ifndef INC_IGNITION_H_
#define INC_IGNITION_H_

extern int Ignition_DwellTimeUs;	// Dwell Time shall be kept minimum 1.5ms
extern int *pAwellTimeUs;			// Pointer to Dwell Time if needed
extern int Ignition_AngleDegree;	// Angle degree to firing of cylinder from a given point.
extern int *pAngleDegree;

enum Ignition_CoilStates {
    IDLE = 0,
    DWELL = 1,
    FIRE = 2
};

extern uint8_t Ignition_CoilStateCyl1;
extern uint8_t Ignition_CoilStateCyl2;
extern uint8_t Ignition_CoilStateCyl3;
extern uint8_t Ignition_CoilStateCyl4;

extern uint32_t Ignition_TimeToFire;	// Time in us to firing a cylinder

extern int Ignition_FireToothCyl1;	// Tooth at which the counter starts to count to align to correct angle for firing Cyl 1
extern int Ignition_FireToothCyl2;	// Tooth at which the counter starts to count to align to correct angle for firing Cyl 2
extern int Ignition_FireToothCyl3;	// Tooth at which the counter starts to count to align to correct angle for firing Cyl 3
extern int Ignition_FireToothCyl4;	// Tooth at which the counter starts to count to align to correct angle for firing Cyl 4

extern int Ignition_DwellToothCyl1;	// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 1
extern int Ignition_DwellToothCyl2;	// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 2
extern int Ignition_DwellToothCyl3;	// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 3
extern int Ignition_DwellToothCyl4;	// Tooth at which the counter starts to count to align to correct angle for dwelling Cyl 4

extern void Ignition_SetIgnitionTiming(int CylId, int AngleOfIgnition, int CrankShaftHz, uint8_t CrankTeethNmbr, uint8_t CrankMissingTeethNmbr);
extern void Ignition_StartTimerFireCylinder(int CylId);	// Release Dwelling and Fire the spar
extern void Ignition_StartTimerDwellCylinder(int CylId, int dwellStartInUs);

extern uint32_t Ignition_AngleToUs(int CrankShaftHz, float AngleDegree);

#endif /* INC_IGNITION_H_ */
