/****************************************************************************
    Description:	Defines the CIntake control class.

    Classes:		CIntake

    Project:		2021 Infinite Recharge At-Home Robot Code.
    
    Copyright 2021 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Intake_h
#define Intake_h

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Intake Contants.
const double dIntakeFwdSpeed	=   0.60;
const double dIntakeRevSpeed    =   0.40;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CIntake class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CIntake
{
public:
    CIntake();
    ~CIntake();

    // Public Methods.
    void Init();
    void Extend(bool bExtend);
    bool GetExtended();
    void IntakeMotor(bool bEnabled);
    void SetSpeed(double dSpeed);
    double GetIntakeCurrent();

private:
    // Object pointers. 
    CANSparkMax*		m_pIntakeMotor;
    Solenoid*			m_pIntakeActuator;
    frc::Timer*         m_pTimer;

    // Declare variables.
};
/////////////////////////////////////////////////////////////////////////////
#endif
