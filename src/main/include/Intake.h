/****************************************************************************
    Description:	Defines the CIntake control class.
    Classes:		CIntake
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Intake_h
#define Intake_h

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Intake Contants.
const double dIntakeFwdSpeed	=  1.00;
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
    double GetIntakeCurrent();

private:
    // Object pointers. 
    // WPI_TalonSRX*		m_pIntakeMotor;
    Solenoid*			m_pIntakeActuator;
    frc::Timer*         m_pTimer;

    // Declare variables.
};
/////////////////////////////////////////////////////////////////////////////
#endif
