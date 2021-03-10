/****************************************************************************
    Description:	Defines the CLift control class.

    Classes:		CLift

    Project:        2021 Infinite Recharge At-Home Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Lift_h
#define Lift_h

#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "IOMap.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CLift class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CLift
{
public:
    CLift();
    ~CLift();
    void Init();
    void ExtendLift(bool bExtend = true);
    bool IsExtended();

private:
    // Object Pointers.
    Solenoid*       m_pLiftSolenoid;
};
/////////////////////////////////////////////////////////////////////////////
#endif