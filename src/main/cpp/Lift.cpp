/****************************************************************************
    Description:	Implements the CLift control class.
    
    Classes:		CLift

    Project:		2021 Infinite Recharge At-Home Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Lift.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CLift Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CLift::CLift()
{
    // Create object pointers.
    m_pLiftSolenoid     = new Solenoid(nLiftSolenoid);
}

/****************************************************************************
    Description:	CLift Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CLift::~CLift()
{
    // Delete objects.  
    delete m_pLiftSolenoid;

    // Set objects to nullptrs.
    m_pLiftSolenoid     = nullptr;
}

/****************************************************************************
    Description:	Init - Called once when the robot is initially turned on.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CLift::Init()
{
    // Make sure the solenoid is extended. (When going into Auto or Teleop, will engage)
    ExtendLift(false);
}

/****************************************************************************
    Description:	ExtendArm - Extends or retracts the cylinders for the Arm.
    Arguments:		bool bExtend
    Returns:		Nothing
****************************************************************************/
void CLift::ExtendLift(bool bExtend)
{
    m_pLiftSolenoid->Set(bExtend);
}

/****************************************************************************
    Description:	IsExtended
    Arguments:		None
    Returns:		bool
****************************************************************************/
bool CLift::IsExtended()
{
    return m_pLiftSolenoid->Get();
}
///////////////////////////////////////////////////////////////////////////////