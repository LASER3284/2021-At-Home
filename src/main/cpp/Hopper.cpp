/****************************************************************************
    Description:	Implements the CHopper control class.

    Classes:		CHopper

    Project:		2021 Infinite Recharge At-Home Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Hopper.h"

using namespace ctre::phoenix;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CHopper Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHopper::CHopper()
{
    // Create Object Pointers.
    m_pShooterFeeder	= new motorcontrol::can::TalonFX(nPreloadMotor);
}

/****************************************************************************
    Description:	CHopper Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHopper::~CHopper()
{
    // Delete objects.
    delete m_pShooterFeeder;

    // Set objects to nullptrs.
    m_pShooterFeeder	= nullptr;
}

/****************************************************************************
    Description:	Initialize Hopper parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CHopper::Init()
{
    // Turn off shooter preload.
    m_pShooterFeeder->Set(motorcontrol::ControlMode::PercentOutput, 0.00);
}

/****************************************************************************
    Description:	Start feeding Energy through the hopper.
    Arugments: 		bool - True for start, false for stop.
    Returns: 		Nothing
****************************************************************************/
void CHopper::Preload(bool bEnabled)
{
    if (bEnabled)
    {
        // Start preloading into the shooter.
        m_pShooterFeeder->Set(motorcontrol::ControlMode::PercentOutput, dHopperPreloadSpeed);
    }
    else
    {
        // Stop the preloader.
        m_pShooterFeeder->Set(motorcontrol::ControlMode::PercentOutput, 0.00);
    }
}
/////////////////////////////////////////////////////////////////////////////
