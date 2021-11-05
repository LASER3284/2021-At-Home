/****************************************************************************
    Description:	Implements the CIntake control class.

    Classes:		CIntake

    Project:		2021 Infinite Recharge At-Home Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Intake.h"

using namespace frc;
using namespace rev;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CIntake Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CIntake::CIntake()
{
    // Create Object Pointers.
    m_pIntakeMotor		= new CANSparkMax(nIntakeMotor, CANSparkMax::MotorType::kBrushless);
    m_pIntakeActuator	= new Solenoid(nIntakeSolenoid);
    m_pTimer            = new Timer();

    // Initialize member variables.
    m_bIntakeEnabled    = false;
}

/****************************************************************************
    Description:	CIntake Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CIntake::~CIntake()
{
    // Delete objects.
    delete m_pIntakeMotor;
    delete m_pIntakeActuator;

    // Set objects to nullptrs.
    m_pIntakeMotor		= nullptr;
    m_pIntakeActuator	= nullptr;
}

/****************************************************************************
    Description:	Initialize Intake parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CIntake::Init()
{
    // Retract Intake Mechanism.
    Extend(false);

    // Turn off Intake Motor.
    IntakeMotor(false);
}

/****************************************************************************
    Description:	Extends or retracts the intake system.
    Arguments: 		bool bExtend - true to extend, false to retract
    Returns: 		Nothing
****************************************************************************/
void CIntake::Extend(bool bExtend)
{
    // Extend or Retract Intake Mechanism.
    m_pIntakeActuator->Set(bExtend);
}

/****************************************************************************
    Description:	Returns the state of the Intake Actuator.
    Arguments: 		None
    Returns: 		bool - True for extended, false for retracted.
****************************************************************************/
bool CIntake::GetExtended()
{
    // Returns the state of the Intake Actuator.
    return m_pIntakeActuator->Get();
}

/****************************************************************************
    Description:	Turns Intake Motor on or off.
    Arguments: 		bool bStartIntake - Enable or disable motor.
    Returns: 		Nothing
****************************************************************************/
void CIntake::IntakeMotor(bool bStartIntake)
{
    // Enable or disable intake.
    if (bStartIntake)
    {
        m_pIntakeMotor->Set(dIntakeFwdSpeed);
        m_bIntakeEnabled = true;
    }
    else
    {
        m_pIntakeMotor->Set(0.0);
        m_bIntakeEnabled = false;
    }
    
}

/****************************************************************************
    Description:	Returns if the intake is enabled.
    Arguments:		None
    Returns:		bool - bEnabled
****************************************************************************/
bool CIntake::GetIntakeMotorEnabled()
{
    return m_bIntakeEnabled;
}

/****************************************************************************
    Description:	Manually sets the intake speed.
    Arguments: 		double dSpeed - The percent output of the motor.
    Returns: 		Nothing
****************************************************************************/
void CIntake::SetSpeed(double dSpeed)
{
    // Set the motor speed.
    m_pIntakeMotor->Set(dSpeed);
}

/****************************************************************************
    Description:	Returns the current Amp draw of the motor.
    Arguments:		None
    Returns:		double - Amperage, in Amps
****************************************************************************/
double CIntake::GetIntakeCurrent()
{
    return m_pIntakeMotor->GetOutputCurrent();
}
/////////////////////////////////////////////////////////////////////////////
