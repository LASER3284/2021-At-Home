/****************************************************************************
    Description:	Implements the CTurret control class.

    Classes:		CTurret

    Project:		2021 Infinite Recharge Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace ctre;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CTurret Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CTurret::CTurret()
{
    // Create Object Pointers.
    m_pTurretMotor			= new WPI_TalonSRX(nTurretMotor);
    m_pTimer				= new Timer();
    m_pPIDController		= new frc2::PIDController(dTurretProportional, dTurretIntegral, dTurretDerivative);
}

/****************************************************************************
    Description:	CTurret Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CTurret::~CTurret()
{
    // Delete objects.
    delete m_pTurretMotor;
    delete m_pTimer;
    delete m_pPIDController;
    // Set objects to nullptrs.
    m_pTurretMotor		= nullptr;
    m_pTimer			= nullptr;
    m_pPIDController	= nullptr;
}

/****************************************************************************
    Description:	Initialize Turret parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CTurret::Init()
{
    // Initialize member variables.
    m_nState						= eTurretIdle;
    m_bIsReady						= true;
    m_bMotionMagic					= false;
    m_bHasOverrun                   = false;
    m_dActual						= (m_pTurretMotor->GetSelectedSensorPosition() + nTurretZeroOffset) / nTurretPulsesPerRev / dTurretRevsPerUnit;
    m_dFakeActual                   = 0.00;
    m_dSetpoint						= 0.00;
    m_dTolerance					= 5.00;
    m_dMaxFindingTime				= 4.50;
    m_dFindingStartTime				= 0.00;

    // Set up the feedback device for an analog encoder.
    m_pTurretMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute);
    // Set encoder to not wrap around.
    m_pPIDController->DisableContinuousInput();
    // Set the encoder and motor as both positive.
    m_pTurretMotor->SetInverted(false);
    m_pTurretMotor->SetSensorPhase(false);
    // Set the peak (maximum) motor output for both directions.
    m_pTurretMotor->ConfigPeakOutputForward(1.00);
    m_pTurretMotor->ConfigPeakOutputReverse(-1.00);
    // Set the tolerance.
    SetTolerance(m_dTolerance);
    // Set the PID and feed forward values.
    SetPID(dTurretProportional, dTurretIntegral, dTurretDerivative);
    // Stop the motor.
    Stop();
    // Set the neutral mode to brake.
    m_pTurretMotor->SetNeutralMode(NeutralMode::Brake);
    // Set acceleration (seconds from neutral to full output).
    m_pTurretMotor->ConfigOpenloopRamp(dTurretOpenLoopRamp);
    // Clear the sticky faults in memory.
    m_pTurretMotor->ClearStickyFaults();
    // Start the timer.
    m_pTimer->Start();
}

/****************************************************************************
    Description:	Main method that calls functionality, to be used in a loop.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CTurret::Tick()
{
    m_dActual = (m_pTurretMotor->GetSelectedSensorPosition() + (double)nTurretZeroOffset) / nTurretPulsesPerRev / dTurretRevsPerUnit;
    m_dFakeActual = (SmartDashboard::GetNumber("Target Angle", 0.0));

    switch(m_nState)
    {
        case eTurretIdle :
            // Idle - Do nothing.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Idle");
            m_pPIDController->Reset();
            m_pTurretMotor->Set(ControlMode::PercentOutput, 0.0);
            break;

        case eTurretStopped :
            // Stopped - Do nothing, but also set the speed to zero.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Stopped");
            m_pPIDController->Reset();
            m_pTurretMotor->Set(ControlMode::PercentOutput, 0.0);
            // Move back to Idle since everything should be stopped.
            m_nState = eTurretIdle;
            break;

        case eTurretFinding :
            // Finding - Go to a given setpoint on the Turret.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Finding");
            m_pTurretMotor->Set(ControlMode::PercentOutput, m_pPIDController->Calculate(m_dActual));
            if (IsAtSetpoint())
            {
                // At setpoint, return to Idle.
                SetState(eTurretIdle);
            }
            break;

        case eTurretTracking :
            // Tracking - Uses a setpoint of zero and an actual from the Camera to center on the target.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Tracking");
            if (!m_bHasOverrun)
            {
                m_pTurretMotor->Set(ControlMode::PercentOutput, m_pPIDController->Calculate(-m_dFakeActual));
            }
            break;
            
        case eTurretManualFwd :
            // ManualFwd - Manually move the motor forward at a constant speed.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Manual Fwd");
            m_pTurretMotor->Set(ControlMode::PercentOutput, dTurretManualFwdSpeed);
            break;
        
        case eTurretManualRev :
            // ManualRev - Manually move the motor backwards at a constant speed.
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "Manual Rev");
            m_pTurretMotor->Set(ControlMode::PercentOutput, dTurretManualRevSpeed);
            break;

        default :
            // Set state on SmartDashboard.
            SmartDashboard::PutString("Turret State", "ERROR");
            break;
    }

    SmartDashboard::PutNumber("Turret Encoder", m_pTurretMotor->GetSelectedSensorPosition());
    SmartDashboard::PutNumber("Turret Actual", m_dActual);
    SmartDashboard::PutNumber("Turret Setpoint", m_dSetpoint);
    SmartDashboard::PutBoolean("Has Turret Overran", m_bHasOverrun);
}

/****************************************************************************
    Description:	Set the variables of the PID controller for position.
    Arguments: 		double dProportional
                    double dIntegral
                    double dDerivative
                    double dFeedForward
    Returns: 		Nothing
****************************************************************************/
void CTurret::SetPID(double dProportional, double dIntegral, double dDerivative)
{
    // Configure PID controller to use the given values.
    m_pPIDController->SetPID(dProportional, dIntegral, dDerivative);
}

/****************************************************************************
    Description:	Set the setpoint of the turret's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CTurret::SetSetpoint(double dSetpoint)
{
    // Check the bounds of the setpoint. Change if neccessary.
    if (dSetpoint > dTurretMaxPosition)
    {
        dSetpoint = dTurretMaxPosition;
    }
    if (dSetpoint < dTurretMinPosition)
    {
        dSetpoint = dTurretMinPosition;
    }

    // Set the member variable.
    m_dSetpoint = dSetpoint;

    // Give the PID controller a setpoint.
    m_pPIDController->SetSetpoint(m_dSetpoint * nTurretPulsesPerRev * dTurretRevsPerUnit);

    // Start the timer for beginning finding.
    m_dFindingStartTime = m_pTimer->Get();

    // Set Turret state to finding.
    SetState(eTurretFinding);
}

/****************************************************************************
    Description:	Set the setpoint of the turret's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CTurret::SetVision(bool bEnabled)
{
    if (m_bHasOverrun)
    {
        SetSetpoint(0.0);
    }
    else
    {
        if (bEnabled)
        {
            // Set the setpoint to zero.
            m_pPIDController->SetSetpoint(0.0);

            // Set Turret state to Tracking.
            SetState(eTurretTracking);
        }
        else
        {
            // Return to Stopped.
            SetState(eTurretStopped);
        } 
    }
}

/****************************************************************************
    Description:	Set the State Machine to the given state.
    Arguments: 		TurretState nState - Given state in state machine
    Returns: 		Nothing
****************************************************************************/
void CTurret::SetState(TurretState nState)
{
    // Set member variable.
    m_nState = nState;
}

/****************************************************************************
    Description:	Sets the tolerance to be used in PID controller.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CTurret::SetTolerance(double dTolerance)
{
    // Set member variable.
    m_dTolerance = dTolerance;
    m_pPIDController->SetTolerance(dTolerance * nTurretPulsesPerRev * dTurretRevsPerUnit);
}

/****************************************************************************
    Description:	Stops the turret at it's position, resets state to idle.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CTurret::Stop()
{
    // Disable the PID controller to stop any movement.
    m_pPIDController->Reset();
    // Stop the motor.
    m_pTurretMotor->Set(ControlMode::PercentOutput, 0.00);
    // Set the state to idle.
    SetState(eTurretIdle);
}

/****************************************************************************
    Description:	Checks to see if Turret is within tolerance of the setpoint.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CTurret::IsAtSetpoint()
{
    return (m_pPIDController->AtSetpoint());
}

/****************************************************************************
    Description:	Checks to see if Turret is outside of the zone, and resets
                    to zero.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CTurret::CheckOverrun()
{
    // If the turret has overran past the maximums and minimums
    if (m_dActual < dTurretMinPosition)
    {
//      m_nState = eTurretManualFwd;
        m_bHasOverrun = true;
    }
    else
    {
        if (m_dActual > dTurretMaxPosition)
        {
//          m_nState = eTurretManualRev;
            m_bHasOverrun = true;
        }
        else
        {
//          m_nState = eTurretIdle;
            m_bHasOverrun = false;
        }
    }

    return m_bHasOverrun;
}
/////////////////////////////////////////////////////////////////////////////
