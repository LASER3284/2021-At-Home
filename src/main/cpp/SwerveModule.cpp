/****************************************************************************  
 *	Implements the SwerveModule class.
 *
 *	Classes:		CSwerveModule
 *
 *	Project:		Swerve Drive
 *
 * 	Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
 ***************************************************************************/

#include "SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**  
 *	@brief          CSwerveDrive Constructor.
 *
 *	@param          pDriveMotor pointer to drive motor.
 *  @param          pAzimuthMotor pointer to the azimuth motor.
 *  @param          pPot pointer to the analog encoder, used as a
 *                  potentiometer for compatibility.
 *  @param          dDegreeOffset value of the offset of the azimuth
 *                  encoder relative to zero.
 * 
 *	@implements     frc::SwerveModuleState
 ***************************************************************************/
CSwerveModule::CSwerveModule(rev::CANSparkMax *pDriveMotor, rev::CANSparkMax *pAzimuthMotor, frc::AnalogInput *pPot, double dDegreeOffset) : frc::SwerveModuleState()
{
    m_pDriveMotor = pDriveMotor;
    m_pAzimuthMotor = pAzimuthMotor;
    m_pPot = pPot;
    m_dOffset = dDegreeOffset;
    m_pAnglePIDController = new frc2::PIDController(m_dAngleProportional, m_dAngleIntegral, m_dAngleDerivative);
    m_dAngleSetpoint = 0.0;
}

/************************************************************************//**  
 *	@brief          CSwerveDrive Destructor.
 *
 *  @param          None 
 * 
 *  @implements     SwerveModuleState
 ***************************************************************************/
CSwerveModule::~CSwerveModule()
{
    delete m_pDriveMotor;
    delete m_pAzimuthMotor;
    delete m_pPot;
    delete m_pAnglePIDController;

    m_pDriveMotor = nullptr;
    m_pAzimuthMotor = nullptr;
    m_pPot = nullptr;
    m_pAnglePIDController = nullptr;
}

/************************************************************************//**  
 *	@brief	        Swerve Module initialization routine.
 *
 *	@param          None
 *
 *	@retval         Nothing
 ***************************************************************************/
void CSwerveModule::Init()
{
    // Make sure PID input is expecting -180/180.
    m_pAnglePIDController->EnableContinuousInput(-180, 180);
    m_pAnglePIDController->SetTolerance(1);
    m_pDriveMotor->GetEncoder().SetVelocityConversionFactor(((1 / (8.31)) / 60) * (3.1415926  * (4 * 0.0254)));
    m_pDriveMotor->GetPIDController().SetP(m_dDriveProportional);
    m_pDriveMotor->GetPIDController().SetI(m_dDriveIntegral);
    m_pDriveMotor->GetPIDController().SetD(m_dDriveDerivative);
}

/************************************************************************//**  
 *	@brief	        Swerve Module looped implementation routine.
 *
 *	@param          None
 *
 *	@retval         Nothing
 ***************************************************************************/
void CSwerveModule::Tick()
{
    // State machine.
    switch (m_nState)
    {
    case eIdle:
        // Stop the motor.
        m_pAzimuthMotor->Set(0.0);
        m_bReady = true;
        break;

    case eFinding:
        // If the state is eFinding, the motor will continue until
        // the PID reaches the target, then the state becomes idle.
        m_bReady = false;
        // Check to see if position is within tolerance.
        if (m_pAnglePIDController->AtSetpoint())
        {
            // Stop the motor and set the current state to eIdle.
            Stop();
        }
        else
        {
            // Continue to the setpoint.
            m_pAzimuthMotor->Set(m_pAnglePIDController->Calculate(GetAngle()));
        }
        break;

    case eManualForward:
        // Manually move position forward.
        m_pAzimuthMotor->Set(0.1);
        m_bReady = false;
        break;

    case eManualReverse:
        // Manually move position backwards.
        m_pAzimuthMotor->Set(-0.1);
        m_bReady = false;
        break;

    default:
        break;
    }

    // Update module state.
    this->speed = (units::meters_per_second_t)(m_pDriveMotor->GetEncoder().GetVelocity());
    this->angle = *new frc::Rotation2d(units::degree_t(GetAngle()));
}

/************************************************************************//**  
 *	@brief	        Set azimuth angle in degrees.
 *
 *	@param          dAngle angle desired in degrees.
 *
 *	@retval        Nothing
 ***************************************************************************/
void CSwerveModule::SetAngle(double dAngle)
{
    // Set the setpoint of the PID controller and the variable.
    m_dAngleSetpoint = dAngle;
    m_pAnglePIDController->SetSetpoint(m_dAngleSetpoint);
    // Run calculate once to make sure the PID controller knows it isn't at setpoint.
    m_pAnglePIDController->Calculate(GetAngle());
    // Change state to eFinding to move to setpoint.
    SetState(eFinding);
}

/************************************************************************//**  
 *	@brief	        Return the current azimuth angle in degrees.
 *
 *	@param          None
 *
 *	@retval         double angle in degrees.
 ***************************************************************************/
double CSwerveModule::GetAngle()
{
    // Get the voltage, divide it by 5 to get a 0-1 value.
    // Multiply it by 360 to get degrees, add the module's offset, and flip it by 180.
    return (((-m_pPot->GetVoltage() / 5.0) * 360.0) + m_dOffset) - 180.0;
}

/************************************************************************//**  
 *	@brief          Returns the setpoint for the azimuth in degrees.
 *
 *	@param          None
 *
 *	@retval        double angle in degrees.
 ***************************************************************************/
double CSwerveModule::GetAngleSetpoint()
{
    return m_dAngleSetpoint;
}

/************************************************************************//**  
 *	@brief	        Sets the speed of the drive motor in percent.
 *
 *	@param          dSpeed desired speed in percent.
 *
 *	@retval        Nothing
 ***************************************************************************/
void CSwerveModule::SetSpeed(double dSpeed)
{
    // Store variable.
    m_dSpeedSetpoint = dSpeed;
    m_pDriveMotor->Set(m_dSpeedSetpoint);
}

/************************************************************************//**  
 *	@brief	        Returns the current speed of the drive motor in percent.
 *
 *	@param          None
 *
 *  @retval        double speed in percent.
 ***************************************************************************/
double CSwerveModule::GetSpeed()
{
    return m_pDriveMotor->Get();
}

/************************************************************************//**  
 *	@brief	        Returns the desired speed of the drive motor in percent.
 *
 *	@param          None
 *
 *	@retval        double speed in percent.
 ***************************************************************************/
double CSwerveModule::GetSpeedSetpoint()
{
    return m_dSpeedSetpoint;
}

/************************************************************************//**  
 *	@brief	        Set the serve module drive motor reversed.
 *
 *	@param          bool bIsReversed
 *
 *	@retval         Nothing
 ***************************************************************************/
void CSwerveModule::SetModuleReversed(bool bIsReversed)
{
    m_pDriveMotor->SetInverted(bIsReversed);
}

 /***********************************************************************//** 
 *	@brief	        Stops the azimuth motor and returns the state machine to
 *                  idle.
 *
 *	@param		    None
 *
 *	@retval        Nothing
 ***************************************************************************/
void CSwerveModule::Stop()
{
    // Make sure the motor is stopped, then move to idle to keep it stopped.
    m_pAzimuthMotor->StopMotor();
    m_nState = eIdle;
}

/************************************************************************//**
 *	@brief	        Sets the state of the state machine.
 *
 *	@param          nState the state desired
 *
 *	@retval        Nothing
 ***************************************************************************/
void CSwerveModule::SetState(int nState)
{
    m_nState = nState;
}

/************************************************************************//**
 *	@brief	        Returns the current state of the state machine.
 *
 *	@param          None
 *
 *	@retval        int current state of the state
 ***************************************************************************/
int CSwerveModule::GetState()
{
    return m_nState;
}

/************************************************************************//**
 *	@brief	        Returns the current state of the swerve module.
 *
 *	@param          None
 *
 *	@retval         SwerveModuleState Current swerve module state.
 ***************************************************************************/
frc::SwerveModuleState CSwerveModule::GetModuleState()
{
    return SwerveModuleState{this->speed, this->angle};
}

/************************************************************************//**
 *	@brief	        Sets the SwerveModuleState state of the module.
 *
 *	@param          None
 *
 *	@retval         SwerveModuleState Current swerve module state.
 ***************************************************************************/
void CSwerveModule::SetModuleState(frc::SwerveModuleState desiredState)
{
    // Optimize the desired state, it should never rotate more than 90 degrees.
    desiredState = this->Optimize(desiredState, units::degree_t(GetAngle()));
    // Convert angle and speed to double setpoints.
    double speedOutput = (desiredState.speed.to<double>());
    double angleOutput = desiredState.angle.Degrees().to<double>();
    // Set the speed of the drive.
    m_pDriveMotor->GetPIDController().SetReference(speedOutput, rev::ControlType::kVelocity);
    // Set the setpoint of the angle controller.
    SetAngle(angleOutput);
}
/////////////////////////////////////////////////////////////////////////////