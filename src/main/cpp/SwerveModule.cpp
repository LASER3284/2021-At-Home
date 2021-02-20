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

using namespace ctre::phoenix;
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
CSwerveModule::CSwerveModule(motorcontrol::can::TalonFX* pDriveMotor, motorcontrol::can::TalonFX* pAzimuthMotor, sensors::CANCoder* pEncoder, double dDegreeOffset) : frc::SwerveModuleState()
{
    m_pDriveMotor = pDriveMotor;
    m_pAzimuthMotor = pAzimuthMotor;
    m_pEncoder = pEncoder;
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
    delete m_pEncoder;
    delete m_pAnglePIDController;

    m_pDriveMotor = nullptr;
    m_pAzimuthMotor = nullptr;
    m_pEncoder = nullptr;
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
    m_pDriveMotor->ConfigSelectedFeedbackSensor(motorcontrol::FeedbackDevice::IntegratedSensor);
    m_pDriveMotor->Config_kF(0, m_dDriveFeedForward);
    m_pDriveMotor->Config_kP(0, m_dDriveProportional);
    m_pDriveMotor->Config_kI(0, m_dDriveIntegral);
    m_pDriveMotor->Config_kD(0, m_dDriveDerivative);
    m_pEncoder->SetPositionToAbsolute();
    m_pEncoder->ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
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
        m_pAzimuthMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
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
            m_pAzimuthMotor->Set(motorcontrol::ControlMode::PercentOutput, m_pAnglePIDController->Calculate(GetAngle()));
        }
        break;

    case eManualForward:
        // Manually move position forward.
        m_pAzimuthMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.1);
        m_bReady = false;
        break;

    case eManualReverse:
        // Manually move position backwards.
        m_pAzimuthMotor->Set(motorcontrol::ControlMode::PercentOutput, -0.1);
        m_bReady = false;
        break;

    default:
        break;
    }

    // Update module state.
    this->speed = (units::meters_per_second_t)GetSpeed();
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
    // Get the encoder value, divide it by the sensor resolution to get a 0-1 value.
    // Multiply it by 360 to get degrees, add the module's offset, and flip it by 180.
    return (m_pEncoder->GetAbsolutePosition() + m_dOffset);
    std::cout << "RAW: " << m_pEncoder->GetPosition() << std::endl;
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
    return (m_pDriveMotor->GetSelectedSensorVelocity() / m_dEncoderTicksPerRev) * m_dEncoderConvert;
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
    m_pAzimuthMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
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
    m_pDriveMotor->Set(motorcontrol::ControlMode::Velocity, speedOutput / m_dEncoderConvert * m_dEncoderTicksPerRev);
    // Set the setpoint of the angle controller.
    SetAngle(angleOutput);
    SetSpeed(speedOutput);
}
/////////////////////////////////////////////////////////////////////////////