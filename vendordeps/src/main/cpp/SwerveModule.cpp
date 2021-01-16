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
CSwerveModule::CSwerveModule(rev::CANSparkMax *pDriveMotor, rev::CANSparkMax *pAzimuthMotor, frc::AnalogPotentiometer *pPot, double dDegreeOffset) : frc::SwerveModuleState()
{
    m_pDriveMotor = pDriveMotor;
    m_pAzimuthMotor = pAzimuthMotor;
    m_pPot = pPot;
    m_dOffset = dDegreeOffset;
    m_pPIDController = new frc2::PIDController(0.012, 0.0, 0.00003);
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
    delete m_pPIDController;

    m_pDriveMotor = nullptr;
    m_pAzimuthMotor = nullptr;
    m_pPot = nullptr;
    m_pPIDController = nullptr;
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
    m_pPIDController->EnableContinuousInput(-180, 180);
    m_pPIDController->SetTolerance(1);
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
        if (m_pPIDController->AtSetpoint())
        {
            // Stop the motor and set the current state to eIdle.
            Stop();
        }
        else
        {
            // Continue to the setpoint.
            m_pAzimuthMotor->Set(m_pPIDController->Calculate(GetAngle()));
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

    // Update the variables from SwerveModuleState every tick.
    // TODO: The velocity needs to be calculated properly, do this!
    this->speed = (units::feet_per_second_t)m_pDriveMotor->GetEncoder().GetVelocity();
    this->angle = *new frc::Rotation2d((units::radian_t)(GetAngle() * (3.1415 / 180)));
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
    // If the setpoint is 0, then skip ignore the setpoint to
    // prevent jumping back to 0 when letting go of the stick.
    if (dAngle == 0)
    {
        // Idle to make sure azimuth stops.
        SetState(eIdle);
    }
    else
    {
        // Set the setpoint of the PID controller and the variable.
        m_dAngleSetpoint = dAngle;
        m_pPIDController->SetSetpoint(m_dAngleSetpoint);
        // Run calculate once to make sure the PID controller knows it isn't at setpoint.
        m_pPIDController->Calculate(GetAngle());
        // Change state to eFinding to move to setpoint.
        SetState(eFinding);
    }
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
    // Subtract 180 to get the encoder in a -180/180 range.
    return (m_pPot->Get() + m_dOffset) - 180.0;
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
/////////////////////////////////////////////////////////////////////////////