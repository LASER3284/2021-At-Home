/****************************************************************************
 *	Implements the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		Swerve Drive
 ****************************************************************************/
#include "RobotMain.h"
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**
 *  @brief	        RobotMain Contructor.
 *
 *	@param          None
 *
 * 	@implements		frc::TimedRobot			
 ****************************************************************************/
CRobotMain::CRobotMain()
{
	// Create objects.
    m_pTimer                    = new frc::Timer();
	m_pDriveController  		= new frc::Joystick(0);
	m_pRobotDrive				= new CDrive(m_pDriveController);

	// Initialize variables.
	m_nTeleopState 		= eTeleopStopped;
}

/************************************************************************//**
 *  @brief	        RobotMain Destructor.
 *
 *	@param          None
 *
 * 	@implements		frc::TimedRobot			
 ****************************************************************************/
CRobotMain::~CRobotMain()
{
	delete m_pTimer;
	delete m_pDriveController;
	delete m_pRobotDrive;

	m_pTimer			= nullptr;
	m_pDriveController  = nullptr;
	m_pRobotDrive		= nullptr;
}

/************************************************************************//**
 *  @brief	        Robot initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::RobotInit()
{
	// Start the robot timer.
	m_pTimer->Start();
}

/************************************************************************//**
 *  @brief	        Robot looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::RobotPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Autonomous initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::AutonomousInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Autonomous looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::AutonomousPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Teleop initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TeleopInit()
{
	m_pRobotDrive->Init();
}

/************************************************************************//**
 *  @brief	        Robot Teleop looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TeleopPeriodic()
{
	// Create method variables.
	static double dTrajectoryStartTime = 0.0;

	/*
		// Drive Controller - Follow predeterimed path from location. (A Button)
	*/
	if (m_pDriveController->GetRawButtonPressed(eButtonA))
	{
		// Generate and follow path.
		m_nTeleopState = eTeleopGeneratePath;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonA) && (m_nTeleopState == eTeleopGeneratePath || m_nTeleopState == eTeleopFollowing))
		{
			// Stop the drive.
			m_pRobotDrive->Stop();
			// Enable joystick control.
			m_pRobotDrive->SetJoystickControl(true);

			// Move to Teleop Idle.
			m_nTeleopState = eTeleopIdle;
		}
	}

	/*
		// Drive Controller - Reset the odometry. (B Button)
	*/
	if (m_pDriveController->GetRawButtonPressed(eButtonB))
	{
		// Reset odometry.
		m_pRobotDrive->ResetOdometry();
	}
	

	// Teleop State Machine.
	switch (m_nTeleopState)
    {
        case eTeleopStopped : 
			// Do stopping things here when ready.

			// Move to teleop idle.
			m_nTeleopState = eTeleopIdle;
			break;

		case eTeleopIdle :
			// Do nothing.
			break;

		case eTeleopGeneratePath : 
			// Generate the robot path. 
			m_pRobotDrive->GenerateTrajectoryFromCurrentPosition();
			// Store the start time.
			dTrajectoryStartTime = m_pTimer->Get();
			// Move to eTeleopFolling state.
			m_nTeleopState = eTeleopFollowing;
			break;

		case eTeleopFollowing : 
			// Disable joystick control for the duration of the path.
			m_pRobotDrive->SetJoystickControl(false);

			// Follow the path if time hasn't expired.
			if ((m_pTimer->Get() - dTrajectoryStartTime) < m_pRobotDrive->GetTrajectoryTotalTime() + 30.0)
			{
				// Move to point in path at elapsed time. 
				m_pRobotDrive->FollowTrajectory(m_pTimer->Get() - dTrajectoryStartTime);
			}
			else
			{
				// Stop the drive motors. (just in case we weren't done following the trajectory)
				m_pRobotDrive->Stop();
				// Enable joystick control.
				m_pRobotDrive->SetJoystickControl(true);

				// Move to teleop idle. (Not moving to the stopped state because we only want the drive to stop.)
				m_nTeleopState = eTeleopIdle;
			}
			break;

        default :
            // Return to idle.
            m_nTeleopState = eTeleopStopped;
            break;
    }

	// Call drive tick.
	m_pRobotDrive->Tick();
}

/************************************************************************//**
 *  @brief	        Robot Disabled initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::DisabledInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Disabled looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::DisabledPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Test initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TestInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Test looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TestPeriodic()
{

}
/////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif
