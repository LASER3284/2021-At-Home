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
	m_pAutonomousChooser		= new SendableChooser<string>();

	// Initialize variables.
	m_nTeleopState 		= eTeleopStopped;
	m_nAutoState		= eAutoStopped;
	m_dStartTime		= 0.0;
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
	delete m_pAutonomousChooser;

	m_pTimer				= nullptr;
	m_pDriveController  	= nullptr;
	m_pRobotDrive			= nullptr;
	m_pAutonomousChooser	= nullptr;
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
	// Initialize classes.
	m_pRobotDrive->Init();

	// Put autonomous modes on the dashboard.
	m_pAutonomousChooser->SetDefaultOption("Autonomous Idle", "Autonomous Idle");
    m_pAutonomousChooser->AddOption("Barrel Path", "Barrel Path");
	m_pAutonomousChooser->AddOption("Slalom Path", "Slalom Path");
	m_pAutonomousChooser->AddOption("Bounce Path", "Bounce Path");
	m_pAutonomousChooser->AddOption("Basic Path", "Basic Path");
    m_pAutonomousChooser->AddOption("Test Path", "Test Path");
	m_pAutonomousChooser->AddOption("Just Sing a Song", "Just Sing a Song");
	SmartDashboard::PutData(m_pAutonomousChooser);

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
	// Initialize/reinitialize classes.
	m_pRobotDrive->Init();

	// Disable joystick input.
	m_pRobotDrive->SetJoystickControl(false);

	// Get the selected auto mode from SmartDashboard.
	int m_nSelectedTrajectory = -1;
	string m_strAutonomousSelected = m_pAutonomousChooser->GetSelected();
	if (m_strAutonomousSelected == "Autonomous Idle")
	{
		m_nAutoState = eAutoStopped;
	}
	if (m_strAutonomousSelected == "Barrel Path")
	{
		m_nAutoState = eAutoBarrelPath1;
		m_nSelectedTrajectory = eBarrelPath;
	}
	if (m_strAutonomousSelected == "Slalom Path")
	{
		m_nAutoState = eAutoSlalomPath1;
		m_nSelectedTrajectory = eSlalomPath;
	}
	if (m_strAutonomousSelected == "Bounce Path")
	{
		m_nAutoState = eAutoBouncePath1;
		m_nSelectedTrajectory = eBouncePath;
	}
	if (m_strAutonomousSelected == "Basic Path")
	{
		m_nAutoState = eAutoBasicPath1;
		m_nSelectedTrajectory = eBasicPath;
	}
	if (m_strAutonomousSelected == "Just Sing a Song")
	{
		m_nAutoState = eSingASong;
	}

	// If the autonomous state is not eAutoIdle, then set the selected trajectory in the drive class.
	if (m_nAutoState != eAutoStopped)
	{
		// Select trajectory.
		m_pRobotDrive->SetSelectedTrajectory(m_nSelectedTrajectory);

		// Reset odometry.
		m_pRobotDrive->ResetOdometry();

		// Store start time.
		m_dStartTime = m_pTimer->Get();
	}
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
	double dElapsedTime = m_pTimer->Get() - m_dStartTime;

	// Autonomous state machine.
	switch (m_nAutoState)
	{
		case eAutoStopped :
			// Do stopping things here.
			m_pRobotDrive->Stop();
			// Move to auto idle state.
			m_nAutoState = eAutoIdle;

		case eAutoIdle :
			// Do nothing.
			break;

		case eAutoBarrelPath1 :
			// Follow the trajectory.
			m_pRobotDrive->FollowTrajectory(dElapsedTime);

			// Move to the next state when the robot is done path following.
			if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
			{
				m_nAutoState = eAutoStopped;
			}
			break;

		case eAutoSlalomPath1 :
			// Follow the trajectory.
			m_pRobotDrive->FollowTrajectory(dElapsedTime);

			// Move to the next state when the robot is done path following.
			if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
			{
				m_nAutoState = eAutoStopped;
			}
			break;

		case eAutoBouncePath1 :
			// Follow the trajectory.
			m_pRobotDrive->FollowTrajectory(dElapsedTime);

			// Move to the next state when the robot is done path following.
			if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
			{
				m_nAutoState = eAutoStopped;
			}
			break;

		case eAutoBasicPath1 :
			// Follow the trajectory.
			m_pRobotDrive->FollowTrajectory(dElapsedTime);

			// Move to the next state when the robot is done path following.
			if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
			{
				m_nAutoState = eAutoStopped;
			}
			break;

		case eAutoTestPath1 :
			// Follow the trajectory.
			m_pRobotDrive->FollowTrajectory(dElapsedTime);

			// Move to the next state when the robot is done path following.
			if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
			{
				m_nAutoState = eAutoStopped;
			}
			break;

		case eSingASong :
			// Sing a song...
			break;
	}

	// Call drive tick.
	m_pRobotDrive->Tick();
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
	// Enable joystick control.
	m_pRobotDrive->SetJoystickControl(true);
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

	/*************************************************************************
		Drive Controller - Follow predeterimed path from location. (A Button)
	*************************************************************************/
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

	/*************************************************************************
		Drive Controller - Reset the odometry. (B Button)
	*************************************************************************/
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
			if ((m_pTimer->Get() - dTrajectoryStartTime) < m_pRobotDrive->GetTrajectoryTotalTime() + 5.0)
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
	// Stop the drive motors.
	m_pRobotDrive->Stop();
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
