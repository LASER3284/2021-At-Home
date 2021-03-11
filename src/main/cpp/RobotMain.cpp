/****************************************************************************
 *	Implements the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		2021 Infinite Recharge At-Home Robot Code.
 *
 *   Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
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
	m_pAuxController			= new frc::Joystick(1);
	m_pRobotDrive				= new CDrive(m_pDriveController);
	m_pIntake					= new CIntake();
	m_pTurret					= new CTurret();
	m_pShooter					= new CShooter();
	m_pHood						= new CHood();
	m_pHopper					= new CHopper();
    m_pLift                     = new CLift();
	m_pAutonomousChooser		= new SendableChooser<string>();
	m_pCompressor				= new frc::Compressor();

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
	delete m_pIntake;
	delete m_pTurret;
	delete m_pShooter;
	delete m_pHood;
	delete m_pHopper;
    delete m_pLift;
	delete m_pAutonomousChooser;

	m_pTimer				= nullptr;
	m_pDriveController  	= nullptr;
	m_pRobotDrive			= nullptr;
	m_pIntake				= nullptr;
	m_pTurret				= nullptr;
	m_pShooter				= nullptr;
	m_pHood					= nullptr;
	m_pHopper				= nullptr;
    m_pLift                 = nullptr;
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
    m_pHood->Init();
    m_pIntake->Init();
    m_pTurret->Init();
    m_pShooter->Init();
    m_pHopper->Init();
    m_pLift->Init();

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
	m_pShooter->Tick();
	m_pHood->Tick();
	m_pTurret->Tick();
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
	m_pCompressor->Start();

	// Turn off vision LEDs.
	m_pShooter->SetVisionLED(false);
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
	static bool bHasFired           = false;
    static bool bTurretMoving       = false;
    static bool bHoodMoving         = false;

    /********************************************************************
        Drive Controller - Toggle Intake (Right Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonRB))
    {
        if (m_nTeleopState == eTeleopIntake)
        {
            // Leave to idle.
            m_nTeleopState = eTeleopStopped;
        }
        else
        {
            // Start intaking.
            m_nTeleopState = eTeleopIntake;
        }
    }

	/********************************************************************
        Drive Controller - Fire (Left Trigger)
    ********************************************************************/
    if (m_pDriveController->GetRawAxis(eLeftTrigger) >= 0.65 && !m_pDriveController->GetRawButton(eButtonRB))
    {
        // Set state to Firing.
        m_nTeleopState = eTeleopFiring;
        bHasFired = true;
    }
    else
    {
        if (bHasFired)
        {
            // Has been fired, return to idle.
            m_pShooter->SetState(eShooterIdle);
            m_nTeleopState = eTeleopStopped;
            bHasFired = false;
        }
    }

    /********************************************************************
        Aux Controller - Lift Robot (Right Stick)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonRS))
    {
        // Toggle the lift of the robot.
        m_pLift->ExtendLift(!m_pLift->IsExtended());
        
        // Check for driver safety.
        if (!m_pLift->IsExtended())
        {
            // Disable driver control.
            m_pRobotDrive->SetJoystickControl(false);
        }
        else
        {
            // Enable driver control.
            m_pRobotDrive->SetJoystickControl(true);
        }
        
    }

    /********************************************************************
        Drive Controller - Close Range Fire (Left Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonLB))
    {
        // Set state to Firing.
        m_nTeleopState = eTeleopCloseRangeFiring;
        bHasFired = true;
    }
    else
    {
        if (bHasFired)
        {
            // Has been fired, return to idle.
            m_pShooter->SetState(eShooterIdle);
            m_nTeleopState = eTeleopStopped;
            bHasFired = false;
        }
    }

    /********************************************************************
        Drive Controller - AutoFire (Right Trigger + Aux Right Trigger)
    ********************************************************************/
    if ((m_pDriveController->GetRawAxis(eLeftTrigger) >= 0.65) && (m_pAuxController->GetRawAxis(eRightTrigger) >= 0.65))
    {
        // Set the state to AutoFire.
        m_nTeleopState = eTeleopAutoFiring;
    }
    // Other states related to this button will set state back to Idle.

    /********************************************************************
        Drive Controller - Manual Move Hood Up (Up POV)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 0)
    {
        // Manual move up.
        m_pHood->SetState(eHoodManualFwd);
        bHoodMoving = true;
    }
    else
    {
    /********************************************************************
        Drive Controller - Manual Move Hood Down (Down POV)
    ********************************************************************/
        if (m_pDriveController->GetPOV() == 180)
        {
            // Manual move down.
            m_pHood->SetState(eHoodManualRev);
            bHoodMoving = true;
        }
        else
        {
            if (bHoodMoving)
            {
                // No longer moving, set to idle.
                m_pHood->Stop();
                bHoodMoving = false;
            }
        }
    }

	/********************************************************************
        Drive Controller - Move Hood to Preset Far Position (Button A)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonA))
    {
        m_pHood->SetSetpoint(SmartDashboard::GetNumber("Hood Position Far", 0.0));
        m_pHood->SetState(eHoodFinding);
    }

    /********************************************************************
        Drive Controller -  Move Hood to Preset Near Position (Button Y)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonY))
    {
        m_pHood->SetSetpoint(SmartDashboard::GetNumber("Hood Position Near", 0.0));
        m_pHood->SetState(eHoodFinding);
    }

    /********************************************************************
        Drive Controller - Zero Hood Encoder (Button X)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonX))
    {
        m_pHood->Rezero();
    }

	/*************************************************************************
		Drive Controller - Follow predeterimed path from location. (Start Button)
	*************************************************************************/
	if (m_pDriveController->GetRawButtonPressed(eStart))
	{
		// Generate and follow path.
		m_nTeleopState = eTeleopGeneratePath;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eStart) && (m_nTeleopState == eTeleopGeneratePath || m_nTeleopState == eTeleopFollowing))
		{
			// Stop the drive.
			m_pRobotDrive->Stop();
			// Enable joystick control.
			m_pRobotDrive->SetJoystickControl(true);

			// Move to Teleop Idle.
			m_nTeleopState = eTeleopIdle;
		}
	}

    /********************************************************************
        Aux Controller - Manual Move Turret Left (Left Bumper)
    ********************************************************************/
    if (m_pAuxController->GetRawButton(eButtonLB))
    {
        // Manually move left.
        m_pTurret->SetState(eTurretManualRev);
        bTurretMoving = true;
    }
    else
    {
    /********************************************************************
        Aux Controller - Manual Move Turret Right (Right Bumper)
    ********************************************************************/
        if (m_pAuxController->GetRawButton(eButtonRB))
        {
            // Manually move right.
            m_pTurret->SetState(eTurretManualFwd);
            bTurretMoving = true;
        }
        else
        {
            if (bTurretMoving)
            {
                // No longer pressing any buttons, move to Idle.
                m_pTurret->SetState(eTurretIdle);
                bTurretMoving = false;
            }
        }
    }

	/********************************************************************
        Aux Controller - Vision Aiming (Right Trigger)
    ********************************************************************/
    if ((m_pAuxController->GetRawAxis(eRightTrigger) > 0.65) && !(m_pDriveController->GetRawAxis(eLeftTrigger) > 0.65))
    {
        // Set state to Aiming.
        m_nTeleopState = eTeleopAiming;
    }
    if (!(m_pAuxController->GetRawAxis(eRightTrigger) > 0.65) && !(m_pDriveController->GetRawAxis(eLeftTrigger) > 0.65))
    {
        // If released while still in aiming...
        if (m_nTeleopState == eTeleopAiming)
        {
            // Go back to idle.
            m_nTeleopState = eTeleopStopped;
        }
        // If the button was released but we didn't change states
        // yet, do nothing to prevent it from leaving it's current
        // state.
    }

    /********************************************************************
        Aux Controller - Toggle Shooter "Idle" speed (Button B)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonB))
    {
        m_pShooter->SetState(m_pShooter->GetState() == eShooterIdle ? eShooterStopped : eShooterIdle);
    }

	/*************************************************************************
		Aux Controller - Reset the odometry. (Start Button)
	*************************************************************************/
	if (m_pAuxController->GetRawButtonPressed(eStart))
	{
		// Reset odometry.
		m_pRobotDrive->ResetOdometry();
	}
	

	// Teleop State Machine.
	switch (m_nTeleopState)
    {
        case eTeleopStopped : 
			// Disable LEDs
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(false);
            // Return intake to it's retracted state.
            m_pIntake->Extend(false);
            // m_pIntake->IntakeMotor(false);
            // Enable joystick control.
            m_pRobotDrive->SetJoystickControl(true);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Idle the Hood, Turret, and Hopper.
            m_pHood->SetState(eHoodReset);
            // m_pTurret->Stop();
            m_pHopper->Preload(false);

			// Move to teleop idle.
			m_nTeleopState = eTeleopIdle;
			break;

		case eTeleopIdle :
			// Do nothing.
			break;

		case eTeleopIntake :
            /********************************************************************
                Intake - Robot is intaking Energy.
            ********************************************************************/
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake on a half second delay.
            if ((m_pTimer->Get() - m_dStartTime) >= 0.5)
            {
                // m_pIntake->IntakeMotor(true);
            }
            // Stop Shooter, stop Turret, and stop Hood.
            m_pShooter->Stop();
            m_pTurret->Stop();
            m_pHopper->Preload(false);
            break;

        case eTeleopAiming :
            /********************************************************************
                Aiming - Turret is tracking the position of the high goal
                         using the Vision points determined.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Set the Turret to tracking mode.
            m_pTurret->SetVision(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            // Idle shooter.
            m_pShooter->SetSetpoint(dShooterIdleVelocity);
            // Stop Preloader.
            m_pHopper->Preload(false);
            // Set the Hood to tracking mode.
            m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
            break;
        
        case eTeleopFiring :
            /********************************************************************
                Firing - Robot simply fires wherever it is currently aiming.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Don't vision track.
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretIdle);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsAtSetpoint())
            {
                // Start preloading into the shooter.
                m_pHopper->Preload(true);
            }
            break;

        case eTeleopCloseRangeFiring :
            /********************************************************************
                Close RangeFiring - Robot simply fires wherever it is currently aiming.
            ********************************************************************/
            // Disable robot control.
            m_pRobotDrive->SetJoystickControl(false);
            // Lift the front of the robot.
            m_pLift->ExtendLift(true);
            // Don't vision track.
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretIdle);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterCloseRangeVelocity);
            if (m_pShooter->IsAtSetpoint())
            {
                // Start preloading into the shooter.
                m_pHopper->Preload(true);
            }
            break;

        case eTeleopAutoFiring:
            /********************************************************************
                AutoFiring - Robot is firing the Energy into the high goal
                             while tracking the goal actively.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(true);
            // Set the Turret to Tracking mode.
            m_pTurret->SetState(eTurretTracking);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsAtSetpoint()) 
            {
                // Start preloading into the shooter.
                m_pHopper->Preload();
            }
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
	m_pShooter->Tick();
	m_pHood->Tick();
	m_pTurret->Tick();
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

	// Turn off vision LED.
	m_pShooter->SetVisionLED(true);
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
