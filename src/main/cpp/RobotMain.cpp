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
    m_pAutonomousChooser->AddOption("Shooting", "Shooting");
    m_pAutonomousChooser->AddOption("Barrel Path", "Barrel Path");
	m_pAutonomousChooser->AddOption("Slalom Path", "Slalom Path");
	m_pAutonomousChooser->AddOption("Bounce Path", "Bounce Path");
	m_pAutonomousChooser->AddOption("Basic Path", "Basic Path");
    m_pAutonomousChooser->AddOption("Galactic Search", "Galactic Search");
    m_pAutonomousChooser->AddOption("Test Path", "Test Path");
	m_pAutonomousChooser->AddOption("Just Sing a Song", "Just Sing a Song");
	SmartDashboard::PutData(m_pAutonomousChooser);
    SmartDashboard::PutNumber("Y Setpoint Offset", 0.0);

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

    // Record Start Time.
    m_dStartTime = m_pTimer->Get();

	// Get the selected auto mode from SmartDashboard.
	int m_nSelectedTrajectory = -1;
	string m_strAutonomousSelected = m_pAutonomousChooser->GetSelected();
	if (m_strAutonomousSelected == "Autonomous Idle")
	{
		m_nAutoState = eAutoStopped;
	}
    if (m_strAutonomousSelected == "Shooting")
	{
		m_nAutoState = eAutoShoot;
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
    if (m_strAutonomousSelected == "Galactic Search")
    {
        m_nAutoState = eAutoGalacticSearch1;
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
    // Get and store vision data.
    double dBallXLocation = SmartDashboard::GetNumber("Target Center X", 0.0);
    double dBallRadius = SmartDashboard::GetNumber("Target Radius", 0.0);
    // Create instance variables.
    double dRed1BallExpectedLocation    = 140.0;          //  
    double dRed1BallExpectedRadius      = 52.0;          //  
    double dRed2BallExpectedLocation    = -287.0;          //  
    double dRed2BallExpectedRadius      = 33.0;          //  ----| These are the values for the closest ball on each path. 
    double dBlue1BallExpectedLocation   = -59.0;          //  
    double dBlue1BallExpectedRadius     = 14.0;          //  
    double dBlue2BallExpectedLocation   = 217.0;          //  
    double dBlue2BallExpectedRadius     = 18.0;          //
    double dBallLocationTolerance       = 20.0;  
    double dBallRadiusTolerance         = 20.0; 
    bool bGalacticSearchPathFound       = false;

	// Autonomous state machine.
	switch (m_nAutoState)
	{
		case eAutoStopped :
			// Do stopping things here.
			m_pRobotDrive->Stop();
            // Disable LEDs
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(false);
            // Return intake to it's retracted state.
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Idle the Hood, Turret, and Hopper.
            m_pHood->SetState(eHoodReset);
            // m_pTurret->Stop();
            m_pHopper->Preload(false);

			// Move to auto idle state.
			m_nAutoState = eAutoIdle;

		case eAutoIdle :
			// Do nothing.
			break;

        case eAutoShoot :
            // Fire the 3 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 1.00)
            {
                // Lower intake.
                m_pIntake->Extend(true);
            }
            else
            {
                m_nAutoState = eAutoShoot2;
            }
            break;

        case eAutoShoot2 :
            if (fabs(m_pTimer->Get() - m_dStartTime) < 6.00)
            {
                // Set turret to tacking mode.
                m_pTurret->SetVision(true);
                // Enable LEDs.
                m_pShooter->SetVisionLED(true);
                // Aim with vision.
                m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0) + SmartDashboard::GetNumber("Y Setpoint Offset", 0));
                // Set firing velocity.
                m_pShooter->SetSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preload.
                    m_pHopper->Preload(true);
                    m_pIntake->IntakeMotor(true);
                }
            }
            else
            {
                // Set trajectory to follow.
                m_pRobotDrive->SetSelectedTrajectory(eAutoPathReverse);
                // Move to next state.
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoShoot3 :
            // Retract intake.
            m_pIntake->Extend(false);
            // Stop intake.
            m_pIntake->IntakeMotor(false);
            // Set vision false.
            m_pShooter->SetVisionLED(false);
            // Stop turret.
            m_pTurret->SetVision(false);
            // Set idle.
            m_pShooter->SetSetpoint(eShooterIdle);
            // Follow path.
            m_pRobotDrive->FollowTrajectory(m_pTimer->Get() - m_dStartTime);
            // Check time.
            if ((m_pTimer->Get() - m_dStartTime) > m_pRobotDrive->GetTrajectoryTotalTime())
            {
                // Stop robot drive.
                m_pRobotDrive->Stop();
                // Move to next state.
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoShoot4 :
            // Move to idle.
            m_nAutoState = eTeleopIdle;
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

        case eAutoGalacticSearch1 :
            // Set vision to power cell tracking mode.
            SmartDashboard::PutBoolean("Camera Source", true);

            // Wait 1 sec for vision to adjust.
            if ((m_pTimer->Get() - m_dStartTime) > 0.2)
            {
                // Select red/blue a/b path based on power cell location.
                // Red 1 path.
                if ((fabs(dBallXLocation - dRed1BallExpectedLocation) < dBallLocationTolerance) && (fabs(dBallRadius - dRed1BallExpectedRadius) < dBallRadiusTolerance))
                {
                    // Select path.
                    m_pRobotDrive->SetSelectedTrajectory(eGalacticSearchRed1);
                    // Put selected state on dashboard for debugging.
                    SmartDashboard::PutString("Galactic Search Path", "Red 1");
                    // Set that a path was selected.
                    bGalacticSearchPathFound = true;
                }
                // Red 2 path.
                if ((fabs(dBallXLocation - dRed2BallExpectedLocation) < dBallLocationTolerance) && (fabs(dBallRadius - dRed2BallExpectedRadius) < dBallRadiusTolerance))
                {
                    // Select path.
                    m_pRobotDrive->SetSelectedTrajectory(eGalacticSearchRed2);
                    // Put selected state on dashboard for debugging.
                    SmartDashboard::PutString("Galactic Search Path", "Red 2");
                    // Set that a path was selected.
                    bGalacticSearchPathFound = true;
                }
                // Blue 1 path.
                if ((fabs(dBallXLocation - dBlue1BallExpectedLocation) < dBallLocationTolerance) && (fabs(dBallRadius - dBlue1BallExpectedRadius) < dBallRadiusTolerance))
                {
                    // Select path.
                    m_pRobotDrive->SetSelectedTrajectory(eGalacticSearchBlue1);
                    // Put selected state on dashboard for debugging.
                    SmartDashboard::PutString("Galactic Search Path", "Blue 1");
                    // Set that a path was selected.
                    bGalacticSearchPathFound = true;
                }
                // Blue 2 path.
                if ((fabs(dBallXLocation - dBlue2BallExpectedLocation) < dBallLocationTolerance) && (fabs(dBallRadius - dBlue2BallExpectedRadius) < dBallRadiusTolerance))
                {
                    // Select path.
                    m_pRobotDrive->SetSelectedTrajectory(eGalacticSearchBlue2);
                    // Put selected state on dashboard for debugging.
                    SmartDashboard::PutString("Galactic Search Path", "Blue 2");
                    // Set that a path was selected.
                    bGalacticSearchPathFound = true;
                }
            }

            // Move to the next state when a path is selected.
            if (bGalacticSearchPathFound)
            {
                // Move auto states.
                m_nAutoState = eAutoGalacticSearch2;
                // Record new start time.
                m_dStartTime = m_pTimer->Get();
            }
            break;

        case eAutoGalacticSearch2 :
            // Follow the path while intaking.
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake motor.
            m_pIntake->IntakeMotor(true);
            // Set vision false.
            m_pShooter->SetVisionLED(false);
            // Stop turret.
            m_pTurret->SetVision(false);
            // Follow path.
            m_pRobotDrive->FollowTrajectory(dElapsedTime);
            // Check time.
            if (dElapsedTime > m_pRobotDrive->GetTrajectoryTotalTime())
            {
                // Move to next state.
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
			// Sing a song!
            if (!m_pRobotDrive->GetOrchestra()->IsPlaying())
            {
                m_pRobotDrive->GetOrchestra()->LoadMusic(kSong);
                m_pRobotDrive->GetOrchestra()->Play();
            }
			break;
	}

	// Call drive tick.
    if (m_nAutoState != eSingASong)
    {
        m_pRobotDrive->Tick();
	    m_pShooter->Tick();
	    m_pHood->Tick();
	    m_pTurret->Tick();   
    }
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

    // Extend intake.
    // m_pIntake->Extend(true);
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
    static double dCloseRangeStartTime = 0.0;
	static bool bHasFired           = false;
    static bool bTurretMoving       = false;
    static bool bHoodMoving         = false;

    /********************************************************************
        Drive Controller - Toggle Intake (Button B)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonB))
    {
        if (!m_pIntake->GetExtended())
        {
            // Extend Intake.
            m_pIntake->Extend(true);
            m_nTeleopState = eTeleopIntake;
        }
        else
        {
            // Retract Intake.
            m_pIntake->Extend(false);
            m_nTeleopState = eTeleopStopped;
        }
    }

    /********************************************************************
        Drive Controller - Start Intake Rollers (Right Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonRB))
    {
        // Start intake.
        if (m_nTeleopState == eTeleopIntake2)
        {
            m_nTeleopState = eTeleopStopped;
        }
        else
        {
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
        Drive Controller - Close Range Fire (Left Bumper)
    ********************************************************************/
   static bool bTimerSet = false;
    if (m_pDriveController->GetRawButton(eButtonLB))
    {
        // Record start time.
        if (!bTimerSet)
        {
            dCloseRangeStartTime = m_pTimer->Get();
        }
        // Set state to Firing.
        m_nTeleopState = eTeleopCloseRangeFiring;
        bHasFired = true;
        bTimerSet = true;

    }
    else
    {
        if (bHasFired)
        {
            // Has been fired, return to idle.
            m_pShooter->SetState(eShooterIdle);
            m_nTeleopState = eTeleopStopped;
            bHasFired = false;
            bTimerSet = true;
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
        Aux Controller - Manual Move Hood Up (Up POV)
    ********************************************************************/
    if (m_pAuxController->GetPOV() == 0)
    {
        // Manual move up.
        m_pHood->SetState(eHoodManualFwd);
        bHoodMoving = true;
    }
    else
    {
    /********************************************************************
        Aux Controller - Manual Move Hood Down (Down POV)
    ********************************************************************/
        if (m_pAuxController->GetPOV() == 180)
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
        Aux Controller - Zero Hood Encoder (Button X)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonX))
    {
        m_pHood->Rezero();
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
        Aux Controller - Intake Unjam (Left Trigger)
    ********************************************************************/
    static bool bToggle = false;
    if (m_pAuxController->GetRawAxis(eLeftTrigger) > 0.55)
    {
        // Set manual intake speed.
        m_pIntake->SetSpeed(-(m_pAuxController->GetRawAxis(eLeftTrigger) - 0.5));
        bToggle = true;
    }
    else
    {
        if (bToggle)
        {
            m_pIntake->SetSpeed(0.0);
            bToggle = false;
        }
    }
    

    /********************************************************************
        Aux Controller - Toggle Shooter "Idle" speed (Button B)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonB))
    {
        m_pShooter->SetState(m_pShooter->GetState() == eShooterIdle ? eShooterStopped : eShooterIdle);
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
            // m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            // Enable joystick control.
            m_pRobotDrive->SetJoystickControl(true);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Idle the Hood, Turret, and Hopper.
            m_pHood->SetState(eHoodReset);
            // m_pTurret->Stop();
            m_pHopper->Preload(false);
            m_pHood->SetHoodSafety(true);

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
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(false);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendLift(false);
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake.
            m_pIntake->IntakeMotor(true);
            // Stop Turret, and stop Hood.
            m_pTurret->Stop();
            m_pHopper->Preload(false);
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake on a half second delay.
            if ((m_pTimer->Get() - m_dStartTime) >= 0.2)
            {
                m_pIntake->IntakeMotor(true);
            }
            // Move to next state.
            m_nTeleopState = eTeleopIntake2;
            break;

        case eTeleopIntake2 :
            // Start intake on a half second delay.
            if ((m_pTimer->Get() - m_dStartTime) >= 0.5)
            {
                m_pIntake->IntakeMotor(true);
            }
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
            m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0) + SmartDashboard::GetNumber("Y Setpoint Offset", 0));
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
            // *Do* vision track.
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(true);
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretTracking);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterCloseRangeVelocity);
            // Set the hood to zero.
            m_pHood->SetSetpoint(1.0);
            // Turn off hood safety.
            m_pHood->SetHoodSafety(false);
            if (m_pShooter->IsAtSetpoint() && ((m_pTimer->Get() - dCloseRangeStartTime) >= 3.5))
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
                // Intake 
                m_pIntake->IntakeMotor(true);
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

    // Retract Intake.
    m_pIntake->Extend(false);

    // Disable music if playing.
    if (m_pRobotDrive->GetOrchestra()->IsPlaying())
    {
        m_pRobotDrive->GetOrchestra()->Stop();
    }
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
