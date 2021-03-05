/****************************************************************************
	Description:	Implements the CDrive class.

	Classes:		CDrive

	Project:		Swerve Drive
****************************************************************************/
#include "Drive.h"
#include <frc/smartdashboard/SmartDashboard.h>
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**
 *  @brief	        CDrive Constructor.
 *
 *	@param          pJoystick pointer to the driver's joystick.
 *
 *  @implements     Nothing
 ****************************************************************************/
CDrive::CDrive(frc::Joystick* pDriveController)
{
    // Initialize member variables.
    m_bJoystickControl          = true;

    // Store the joystick pointer.
    m_pDriveController          = pDriveController;
    // Create objects to be passed into the modules.
    m_pDriveMotorFrontLeft      = new motorcontrol::can::TalonFX(nDriveMotorLeftFront);
    m_pDriveMotorFrontRight     = new motorcontrol::can::TalonFX(nDriveMotorRightFront);
    m_pDriveMotorBackLeft       = new motorcontrol::can::TalonFX(nDriveMotorLeftBack);
    m_pDriveMotorBackRight      = new motorcontrol::can::TalonFX(nDriveMotorRightBack);
    m_pAzimuthMotorFrontLeft    = new motorcontrol::can::TalonFX(nAzimuthMotorLeftFront);
    m_pAzimuthMotorFrontRight   = new motorcontrol::can::TalonFX(nAzimuthMotorRightFront);
    m_pAzimuthMotorBackLeft     = new motorcontrol::can::TalonFX(nAzimuthMotorLeftBack);
    m_pAzimuthMotorBackRight    = new motorcontrol::can::TalonFX(nAzimuthMotorRightBack);
    m_pEncoderFrontLeft         = new sensors::CANCoder(nEncoderFrontLeft);
    m_pEncoderFrontRight        = new sensors::CANCoder(nEncoderFrontRight);
    m_pEncoderBackLeft          = new sensors::CANCoder(nEncoderBackLeft);
    m_pEncoderBackRight         = new sensors::CANCoder(nEncoderBackRight);
    // Create our 4 swerve modules.
    m_pModFrontLeft             = new CSwerveModule(m_pDriveMotorFrontLeft, m_pAzimuthMotorFrontLeft, m_pEncoderFrontLeft, 67);
    m_pModFrontRight            = new CSwerveModule(m_pDriveMotorFrontRight, m_pAzimuthMotorFrontRight, m_pEncoderFrontRight, -150);
    m_pModBackLeft              = new CSwerveModule(m_pDriveMotorBackLeft, m_pAzimuthMotorBackLeft, m_pEncoderBackLeft, -113);
    m_pModBackRight             = new CSwerveModule(m_pDriveMotorBackRight, m_pAzimuthMotorBackRight, m_pEncoderBackRight, -160);
    // Create the NavX Gyro.
    m_pGyro                     = new AHRS(SerialPort::Port::kUSB);
    // Create Odometry. Start at -1 to prevent an error.
    TrajectoryConstants.SelectTrajectory(-1);
    m_pSwerveDriveOdometry      = new SwerveDriveOdometry<4>(m_kKinematics, Rotation2d(degree_t(GetYaw())), TrajectoryConstants.GetSelectedTrajectoryStartPoint());
     // Setup motion profile controller with PID Controllers.
    auto m_pAnglePID = ProfiledPIDController<radian>{m_dPIDThetakP, m_dPIDThetakI, m_dPIDThetakD, TrapezoidProfile<radian>::Constraints{TrajectoryConstants.kMaxRotationSpeed, TrajectoryConstants.kMaxRotationAcceleration / 1_s}};
    m_pAnglePID.EnableContinuousInput((degree_t)-180, (degree_t)180);
    m_pHolonomicDriveController = new HolonomicDriveController( 
        frc2::PIDController{m_dPIDXkP, m_dPIDXkI, m_dPIDXkD}, 
        frc2::PIDController{m_dPIDYkP, m_dPIDYkI, m_dPIDYkD},
        m_pAnglePID
    );
}

/************************************************************************//**
 *  @brief	        CDrive Destructor.
 *
 *	@param          None
 *
 *  @implements     Nothing
 ****************************************************************************/
CDrive::~CDrive()
{
    delete m_pDriveController;
    delete m_pDriveMotorFrontLeft;       
    delete m_pDriveMotorFrontRight;     
    delete m_pDriveMotorBackLeft;       
    delete m_pDriveMotorBackRight;      
    delete m_pAzimuthMotorFrontLeft;    
    delete m_pAzimuthMotorFrontRight;   
    delete m_pAzimuthMotorBackLeft;     
    delete m_pAzimuthMotorBackRight;    
    delete m_pEncoderFrontLeft;             
    delete m_pEncoderFrontRight;            
    delete m_pEncoderBackLeft;              
    delete m_pEncoderBackRight;
    delete m_pModFrontLeft;
    delete m_pModFrontRight;
    delete m_pModBackLeft;
    delete m_pModBackRight;  
    delete m_pGyro;
    delete m_pSwerveDriveOdometry;
    delete m_pHolonomicDriveController;

    m_pDriveController          = nullptr;
    m_pDriveMotorFrontLeft      = nullptr;
    m_pDriveMotorFrontRight     = nullptr;
    m_pDriveMotorBackLeft       = nullptr;
    m_pDriveMotorBackRight      = nullptr;
    m_pAzimuthMotorFrontLeft    = nullptr;
    m_pAzimuthMotorFrontRight   = nullptr;
    m_pAzimuthMotorBackLeft     = nullptr;
    m_pAzimuthMotorBackRight    = nullptr;
    m_pEncoderFrontLeft             = nullptr;
    m_pEncoderFrontRight            = nullptr;
    m_pEncoderBackLeft              = nullptr;
    m_pEncoderBackRight             = nullptr;
    m_pModFrontLeft             = nullptr;
    m_pModFrontRight            = nullptr;
    m_pModBackLeft              = nullptr;
    m_pModBackRight             = nullptr;
    m_pGyro                     = nullptr;
    m_pSwerveDriveOdometry      = nullptr;
    m_pHolonomicDriveController = nullptr;
}

/************************************************************************//**
 *  @brief	        Drive Initialization routine.
 *
 *	@param          None
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::Init()
{
    // Clear faults for motor controllers.
    m_pDriveMotorFrontLeft->ClearStickyFaults(); 
    m_pDriveMotorFrontRight->ClearStickyFaults();
    m_pDriveMotorBackLeft->ClearStickyFaults(); 
    m_pDriveMotorBackRight->ClearStickyFaults();   
    m_pAzimuthMotorFrontLeft->ClearStickyFaults(); 
    m_pAzimuthMotorFrontRight->ClearStickyFaults();
    m_pAzimuthMotorBackLeft->ClearStickyFaults();  
    m_pAzimuthMotorBackRight->ClearStickyFaults(); 

    // Initialize swerve modules.
    m_pModFrontLeft->Init();
    m_pModFrontRight->Init();
    m_pModBackLeft->Init();
    m_pModBackRight->Init();

    // Reverse Azimuth motor.
    m_pAzimuthMotorFrontLeft->SetInverted(false);
    m_pAzimuthMotorFrontRight->SetInverted(false);
    m_pAzimuthMotorBackLeft->SetInverted(false);
    m_pAzimuthMotorBackRight->SetInverted(false);

    m_pDriveMotorFrontLeft->SetInverted(false);
    m_pDriveMotorFrontRight->SetInverted(false);
    m_pDriveMotorBackLeft->SetInverted(false);
    m_pDriveMotorBackRight->SetInverted(false);

    // Zero the NavX.
    m_pGyro->ZeroYaw();
}

/************************************************************************//**
 *  @brief	        Drive looped implementation routine.
 *
 *	@param          None
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::Tick()
{
    if (m_bJoystickControl)
    {   
        // Check joystick for deadzones. If it is below the deadzone threshold, set to 0.
        m_dYAxis = m_pDriveController->GetRawAxis(1) * ((m_pDriveController->GetRawAxis(eRightTrigger) + 0.5) / 1.5);
        if (fabs(m_dYAxis) < m_dJoystickDeadzone)
        {
            m_dYAxis = 0.0;
        }
        else
        {
            // If the joystick is not in the deadzone, multiply it and square it.
            m_dYAxis = copysign(pow(m_dYAxis * m_dTeleopMultiplier, 2), m_dYAxis);
        }
        
        // Check joystick for deadzones. If it is below the deadzone threshold, set to 0.
        m_dXAxis = m_pDriveController->GetRawAxis(0) * ((m_pDriveController->GetRawAxis(eRightTrigger) + 0.5) / 1.5);
        if (fabs(m_dXAxis) < m_dJoystickDeadzone)
        {
            m_dXAxis = 0.0;
        }
        else
        {
            // If the joystick is not in the deadzone, multiply it and square it.
            m_dXAxis = copysign(pow(m_dXAxis * m_dTeleopMultiplier, 2), m_dXAxis);
        }
        
        // Check joystick for deadzones. If it is below the deadzone threshold, set to 0.
        m_dRotateCW = m_pDriveController->GetRawAxis(4);
        if (fabs(m_dRotateCW) < m_dJoystickDeadzone)
        {
            m_dRotateCW = 0.0;
        }
        else
        {
            // If the joystick is not in the deadzone, multiply it and square it.
            m_dRotateCW = copysign(pow(m_dRotateCW * m_dTeleopMultiplier, 2), m_dRotateCW);
        }

        // Calculate and set module states.
        // This is where we invert all joysticks.
        auto States = m_kKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(meters_per_second_t(-m_dYAxis), meters_per_second_t(-m_dXAxis), radians_per_second_t(-m_dRotateCW), degree_t(GetYaw())));
        // Normalize the wheelspeeds, making sure they don't go past 100% output.
        m_kKinematics.NormalizeWheelSpeeds(&States, TrajectoryConstants.kMaxTranslationSpeed);
        // Send the states to each module.
        SetModuleStates(States);
    }
    
    // Update drive odometry.
    m_pSwerveDriveOdometry->Update(Rotation2d(degree_t(GetYaw())), m_pModFrontLeft->GetModuleState(), m_pModFrontRight->GetModuleState(), m_pModBackLeft->GetModuleState(), m_pModBackRight->GetModuleState());

    // Put values on SmartDashboard.
    SmartDashboard::PutNumber("Odometry X Pos", double(GetRobotPose().X()));
    SmartDashboard::PutNumber("Odometry Y Pos", double(GetRobotPose().Y()));
    SmartDashboard::PutNumber("Odometry Z Pos", double(GetRobotPose().Rotation().Degrees()));

    SmartDashboard::PutNumber("FrontLeft Speed", m_pModFrontLeft->GetSpeed());
    SmartDashboard::PutNumber("FrontRight Speed", m_pModFrontRight->GetSpeed());
    SmartDashboard::PutNumber("BackLeft Speed", m_pModBackLeft->GetSpeed());
    SmartDashboard::PutNumber("BackRight Speed", m_pModBackRight->GetSpeed());
    SmartDashboard::PutNumber("FrontLeft SpeedSP", m_pModFrontLeft->GetSpeedSetpoint() / 2048 * 0.391159);
    SmartDashboard::PutNumber("FrontRight SpeedSP", m_pModFrontRight->GetSpeedSetpoint() / 2048 * 0.391159);
    SmartDashboard::PutNumber("BackLeft SpeedSP", m_pModBackLeft->GetSpeedSetpoint() / 2048 * 0.391159);
    SmartDashboard::PutNumber("BackRight SpeedSP", m_pModBackRight->GetSpeedSetpoint() / 2048 * 0.391159
    );

    SmartDashboard::PutNumber("FrontLeft Angle", m_pModFrontLeft->GetAngle());
    SmartDashboard::PutNumber("FrontRight Angle", m_pModFrontRight->GetAngle());
    SmartDashboard::PutNumber("BackLeft Angle", m_pModBackLeft->GetAngle());
    SmartDashboard::PutNumber("BackRight Angle", m_pModBackRight->GetAngle());
    SmartDashboard::PutNumber("FrontLeft AngleSP", m_pModFrontLeft->GetAngleSetpoint());
    SmartDashboard::PutNumber("FrontRight AngleSP", m_pModFrontRight->GetAngleSetpoint());
    SmartDashboard::PutNumber("BackLeft AngleSP", m_pModBackLeft->GetAngleSetpoint());
    SmartDashboard::PutNumber("BackRight AngleSP", m_pModBackRight->GetAngleSetpoint());

    // Call swerve module ticks.
    m_pModFrontLeft->Tick();
    m_pModFrontRight->Tick();
    m_pModBackLeft->Tick();
    m_pModBackRight->Tick();
}

/************************************************************************//**
 *  @brief	        Set the drive motors' speed to zero.
 *
 *	@param          None
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::Stop()
{
    m_pDriveMotorFrontLeft->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    m_pDriveMotorFrontRight->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    m_pDriveMotorBackLeft->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    m_pDriveMotorBackRight->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
}

/************************************************************************//**
 *  @brief	        Enables or disables joystick control. 
 *
 *	@param          bool bJoystickControl 
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::SetJoystickControl(bool bJoystickControl)
{
    m_bJoystickControl = bJoystickControl;
}

/************************************************************************//**
 *  @brief	        Sets the module states across all 4 swerve modules.
 *
 *	@param          SwerveModuleState[] States desired. 
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    m_pModFrontLeft->SetModuleState(desiredStates[0]);
    m_pModFrontRight->SetModuleState(desiredStates[1]);
    m_pModBackLeft->SetModuleState(desiredStates[2]);
    m_pModBackRight->SetModuleState(desiredStates[3]);
}

/************************************************************************//**
 *  @brief	        Gets the current robot pose from odometry. 
 *
 *	@param          None
 *
 *  @retval         Pose2d The current robot pose.
 ****************************************************************************/
Pose2d CDrive::GetRobotPose()
{
    return m_pSwerveDriveOdometry->GetPose();
}

/************************************************************************//**
 *  @brief	        Resets the robot position to the current selected trajectory 
 *                  start point. 
 *
 *	@param          None
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::ResetOdometry()
{
    // Reset position.
    m_pSwerveDriveOdometry->ResetPosition(TrajectoryConstants.GetSelectedTrajectoryStartPoint(), Rotation2d(degree_t(GetYaw())));
}

/************************************************************************//**
 *  @brief	        Get the total trajectory time.
 *
 *	@param          None
 *
 *  @retval         double dTotalTime
 ****************************************************************************/
double CDrive::GetTrajectoryTotalTime()
{
    return TrajectoryConstants.GetSelectedTrajectoryTotalTime();
}

/************************************************************************//**
 *  @brief	        Select a predetermined path from a list of auto states. 
 *
 *	@param          nAutoState The current auto path.
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::SetSelectedTrajectory(int nAutoState)
{
    TrajectoryConstants.SelectTrajectory(nAutoState);
}

/************************************************************************//**
 *  @brief	        Build a list of waypoints from the current position of
 *                  the robot and list of preset points.
 *
 *	@param          None
 *
 *  @retval         Trajectory The compiled trajectory.
 ****************************************************************************/
void CDrive::GenerateTrajectoryFromCurrentPosition()
{
    // Create the trajectory config.
    auto Config = TrajectoryConfig(TrajectoryConstants.kMaxTranslationSpeed, TrajectoryConstants.kMaxTranslationAcceleration);
    // Recreate swerve kinematics as an object.
    SwerveDriveKinematics<4> Kinematics = SwerveDriveKinematics<4>(FrontLeft, FrontRight, BackLeft, BackRight);
    // Add kinematics to ensure max speed for each module is actually obeyed.
    Config.SetKinematics(Kinematics);

    // Build the trajectory waypoints.
    vector<Pose2d> vWaypoints;
    // Place our current robot pose in a vector.
    vWaypoints.emplace_back(GetRobotPose());
    // Add preset path points.
    for (Pose2d Point : TrajectoryConstants.PresetWaypoints)
    {
        vWaypoints.emplace_back(Point);
    }

    // Generate the trajectory and store it in CTrajectoryConstants.
    try
    {
        TrajectoryConstants.SelectTrajectory(TrajectoryGenerator::GenerateTrajectory(vWaypoints, Config));
    }
    catch(const std::exception& e)
    {
        // Print error to console.
        std::cerr << e.what() << '\n';
    }
}

/************************************************************************//**
 *  @brief	        Follow the given path.
 *
 *	@param          Trajectory Path for robot to follow.
 *
 *  @retval         Nothing
 ****************************************************************************/
void CDrive::FollowTrajectory(double dElapsedTime)
{
    // Sample the trajectory at the given time.
    auto Goal = TrajectoryConstants.GetSelectedTrajectory().Sample(second_t(dElapsedTime));
    double dDesiredTheta = (Goal.curvature() * (180 / 3.1415926));
    SmartDashboard::PutNumber("Desired Theta", double(dDesiredTheta));

    // Calculate the module states based on current position.
    auto AdjustedSpeeds = m_pHolonomicDriveController->Calculate(GetRobotPose(), Goal, degree_t(dDesiredTheta));

    SmartDashboard::PutNumber("Wheel SPEEDX", double(AdjustedSpeeds.vx));
    SmartDashboard::PutNumber("Wheel SPEEDY", double(AdjustedSpeeds.vy));

    // Convert adjusted speeds to module states.
    auto States = m_kKinematics.ToSwerveModuleStates(AdjustedSpeeds);

    // Set modules states.
    SetModuleStates(States);
}

/************************************************************************//**
 *  @brief	        Return the rotational angle of the robot.
 *
 *	@param          None
 *
 *  @retval         double Angle in degrees
 ****************************************************************************/
double CDrive::GetYaw()
{
    // This inverts the Gyro, which is required by the module's implementation.
    return -m_pGyro->GetYaw();
}
/////////////////////////////////////////////////////////////////////////////