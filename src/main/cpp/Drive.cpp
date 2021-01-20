/****************************************************************************
	Description:	Implements the CDrive class.

	Classes:		CDrive

	Project:		Swerve Drive
****************************************************************************/
#include "Drive.h"
#include "IOMap.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace units;
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
    // Store the joystick pointer.
    m_pDriveController          = pDriveController;
    // Create objects to be passed into the modules.
    m_pDriveMotorFrontLeft      = new rev::CANSparkMax(nDriveMotorLeftFront, rev::CANSparkMax::MotorType::kBrushless);
    m_pDriveMotorFrontRight     = new rev::CANSparkMax(nDriveMotorRightFront, rev::CANSparkMax::MotorType::kBrushless);
    m_pDriveMotorBackLeft       = new rev::CANSparkMax(nDriveMotorLeftBack, rev::CANSparkMax::MotorType::kBrushless);
    m_pDriveMotorBackRight      = new rev::CANSparkMax(nDriveMotorRightBack, rev::CANSparkMax::MotorType::kBrushless);
    m_pAzimuthMotorFrontLeft    = new rev::CANSparkMax(nAzimuthMotorLeftFront, rev::CANSparkMax::MotorType::kBrushless);
    m_pAzimuthMotorFrontRight   = new rev::CANSparkMax(nAzimuthMotorRightFront, rev::CANSparkMax::MotorType::kBrushless);
    m_pAzimuthMotorBackLeft     = new rev::CANSparkMax(nAzimuthMotorLeftBack, rev::CANSparkMax::MotorType::kBrushless);
    m_pAzimuthMotorBackRight    = new rev::CANSparkMax(nAzimuthMotorRightBack, rev::CANSparkMax::MotorType::kBrushless);
    m_pPotFrontLeft             = new frc::AnalogPotentiometer(nPotFrontLeft, 360);
    m_pPotFrontRight            = new frc::AnalogPotentiometer(nPotFrontRight, 360);
    m_pPotBackLeft              = new frc::AnalogPotentiometer(nPotBackLeft, 360);
    m_pPotBackRight             = new frc::AnalogPotentiometer(nPotBackRight, 360);
    // Create our 4 swerve modules.
    m_pModFrontLeft             = new CSwerveModule(m_pDriveMotorFrontLeft, m_pAzimuthMotorFrontLeft, m_pPotFrontLeft, 1.0); // 8
    m_pModFrontRight            = new CSwerveModule(m_pDriveMotorFrontRight, m_pAzimuthMotorFrontRight, m_pPotFrontRight, -29.0); // -20
    m_pModBackLeft              = new CSwerveModule(m_pDriveMotorBackLeft, m_pAzimuthMotorBackLeft, m_pPotBackLeft, 48.0); // 45
    m_pModBackRight             = new CSwerveModule(m_pDriveMotorBackRight, m_pAzimuthMotorBackRight, m_pPotBackRight, 49.0); // 50
    // Create the drive controller and PID controllers.
    m_pHoloDrive = new frc::HolonomicDriveController(
        frc2::PIDController{0.0, 0.0, 0.0},
        frc2::PIDController{0.0, 0.0, 0.0},
        frc::ProfiledPIDController<radian>{0.0, 0.0, 0.0, frc::TrapezoidProfile<radian>::Constraints{0.0_rad_per_s, 0.0_rad_per_s / 1_s}
    });
    // m_pPIDx                     = new frc2::PIDController(0, 0, 0);
    // m_pPIDy                     = new frc2::PIDController(0, 0, 0);
    // m_pPIDtheta                 = new frc::ProfiledPIDController<radian>(0.0, 0.0, 0.0, frc::TrapezoidProfile<radian>::Constraints{0.0_rad_per_s, 0.0_rad_per_s / 1_s}
    // In theory, if we used SwerveControllerCommand, this shouldn't be necessary,
    // but the arguments don't work for some reason? No clue why.
    //m_pHoloDrive                = new frc::HolonomicDriveController(*m_pPIDx, *m_pPIDy, *m_pPIDtheta);

    // Flip all the azimuth motors.
    m_pAzimuthMotorFrontLeft->SetInverted(true);
    m_pAzimuthMotorFrontRight->SetInverted(true);   
    m_pAzimuthMotorBackLeft->SetInverted(true);
    m_pAzimuthMotorBackRight->SetInverted(true);
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
    delete m_pPotFrontLeft;             
    delete m_pPotFrontRight;            
    delete m_pPotBackLeft;              
    delete m_pPotBackRight;
    delete m_pModFrontLeft;
    delete m_pModFrontRight;
    delete m_pModBackLeft;
    delete m_pModBackRight;  

    m_pDriveController          = nullptr;
    m_pDriveMotorFrontLeft      = nullptr;
    m_pDriveMotorFrontRight     = nullptr;
    m_pDriveMotorBackLeft       = nullptr;
    m_pDriveMotorBackRight      = nullptr;
    m_pAzimuthMotorFrontLeft    = nullptr;
    m_pAzimuthMotorFrontRight   = nullptr;
    m_pAzimuthMotorBackLeft     = nullptr;
    m_pAzimuthMotorBackRight    = nullptr;
    m_pPotFrontLeft             = nullptr;
    m_pPotFrontRight            = nullptr;
    m_pPotBackLeft              = nullptr;
    m_pPotBackRight             = nullptr;
    m_pModFrontLeft             = nullptr;
    m_pModFrontRight            = nullptr;
    m_pModBackLeft              = nullptr;
    m_pModBackRight             = nullptr;
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
    m_pDriveMotorFrontLeft->ClearFaults(); 
    m_pDriveMotorFrontRight->ClearFaults();
    m_pDriveMotorBackLeft->ClearFaults(); 
    m_pDriveMotorBackRight->ClearFaults();   
    m_pAzimuthMotorFrontLeft->ClearFaults(); 
    m_pAzimuthMotorFrontRight->ClearFaults();
    m_pAzimuthMotorBackLeft->ClearFaults();  
    m_pAzimuthMotorBackRight->ClearFaults(); 

    // Initialize swerve modules.
    m_pModFrontLeft->Init();
    m_pModFrontRight->Init();
    m_pModBackLeft->Init();
    m_pModBackRight->Init();

    // Zero the NavX.
    m_Gyro.ZeroYaw();
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
    // Set up some variables.
    double dHypot = std::hypot(dWidth, dLength);
    double dForward, dSteering, dRotateCW, dTheta, dTemp;

    // Check 3 joysticks for deadzones. If they are below the deadzone threshold, set to 0.
    dForward = -m_pDriveController->GetRawAxis(1);
    if (fabs(dForward) < m_dJoystickDeadzone)
    {
        dForward = 0.0;
    }

    dSteering = m_pDriveController->GetRawAxis(0);
    if (fabs(dSteering) < m_dJoystickDeadzone)
    {
        dSteering = 0.0;
    }

    dRotateCW = m_pDriveController->GetRawAxis(4);
    if (fabs(dRotateCW) < m_dJoystickDeadzone)
    {
        dRotateCW = 0.0;
    }

    // Get Gyro angle from the NavX.
    dTheta = (m_Gyro.GetAngle()) * (3.1415 / 180);
    
    // Calculate module wheel angles from the gyro.
    dTemp = dForward * std::cos(dTheta) + dSteering * std::sin(dTheta);
    dSteering = -dForward * std::sin(dTheta) + dSteering * std::cos(dTheta);
    dForward = dTemp;

    // Do math. I honestly have no clue what these variable names are.
    double A = dSteering - dRotateCW * (dLength/dHypot);
    double B = dSteering + dRotateCW * (dLength/dHypot);
    double C = dForward - dRotateCW * (dWidth/dHypot);
    double D = dForward + dRotateCW * (dWidth/dHypot);

    // Do wheelspeed math with the above math. Some trig stuff or whatever.
    double dWheelSpeedFL = std::hypot(B, D);
    double dWheelAngleFL = std::atan2(B, D) * 180 / 3.1415;

    double dWheelSpeedFR = std::hypot(B, C);
    double dWheelAngleFR = std::atan2(B, C) * 180 / 3.1415;

    double dWheelSpeedBL = std::hypot(A, D);
    double dWheelAngleBL = std::atan2(A, D) * 180 / 3.1415;

    double dWheelSpeedBR = std::hypot(A, C);
    double dWheelAngleBR = std::atan2(A, C) * 180 / 3.1415;

    // Set wheelspeed and wheel angles.
    m_pModFrontLeft->SetSpeed(-dWheelSpeedFL);
    m_pModFrontLeft->SetAngle(dWheelAngleFL);
    m_pModFrontRight->SetSpeed(dWheelSpeedFR);
    m_pModFrontRight->SetAngle(dWheelAngleFR);
    m_pModBackLeft->SetSpeed(-dWheelSpeedBL);
    m_pModBackLeft->SetAngle(dWheelAngleBL);
    m_pModBackRight->SetSpeed(dWheelSpeedBR);
    m_pModBackRight->SetAngle(dWheelAngleBR);

    // Make some arrays to send to SmartDashboard.
    // @TODO These are SLOOOWWW
    double dWheelAngles [4] =
    {
        m_pModFrontLeft->GetAngle(),
        m_pModFrontRight->GetAngle(),
        m_pModBackLeft->GetAngle(),
        m_pModBackRight->GetAngle()
    };

    double dWheelAngleSetpoints [4] =
    {
        m_pModFrontLeft->GetAngleSetpoint(),
        m_pModFrontRight->GetAngleSetpoint(),
        m_pModBackLeft->GetAngleSetpoint(),
        m_pModBackRight->GetAngleSetpoint()
    };

    // This is a double array because wpi::arrayref doesn't like int arrays (why?)
    double dModStates [4] =
    {
        (double)m_pModFrontLeft->GetState(),
        (double)m_pModFrontRight->GetState(),
        (double)m_pModBackLeft->GetState(),
        (double)m_pModBackRight->GetState()
    };

    // Send values to SmartDashboard.
    frc::SmartDashboard::PutNumberArray("Wheel Angles", dWheelAngles);
    frc::SmartDashboard::PutNumberArray("Mod States", dModStates);
    frc::SmartDashboard::PutNumberArray("Wheel Angle Setpoints", dWheelAngleSetpoints);
    frc::SmartDashboard::PutNumber("dSteering", dSteering);
    frc::SmartDashboard::PutNumber("dForward", dForward);

    // Call swerve module ticks.
    m_pModFrontLeft->Tick();
    m_pModFrontRight->Tick();
    m_pModBackLeft->Tick();
    m_pModBackRight->Tick();
}
/////////////////////////////////////////////////////////////////////////////