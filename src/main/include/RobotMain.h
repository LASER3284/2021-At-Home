/****************************************************************************
 *  Defines the RobotMain class.
 *
 *	Classes:		CRobotMain
*
*	Project:		Swerve Drive
***************************************************************************/
#pragma once

#include "Drive.h"
#include "Intake.h"
#include "Turret.h"
#include "Shooter.h"
#include "Hood.h"
#include "Hopper.h"
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
///////////////////////////////////////////////////////////////////////////////

/************************************************************************//**
*  @brief	        RobotMain class definition.
*
*	@param          None
****************************************************************************/
class CRobotMain : public frc::TimedRobot
{
public:
    CRobotMain();
    ~CRobotMain();
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    // State machines.
    enum TeleopStates
    {
        eTeleopStopped,
        eTeleopIdle,
        eTeleopIntake,
        eTeleopAiming,
        eTeleopFiring,
        eTeleopCloseRangeFiring,
        eTeleopAutoFiring,
        eTeleopGeneratePath,
        eTeleopFollowing
    };

    enum AutoStates
    {
        eAutoStopped,
        eAutoIdle,
        eAutoBarrelPath1,
        eAutoSlalomPath1,
        eAutoBouncePath1,
        eAutoBasicPath1,
        eAutoTestPath1,
        eSingASong
    };

    // Object pointers.
    frc::Timer*                 m_pTimer;
    frc::Joystick*              m_pDriveController;
    frc::Joystick*              m_pAuxController;
    CDrive*                     m_pRobotDrive;
    CIntake*                    m_pIntake;
    CTurret*                    m_pTurret;
    CShooter*                   m_pShooter;
    CHood*                      m_pHood;
    CHopper*                    m_pHopper;
    SendableChooser<string>*    m_pAutonomousChooser;
    frc::Compressor*            m_pCompressor;

    // Declare variables.
    int                   m_nAutoState;
    int                   m_nTeleopState;
    double                m_dStartTime;
};
/////////////////////////////////////////////////////////////////////////////