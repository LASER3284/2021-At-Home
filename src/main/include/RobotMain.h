/****************************************************************************
 *  Defines the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		2021 Infinite Recharge At-Home Robot Code.
 *
 *  Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
***************************************************************************/
#pragma once

#include "Drive.h"
#include "Intake.h"
#include "Turret.h"
#include "Shooter.h"
#include "Hood.h"
#include "Hopper.h"
#include "Lift.h"
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>

const std::string kSong = "heart-shaped-box.chrp";
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
        eTeleopIntake2,
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
        eAutoShoot,
        eAutoShoot2,
        eAutoShoot3,
        eAutoShoot4,
        eAutoBarrelPath1,
        eAutoSlalomPath1,
        eAutoBouncePath1,
        eAutoBasicPath1,
        eAutoGalacticSearch1,
        eAutoGalacticSearch2,
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
    CLift*                      m_pLift;
    SendableChooser<string>*    m_pAutonomousChooser;
    frc::Compressor*            m_pCompressor;

    // Declare variables.
    int                   m_nAutoState;
    int                   m_nTeleopState;
    double                m_dStartTime;
};
/////////////////////////////////////////////////////////////////////////////