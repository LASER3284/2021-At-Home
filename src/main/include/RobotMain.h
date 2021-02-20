/****************************************************************************
 *  Defines the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		Swerve Drive
 ***************************************************************************/
#pragma once

#include "Drive.h"
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
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
  CDrive*                     m_pRobotDrive;
  SendableChooser<string>*    m_pAutonomousChooser;

  // Declare variables.
  int                   m_nAutoState;
  int                   m_nTeleopState;
  double                m_dStartTime;
};
/////////////////////////////////////////////////////////////////////////////