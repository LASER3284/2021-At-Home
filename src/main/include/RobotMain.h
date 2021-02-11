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

  // Object pointers.
  frc::Timer*           m_pTimer;
  frc::Joystick*        m_pDriveController;
  CDrive*               m_pRobotDrive;

  // Declare variables.
  int                   m_nTeleopState;
};
/////////////////////////////////////////////////////////////////////////////