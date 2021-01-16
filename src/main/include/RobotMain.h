/****************************************************************************
 *  Defines the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		Swerve Drive
 ***************************************************************************/
#pragma once

#include "Drive.h"
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
  // Object pointers.
  frc::Joystick* m_pDriveController;
  CDrive*        m_pRobotDrive;
};
/////////////////////////////////////////////////////////////////////////////