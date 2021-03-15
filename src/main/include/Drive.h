/****************************************************************************
 *  Defines the Drive class.
 *  
 *  Classes:		CDrive
 *  
 *  Project:		2021 Infinite Recharge At-Home Robot Code.
 * 
 *  Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#pragma once

#include "IOMap.h"
#include "SwerveModule.h"
#include "TrajectoryConstants.h"
#include <frc/Joystick.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <ctre/phoenix/music/Orchestra.h>
#include <AHRS.h>

using namespace frc;
using namespace frc2;
using namespace ctre::phoenix;
using namespace units;

const double m_dJoystickDeadzone  = 0.1;
const double m_dTeleopMultiplier  = 4.0;
// PID gains for the X translation.
const double m_dPIDXkP            = 3.0;
const double m_dPIDXkI            = 2.3;
const double m_dPIDXkD            = 0.02;
// PID gains for the Y translation
const double m_dPIDYkP            = 3.0;
const double m_dPIDYkI            = 2.3;
const double m_dPIDYkD            = 0.02;
// PID gains for the Theta rotation.
const double m_dPIDThetakP        = 4.5;
const double m_dPIDThetakI        = 1.5;
const double m_dPIDThetakD        = 0.2;
// Setup the swerve kinematics.
const Translation2d FrontLeft                      = Translation2d((inch_t)(dWidth / 2), (inch_t)(dLength / 2));
const Translation2d FrontRight                     = Translation2d((inch_t)(dWidth / 2), (inch_t)(-dLength / 2));
const Translation2d BackLeft                       = Translation2d((inch_t)(-dWidth / 2), (inch_t)(dLength / 2));
const Translation2d BackRight                      = Translation2d((inch_t)(-dWidth / 2), (inch_t)(-dLength / 2));
const SwerveDriveKinematics<4> m_kKinematics       = SwerveDriveKinematics<4>(FrontLeft, FrontRight, BackLeft, BackRight);
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**
 *  @brief	        CDrive class definition.
 *
 *	@param          pJoystick pointer to the driver's joystick.
 ****************************************************************************/
class CDrive
{
 public:
  CDrive(frc::Joystick* pDriveController);
  ~CDrive();
  void            Init();
  void            Tick();
  void            Stop();
  void            SetJoystickControl(bool bJoystickControl);
  void            SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  Pose2d          GetRobotPose();
  void            ResetOdometry();
  double          GetTrajectoryTotalTime();
  void            SetSelectedTrajectory(int nAutoState);
  void            GenerateTrajectoryFromCurrentPosition();
  void            FollowTrajectory(double dElapsedTime);
  double          GetYaw();
  bool            GetJoystickControl()          {  return m_bJoystickControl;  };
  music::Orchestra* GetOrchestra()              {  return m_pOrchestra;        };

 private:     
  // Object pointers.     
  Joystick*                          m_pDriveController;
  frc::Timer*                        m_pTimer;
  motorcontrol::can::TalonFX*        m_pDriveMotorFrontLeft;
  motorcontrol::can::TalonFX*        m_pAzimuthMotorFrontLeft;
  motorcontrol::can::TalonFX*        m_pDriveMotorFrontRight;
  motorcontrol::can::TalonFX*        m_pAzimuthMotorFrontRight;
  motorcontrol::can::TalonFX*        m_pDriveMotorBackLeft;
  motorcontrol::can::TalonFX*        m_pAzimuthMotorBackLeft;
  motorcontrol::can::TalonFX*        m_pDriveMotorBackRight;
  motorcontrol::can::TalonFX*        m_pAzimuthMotorBackRight;
  sensors::CANCoder*                 m_pEncoderFrontLeft;
  sensors::CANCoder*                 m_pEncoderFrontRight;
  sensors::CANCoder*                 m_pEncoderBackLeft;
  sensors::CANCoder*                 m_pEncoderBackRight;
  CSwerveModule*                     m_pModFrontLeft;
  CSwerveModule*                     m_pModFrontRight;
  CSwerveModule*                     m_pModBackLeft;
  CSwerveModule*                     m_pModBackRight;
  AHRS*                              m_pGyro;
  SwerveDriveOdometry<4>*            m_pSwerveDriveOdometry;
  HolonomicDriveController*          m_pHolonomicDriveController;
  CTrajectoryConstants               TrajectoryConstants;
  frc2::SwerveControllerCommand<4>*  m_SwerveControllerCommand;
  music::Orchestra*                  m_pOrchestra;
  

  // Member Variables.
  bool              m_bJoystickControl;
  double            m_dYAxis;
  double            m_dXAxis;
  double            m_dRotateCW;
};
/////////////////////////////////////////////////////////////////////////////