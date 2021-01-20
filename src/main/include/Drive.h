/****************************************************************************
 *  Defines the Drive class.
 *  
 *  Classes:		CDrive
 *  
 *  Project:		Swerve Drive
****************************************************************************/
#pragma once

#include "SwerveModule.h"
#include "IOMap.h"
#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <AHRS.h>

using namespace units;

const double m_dJoystickDeadzone  = 0.12;
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
  void      Init();
  void      Tick();

 private:
  // Object pointers.
  frc::Joystick*                          m_pDriveController;
  rev::CANSparkMax*                       m_pDriveMotorFrontLeft;
  rev::CANSparkMax*                       m_pAzimuthMotorFrontLeft;
  rev::CANSparkMax*                       m_pDriveMotorFrontRight;
  rev::CANSparkMax*                       m_pAzimuthMotorFrontRight;
  rev::CANSparkMax*                       m_pDriveMotorBackLeft;
  rev::CANSparkMax*                       m_pAzimuthMotorBackLeft;
  rev::CANSparkMax*                       m_pDriveMotorBackRight;
  rev::CANSparkMax*                       m_pAzimuthMotorBackRight;
  frc::AnalogPotentiometer*               m_pPotFrontLeft;
  frc::AnalogPotentiometer*               m_pPotFrontRight;
  frc::AnalogPotentiometer*               m_pPotBackLeft;
  frc::AnalogPotentiometer*               m_pPotBackRight;
  CSwerveModule*                          m_pModFrontLeft;
  CSwerveModule*                          m_pModFrontRight;
  CSwerveModule*                          m_pModBackLeft;
  CSwerveModule*                          m_pModBackRight;
  frc::HolonomicDriveController*          m_pHoloDrive;
  AHRS                                    m_Gyro                     {frc::SPI::Port::kMXP};
  frc::Translation2d                      m_FrontLeft                {(inch_t)-dWidth, (inch_t)dLength};
  frc::Translation2d                      m_FrontRight               {(inch_t)dWidth, (inch_t)dLength};
  frc::Translation2d                      m_BackLeft                 {(inch_t)-dWidth, (inch_t)-dLength};
  frc::Translation2d                      m_BackRight                {(inch_t)dWidth, (inch_t)-dLength};
  frc::SwerveDriveKinematics<4>           m_Kinematics               {m_FrontLeft, m_FrontRight, m_BackLeft, m_BackRight};
  frc::SwerveDriveOdometry<4>             m_Odometry                 {m_Kinematics, *new frc::Rotation2d(((radian)m_Gyro.GetYaw())};
};
/////////////////////////////////////////////////////////////////////////////