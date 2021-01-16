/****************************************************************************
 *  Defines the Drive class.
 *  
 *  Classes:		CDrive
 *  
 *  Project:		Swerve Drive
****************************************************************************/
#pragma once

#include "SwerveModule.h"
#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <AHRS.h>

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
  frc::Joystick*                m_pDriveController;
  rev::CANSparkMax*             m_pDriveMotorFrontLeft;
  rev::CANSparkMax*             m_pAzimuthMotorFrontLeft;
  rev::CANSparkMax*             m_pDriveMotorFrontRight;
  rev::CANSparkMax*             m_pAzimuthMotorFrontRight;
  rev::CANSparkMax*             m_pDriveMotorBackLeft;
  rev::CANSparkMax*             m_pAzimuthMotorBackLeft;
  rev::CANSparkMax*             m_pDriveMotorBackRight;
  rev::CANSparkMax*             m_pAzimuthMotorBackRight;
  frc::AnalogPotentiometer*     m_pPotFrontLeft;
  frc::AnalogPotentiometer*     m_pPotFrontRight;
  frc::AnalogPotentiometer*     m_pPotBackLeft;
  frc::AnalogPotentiometer*     m_pPotBackRight;
  CSwerveModule*                m_pModFrontLeft;
  CSwerveModule*                m_pModFrontRight;
  CSwerveModule*                m_pModBackLeft;
  CSwerveModule*                m_pModBackRight;
  AHRS*                         m_pGyro;
  //frc::SwerveDriveKinematics<4> m_pKinematics;
};
/////////////////////////////////////////////////////////////////////////////