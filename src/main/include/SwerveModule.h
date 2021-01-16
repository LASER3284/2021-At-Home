/****************************************************************************
 *  Defines the SwerveModule class.
 *  
 *  Classes:		    CSwerveModule
 *  
 *  Project:		    Swerve Drive
 * 
 * 	Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
 ****************************************************************************/
#pragma once

#include <rev/CANSparkMax.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
/////////////////////////////////////////////////////////////////////////////


/**************************************************************************//**
 *  @brief	        CSwerveModule class definition.
 *
 *	@param          pDriveMotor pointer to drive motor.
 *  @param          pAzimuthMotor pointer to the azimuth motor.
 *  @param          pPot pointer to the analog encoder, used as a
 *                  potentiometer for compatibility.
 *  @param          dDegreeOffset value of the offset of the azimuth
 *                  encoder relative to zero.
 ******************************************************************************/
class CSwerveModule
{
 public:
  CSwerveModule(rev::CANSparkMax* pDriveMotor, rev::CANSparkMax* pAzimuthMotor, frc::AnalogPotentiometer* pPot, double dDegreeOffset);
  ~CSwerveModule();
  void      Init();
  void      Tick();
  void      SetAngle(double dAngle);
  double    GetAngle();
  double    GetAngleSetpoint();
  void      SetSpeed(double dSpeed);
  double    GetSpeed();
  double    GetSpeedSetpoint();
  void      SetState(int nState);
  int       GetState();
  void      Stop();
  frc::SwerveModuleState  GetModuleState();

 private:
  // Object pointers.
  frc2::PIDController*          m_pPIDController;
  rev::CANSparkMax*             m_pDriveMotor;
  rev::CANSparkMax*             m_pAzimuthMotor;
  frc::AnalogPotentiometer*     m_pPot;

  // Member variables.
  int                           m_nState;
  bool                          m_bReady;
  double                        m_dOffset;
  double                        m_dAngleSetpoint;
  double                        m_dSpeedSetpoint;
  enum State {eIdle, eFinding, eManualForward, eManualReverse};
};
/////////////////////////////////////////////////////////////////////////////