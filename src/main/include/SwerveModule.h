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
#include <math.h>

const double m_dDriveProportional = 0.015;
const double m_dDriveIntegral     = 0.001;
const double m_dDriveDerivative   = 0.007;
const double m_dAngleProportional = 0.009;
const double m_dAngleIntegral     = 0.000;
const double m_dAngleDerivative   = 0.00003;
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
class CSwerveModule: public frc::SwerveModuleState
{
 public:
  CSwerveModule(rev::CANSparkMax* pDriveMotor, rev::CANSparkMax* pAzimuthMotor, frc::AnalogInput* pPot, double dDegreeOffset);
  ~CSwerveModule();
  void      Init();
  void      Tick();
  void      SetAngle(double dAngle);
  double    GetAngle();
  double    GetAngleSetpoint();
  void      SetSpeed(double dSpeed);
  double    GetSpeed();
  double    GetSpeedSetpoint();
  void      SetModuleReversed(bool bIsReversed);
  void      SetState(int nState);
  int       GetState();
  void      Stop();
  frc::SwerveModuleState GetModuleState();
  void      SetModuleState(frc::SwerveModuleState desiredState);

 private:
  // Object pointers.
  frc2::PIDController*          m_pAnglePIDController;
  rev::CANSparkMax*             m_pDriveMotor;
  rev::CANSparkMax*             m_pAzimuthMotor;
  frc::AnalogInput*             m_pPot;

  // Member variables.
  int                           m_nState;
  bool                          m_bReady;
  double                        m_dOffset;
  double                        m_dAngleSetpoint;
  double                        m_dSpeedSetpoint;
  enum State {eIdle, eFinding, eManualForward, eManualReverse};
};
/////////////////////////////////////////////////////////////////////////////