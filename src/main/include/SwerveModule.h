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

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <math.h>

using namespace ctre::phoenix;

const int m_dEncoderTicksPerRev     = 2048;
const double m_dEncoderConvert      = ((1 / (8.16)) * 10) * (3.1415926  * (4 * 0.0254));
const double m_dDriveProportional   = 0.003;
const double m_dDriveIntegral       = 0.000;
const double m_dDriveDerivative     = 0.000;
const double m_dDriveFeedForward    = 0.049;
const double m_dAngleProportional   = 0.007;
const double m_dAngleIntegral       = 0.000;
const double m_dAngleDerivative     = 0.000;
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
  CSwerveModule(motorcontrol::can::TalonFX* pDriveMotor, motorcontrol::can::TalonFX* pAzimuthMotor, sensors::CANCoder* pEncoder, double dDegreeOffset);
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
  motorcontrol::can::TalonFX*   m_pDriveMotor;
  motorcontrol::can::TalonFX*   m_pAzimuthMotor;
  sensors::CANCoder*            m_pEncoder;

  // Member variables.
  int                           m_nState;
  bool                          m_bReady;
  double                        m_dOffset;
  double                        m_dAngleSetpoint;
  double                        m_dSpeedSetpoint;
  enum State {eIdle, eFinding, eManualForward, eManualReverse};
};
/////////////////////////////////////////////////////////////////////////////