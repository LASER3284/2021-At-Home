/****************************************************************************
 *  Defines the CShooter control class.
 * 
 *  Classes:		CShooter
 * 
 *  Project:		2021 Infinite Recharge At-Home Robot Code.
 * 
 *  Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Shooter_h
#define Shooter_h

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalOutput.h>
#include "IOMap.h"

using namespace frc;

// Shooter Constants.
const double dShooterOpenLoopRamp		=     0.250;
const double dShooterClosedLoopRamp		=     0.250;
const double dShooterManualFwdSpeed		=	  0.500;
const double dShooterManualRevSpeed		=	 -0.500;
const double dShooterMaxVelocity		= 	5700.00;
const double dShooterIdleVelocity		=	3400.00;
const double dShooterFiringVelocity     =   4500.00;
const double dShooterCloseRangeVelocity =   4500.00;
const double dShooterMinVelocity		=	 200.00;
const double dShooterProportional       =      4e-4;
const double dShooterIntegral           =       0.0;
const double dShooterDerivative         =       0.0;
const double dShooterFeedForward        =     17e-5;
const double dShooterTolerance          =     130.0; // 180

// Shooter enum.
enum ShooterState	{eShooterStopped, eShooterIdle, eShooterFinding, eShooterManualFwd, eShooterManualRev};
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**
 *  @brief      	CShooter class definition.
 *
 *  @param		    None
 ***************************************************************************/
class CShooter
{
public:
    CShooter();
    ~CShooter();

    // Public methods.
    void 			Init();
    void 			Tick();
    void 			Stop();
    void			SetPID(double dProportional, double dIntegral, double dDerivative, double dFeedForward);
    void 			SetSetpoint(double dSetpoint);
    void 			SetTolerance(double dTolerance);
    bool			IsAtSetpoint();
    bool			IsReady()													{	return m_bIsReady;		                            };
    void 			SetState(ShooterState nState)								{	m_nState = nState;							        };
    double			GetActual()													{	return m_pLeftShooter->GetEncoder().GetVelocity();	};
    double 			GetSetpoint()												{	return m_dSetpoint;							        };
    double			GetTolerance()												{	return m_dTolerance;							    };
    ShooterState 	GetState()													{	return m_nState;								    };		

    void            SetVisionLED(bool bEnabled = true);

private:
    // Object pointers.
    rev::CANSparkMax*		m_pLeftShooter;
    rev::CANSparkMax*		m_pRightShooter;
    DigitalOutput*          m_pVisionSwitch;
    frc::Timer*					m_pTimer;

    // Declare variables.
    bool			m_bIsReady;
    bool			m_bVisionTracking;
    bool			m_bMotionMagic;
    double 			m_dProportional;
    double 			m_dIntegral;
    double 			m_dDerivative;
    double 			m_dFeedForward;
    double 			m_dTolerance;
    double 			m_dSetpoint;
    double			m_dActual;
    double			m_dMaxFindingTime;
    double			m_dFindingStartTime;
    ShooterState	m_nState;
};
/////////////////////////////////////////////////////////////////////////////
#endif