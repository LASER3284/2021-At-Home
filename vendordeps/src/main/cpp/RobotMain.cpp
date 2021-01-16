/****************************************************************************
 *	Implements the RobotMain class.
 *
 *	Classes:		CRobotMain
 *
 *	Project:		Swerve Drive
 ****************************************************************************/
#include "RobotMain.h"
/////////////////////////////////////////////////////////////////////////////


/************************************************************************//**
 *  @brief	        RobotMain Contructor.
 *
 *	@param          None
 *
 * 	@implements		frc::TimedRobot			
 ****************************************************************************/
CRobotMain::CRobotMain()
{
	m_pDriveController  = new frc::Joystick(0);
	m_pRobotDrive		= new CDrive(m_pDriveController);
}

/************************************************************************//**
 *  @brief	        RobotMain Destructor.
 *
 *	@param          None
 *
 * 	@implements		frc::TimedRobot			
 ****************************************************************************/
CRobotMain::~CRobotMain()
{
	delete m_pDriveController;
	delete m_pRobotDrive;

	m_pDriveController  = nullptr;
	m_pRobotDrive		= nullptr;
}

/************************************************************************//**
 *  @brief	        Robot initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::RobotInit()
{

}

/************************************************************************//**
 *  @brief	        Robot looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::RobotPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Autonomous initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::AutonomousInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Autonomous looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::AutonomousPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Teleop initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TeleopInit()
{
	m_pRobotDrive->Init();
}

/************************************************************************//**
 *  @brief	        Robot Teleop looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TeleopPeriodic()
{
	m_pRobotDrive->Tick();
}

/************************************************************************//**
 *  @brief	        Robot Disabled initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::DisabledInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Disabled looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::DisabledPeriodic()
{

}

/************************************************************************//**
 *  @brief	        Robot Test initialization routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TestInit()
{

}

/************************************************************************//**
 *  @brief	        Robot Test looped routine.
 *
 *	@param          None
 *
 * 	@retval			Nothing			
 ****************************************************************************/
void CRobotMain::TestPeriodic()
{

}
/////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif
