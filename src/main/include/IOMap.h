/****************************************************************************
    Description:	Defines the Swerve Drive I/O map.

    Classes:		None

    Project:		Swerve Drive
****************************************************************************/
#pragma once
/////////////////////////////////////////////////////////////////////////////

// Robot Dimensions
const double dLength                    = 31.5;
const double dWidth                     = 31.5;

// CAN Device IDs
const int nDriveMotorLeftFront	  		=  	1;		//  Falcon 500 ID for left front drive motor
const int nAzimuthMotorLeftFront  		=  	2;		//  Falcon 500 ID for left front azimuth motor 						
const int nDriveMotorRightFront			=   3;		//  Falcon 500 ID for right front drive motor
const int nAzimuthMotorRightFront  		=   4;		//  Falcon 500 ID for right front azimuth motor						
const int nDriveMotorLeftBack	  		=  	5;		//  Falcon 500 ID for left back drive motor
const int nAzimuthMotorLeftBack	  		=   6;		//  Falcon 500 ID for left back azimuth motor					
const int nDriveMotorRightBack	  		=   7;		//  Falcon 500 ID for right back drive motor
const int nAzimuthMotorRightBack		=   8;		//  Falcon 500 ID for right back azimuth motor
const int nEncoderFrontLeft             =   9;      //  CANCoder ID for front left encoder
const int nEncoderFrontRight            =  10;      //  CANCoder ID for front right encoder
const int nEncoderBackLeft              =  11;      //  CANCoder ID for back left encoder
const int nEncoderBackRight             =  12;      //  CANCoder ID for back right encoder
const int nShooterLeft                  =  13;      //  Spark MAX ID for left shooter motor
const int nShooterRight                 =  14;	    //  Spark MAX ID for right shooter motor
const int nHoodMotor                    =  15;      //  Spark MAX ID for hood motor.
const int nIntakeMotor                  =  16;      //  Spark MAX ID for intake motor.
const int nPreloadMotor                 =  17;      //  Falcon 500 ID for the preload motor.
const int nTurretMotor                  =  18;      //  Spark MAX ID for turret motor.

// Solenoid Channels.
const int nIntakeSolenoid               =   0;      // Solenoid channel for the intake.

// Digital Channels
const int nVisionLEDChannel             =   0;      //  Digital channel ID for LED control line

// Xbox Controller Button Assignments
enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Xbox Controller Axis Assignments
enum XboxAxis			{eLeftAxisX = 0, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};
// Logitech Flight Stick Button Assignments
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
/////////////////////////////////////////////////////////////////////////////