/****************************************************************************
    Description:	Defines the Swerve Drive I/O map.

    Classes:		None

    Project:		Swerve Drive
****************************************************************************/
#pragma once
/////////////////////////////////////////////////////////////////////////////

// Robot Dimensions.
const double dLength                    = 31.5;
const double dWidth                     = 31.5;

// CAN Device IDs.
const int nDriveMotorLeftFront	  		=  	9;		//  Spark MAX ID for left front drive motor
const int nAzimuthMotorLeftFront  		=  	10;		//  Spark MAX ID for left front azimuth motor 						
const int nDriveMotorRightFront			=   7;		//  Spark MAX ID for right front drive motor
const int nAzimuthMotorRightFront  		=   8;		//  Spark MAX ID for right front azimuth motor						
const int nDriveMotorLeftBack	  		=  	11;		//  Spark MAX ID for left back drive motor
const int nAzimuthMotorLeftBack	  		=  	12;		//  Spark MAX ID for left back azimuth motor					
const int nDriveMotorRightBack	  		=   14;		//  Spark MAX ID for right back drive motor
const int nAzimuthMotorRightBack		=   13;		//  Spark MAX ID for right back azimuth motor						

// Analog Channels.
const int nPotFrontLeft                 =   0;      //  Potentiometer ID for front left encoder
const int nPotFrontRight                =   1;      //  Potentiometer ID for front right encoder
const int nPotBackLeft                  =   2;      //  Potentiometer ID for back left encoder
const int nPotBackRight                 =   3;      //  Potentiometer ID for back right encoder

// Xbox Controller Button Assignments.
enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Xbox Controller Axis Assignments.
enum XboxAxis			{eLeftAxisX = 0, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};
// Logitech Flight Stick Button Assignments.
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
/////////////////////////////////////////////////////////////////////////////