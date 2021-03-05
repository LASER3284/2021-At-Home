/****************************************************************************
    Description:	Defines the CHopper control class.
    Classes:		CHopper
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Hopper_h
#define Hopper_h

#include <rev/CANSparkMax.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Hopper Contants.
const double dHopperPreloadSpeed  	= 1.0;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CHopper class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CHopper
{
public:
    CHopper();
    ~CHopper();

    // Public Methods.
    void Init();
    void Preload(bool bEnabled = true);

private:
    // Object pointers.
    CANSparkMax*        m_pShooterFeeder;
};
/////////////////////////////////////////////////////////////////////////////
#endif
