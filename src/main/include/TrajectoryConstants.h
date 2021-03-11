/****************************************************************************
    Description:	Defines the Poses used for autonomous.

    Classes:		CTrajectoryConstants

    Project:		2021 Infinite Recharge At-Home Robot Code.

    Copyright © 2021 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef TrajectoryConstants_h
#define TrajectoryConstants_h

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>

using namespace frc;
using namespace units;
using namespace std;
using namespace wpi;

// Trajectory list enum.
enum TrajectoryList 
{ 
    eTestPath1 = 1,
    eBarrelPath,
    eSlalomPath,
    eBouncePath,
    eBasicPath
};
/////////////////////////////////////////////////////////////////////////////

class CTrajectoryConstants
{
public:
    void SelectTrajectory(int nTrajectory)
    {
        // Retrieve the correct trajectory.
        switch(nTrajectory)
        {
            case eTestPath1 :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/Path1.wpilib.json");
                break;

            case eBarrelPath :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/BarrelPath1.wpilib.json");
                break;

            case eSlalomPath :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/SlalomPath1.wpilib.json");
                break;

            case eBouncePath :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/BouncePath1.wpilib.json");
                break;

            case eBasicPath :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/BasicPath1.wpilib.json");
                break;

            default :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/BasicPath1.wpilib.json");
                break;
        }
    }

    void SelectTrajectory(Trajectory Path)
    {
        m_SelectedTrajectory = Path;
    }

    // One-line methods.
    Pose2d GetSelectedTrajectoryStartPoint()    {   return m_SelectedTrajectory.InitialPose();              };
    Trajectory GetSelectedTrajectory()          {   return m_SelectedTrajectory;                            };
    double GetSelectedTrajectoryTotalTime()     {   return double(m_SelectedTrajectory.TotalTime());        };

    // Configure trajectory properties.
    const meters_per_second_t kMaxTranslationSpeed = 4.0_mps;
    const meters_per_second_squared_t kMaxTranslationAcceleration = 3_mps_sq;
    const radians_per_second_t kMaxRotationSpeed = 3.0_rad_per_s;
    const radians_per_second_t kMaxRotationAcceleration = 4.0_rad_per_s;

    // Preset Teleop Trajectory.
    vector<Pose2d> PresetWaypoints
    {
        Pose2d 
        {
            2.0_m,				    // X ending position on field in feet.
            0.0_m,					// Y ending position on field in feet.
            Rotation2d(0_deg)		// Ending rotation on field in degrees.
        }
    };

private:
    Trajectory m_SelectedTrajectory;
};
/////////////////////////////////////////////////////////////////////////////
#endif