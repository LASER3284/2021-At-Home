/****************************************************************************
    Description:	Defines the Poses used for autonomous.
    Classes:		CTrajectoryConstants
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
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
    ePath
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

            case ePath :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/Path2.wpilib.json");
                break;

            default :
                // Read and store the trajectory from a pre-generated JSON file.
                m_SelectedTrajectory = TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/paths/output/Path1.wpilib.json");
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
    const meters_per_second_squared_t kMaxTranslationAcceleration = 7.2_mps_sq;
    const radians_per_second_t kMaxRotationSpeed = 6.28_rad_per_s;
    const radians_per_second_t kMaxRotationAcceleration = 3.14_rad_per_s;

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