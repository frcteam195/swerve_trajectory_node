#pragma once

#include "plannners/DriveMotionPlanner.hpp"
#include "paths/MirroredTrajectory.hpp"
#include "utils/Singleton.hpp"

namespace ck
{
    namespace paths
    {
        class TrajectorySet
        {
        public:
            paths::MirroredTrajectory *test90DegPath;

            TrajectorySet(void);
        };

        class TrajectoryGenerator : public Singleton<TrajectoryGenerator>
        {
            friend Singleton;

        private:
            // static const double kMaxVoltage = 9.0;
            // static const double kMaxAccel = 40.0;
            // static const double kMaxVelocity = 60.0;

            // static const double kFirstPathMaxVoltage = 9.0;
            // static const double kFirstPathMaxAccel = 80.0;
            // static const double kFirstPathMaxVel = 100.0;

            // static const double kMaxCentripetalAccel = 100.0;

            planners::DriveMotionPlanner *mMotionPlanner = NULL;
            TrajectorySet *mTrajectorySet = NULL;

            TrajectoryGenerator(void);

        public:
            MirroredTrajectory generateMirroredTrajectory(bool reversed,
                                                          std::vector<geometry::Pose2d> waypoints,
                                                          /* TODO: Implement Timing Constraint, */
                                                          double maximumVelocity,
                                                          double maximumAcceleration,
                                                          double maximumVoltage);

            void generateTrajectories(void);

            TrajectorySet *getTrajectorySet(void);
        };

    } // namespace paths
} // namespace ck