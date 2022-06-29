#pragma once

#include "ck_utilities/geometry/Pose2dWithCurvature.hpp"
#include "physics/DifferentialDrive.hpp"
#include "trajectory/timing/TimingConstraint.hpp"
#include "trajectory/timing/TimedState.hpp"
#include "trajectory/Trajectory.hpp"
#include "trajectory/TrajectoryIterator.hpp"
#include "ck_utilities/CKMath.hpp"

namespace ck
{
    namespace planners
    {
        class Output
        {
        public:
            Output() {}

            Output(double leftVelocity,
                   double rightVelocity,
                   double leftAcceleration,
                   double rightAcceleration,
                   double leftFeedForwardVoltage,
                   double rightFeedForwardVoltage);

            double mLeftVelocity;  // Radians per Second
            double mRightVelocity; // Radians per Second

            double mLeftAcceleration;  // Radians per Second^2
            double mRightAcceleration; // Radians per Second^2

            double mLeftFeedForwardVoltage;
            double mRightFeedForwardVoltage;

            void flip(void);
        };

        class DriveMotionPlanner
        {
        public:
            enum FollowerType
            {
                FEEDFORWARD_ONLY,
                PURE_PURSUIT,
                PID,
                NONLINEAR_FEEDBACK
            };

            DriveMotionPlanner(void);

            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> generateTrajectory(bool reversed,
                                                                                                                     std::vector<geometry::Pose2d> waypoints,
                                                                                                                     std::vector<trajectory::timing::TimingConstraint<geometry::Pose2dWithCurvature>> constraints,
                                                                                                                     double maximumVelocity,   // Inches per Second
                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                     double maximumVoltage);
                                                                                                                     
            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> generateTrajectory(bool reversed,
                                                                                                                     std::vector<geometry::Pose2d> waypoints,
                                                                                                                     std::vector<trajectory::timing::TimingConstraint<geometry::Pose2dWithCurvature>> constraints,
                                                                                                                     double startVelocity, // Inches per Second
                                                                                                                     double endVelocity, // Inches per Second
                                                                                                                     double maximumVelocity, // Inches per Second
                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                     double maximumVoltage);
            bool isDone(void);

            void reset(void);

            void setFollowerType(FollowerType type);

            void setTrajectory(trajectory::TrajectoryIterator<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> trajectory);

        private:
            static constexpr double kMaxDx = 2.0;
            static constexpr double kMaxDy = 0.25;
            static constexpr double kMaxDtheta = 0.0872665; // 5 Degrees

            trajectory::TrajectoryIterator<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> *mCurrentTrajectory = NULL;
            trajectory::timing::TimedState<geometry::Pose2dWithCurvature> *mSetpoint = new trajectory::timing::TimedState<geometry::Pose2dWithCurvature>(geometry::Pose2dWithCurvature::identity());
    
            FollowerType mFollowerType = FollowerType::NONLINEAR_FEEDBACK;
            
            bool mIsReversed = false;

            physics::DifferentialDrive *mModel;

            geometry::Pose2d mError = geometry::Pose2d::identity();
            double mLastTime = ck::math::POS_INF_F;
            Output *mOutput = new Output();
        };

    } // namespace planners
} // namespace ck