#include "plannners/DriveMotionPlanner.hpp"

#include "Constants.hpp"
#include "ck_utilities/physics/DCMotorTransmission.hpp"
#include "trajectory/timing/CentripetalAccelerationConstraint.hpp"
#include "trajectory/timing/DifferentialDriveDynamicsConstraint.hpp"
#include "trajectory/timing/TimingUtil.hpp"
#include "trajectory/timing/TimingConstraint.hpp"
#include "trajectory/TrajectoryUtil.hpp"

namespace ck
{
    namespace planners
    {
        Output::Output(double leftVelocity,
                   double rightVelocity,
                   double leftAcceleration,
                   double rightAcceleration,
                   double leftFeedForwardVoltage,
                   double rightFeedForwardVoltage)
        {
            mLeftVelocity = leftVelocity;
            mRightVelocity = rightVelocity;

            mLeftAcceleration = leftAcceleration;
            mRightAcceleration = rightAcceleration;

            mLeftFeedForwardVoltage = leftFeedForwardVoltage;
            mRightFeedForwardVoltage = rightFeedForwardVoltage;
        }

        void Output::flip(void)
        {
            double tempLeftVelocity = mLeftVelocity;
            mLeftVelocity = -mRightVelocity;
            mRightVelocity = -tempLeftVelocity;

            double tempLeftAcceleration = mLeftAcceleration;
            mLeftAcceleration = -mRightAcceleration;
            mRightAcceleration = -tempLeftAcceleration;

            double tempLeftFeedForwardVoltage = mLeftFeedForwardVoltage;
            mLeftFeedForwardVoltage = -mRightFeedForwardVoltage;
            mRightFeedForwardVoltage = -tempLeftFeedForwardVoltage;
        }

        DriveMotionPlanner::DriveMotionPlanner(void)
        {
            double wheelRadiusMeters = math::inches_to_meters(K_DRIVE_WHEEL_RADIUS_INCHES);
            double wheelRadiusSquaredMeters = wheelRadiusMeters * wheelRadiusMeters;
            double torquePerVolt = wheelRadiusSquaredMeters * K_ROBOT_LINEAR_INERTIA / (2.0 * K_DRIVE_KA);

            physics::DCMotorTransmission transmission = physics::DCMotorTransmission(1.0 / K_DRIVE_KV, torquePerVolt, K_DRIVE_V_INTERCEPT);

            double effectiveWheelbaseRadius = math::inches_to_meters(K_DRIVE_WHEEL_TRACK_WIDTH_INCHES / 2.0 * K_TRACK_SCRUB_FACTOR);

            mModel = new physics::DifferentialDrive(K_ROBOT_LINEAR_INERTIA,
                                                    K_ROBOT_ANGULAR_INERTIA,
                                                    K_ROBOT_ANGULAR_DRAG,
                                                    wheelRadiusMeters,
                                                    effectiveWheelbaseRadius,
                                                    transmission, transmission);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                                     std::vector<geometry::Pose2d> waypoints,
                                                                                                                                     double maximumVelocity,     // Inches per Second
                                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                                     double maximumVoltage)
        {
            return this->generateTrajectory(reversed,
                                            waypoints,
                                            0.0,
                                            0.0,
                                            maximumVelocity,
                                            maximumAcceleration,
                                            maximumVoltage);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                                     std::vector<geometry::Pose2d> waypoints,
                                                                                                                                     double startVelocity,       // Inches per Second
                                                                                                                                     double endVelocity,         // Inches per Second
                                                                                                                                     double maximumVelocity,     // Inches per Second
                                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                                     double maximumVoltage)
        {
            std::vector<geometry::Pose2d> modifiedWaypoints = waypoints;
            geometry::Pose2d flipFactor = geometry::Pose2d::fromRotation(geometry::Rotation2d(-1.0, 0.0, false));

            // Flip the waypoints if the trajectory is reversed.
            if (reversed)
            {
                for (size_t i = 0; i < waypoints.size(); ++i)
                {
                    modifiedWaypoints.push_back(waypoints[i].transformBy(flipFactor));
                }
            }

            // Create a trajectory from the splines.
            trajectory::Trajectory<geometry::Pose2dWithCurvature> trajectory = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(modifiedWaypoints, kMaxDx, kMaxDy, kMaxDtheta);

            if (reversed)
            {
                std::vector<geometry::Pose2dWithCurvature> flippedCurvaturePoints;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    flippedCurvaturePoints.push_back(geometry::Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flipFactor),
                                                                                   -trajectory.getState(i).getCurvature(),
                                                                                   trajectory.getState(i).getDCurvatureDs()));
                }

                trajectory = trajectory::Trajectory<geometry::Pose2dWithCurvature>(flippedCurvaturePoints);
            }

            // Create the constraint that the robot must be able to traverse the trajectory without ever applying more than the specified voltage.
            trajectory::timing::DifferentialDriveDynamicsConstraint<geometry::Pose2dWithCurvature> driveConstraints(*this->mModel, maximumVoltage);
            trajectory::timing::CentripetalAccelerationConstraint centripetalAccelConstraint(kMaxCentripetalAccel);

            std::vector<trajectory::timing::TimingConstraint<geometry::Pose2dWithCurvature> *> constraints {&driveConstraints, &centripetalAccelConstraint};

            trajectory::DistanceView<geometry::Pose2dWithCurvature> distanceView(trajectory);
            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> timedTrajectory = trajectory::timing::TimingUtil::timeParameterizeTrajectory(reversed,
                                                                                                                                                                               distanceView,
                                                                                                                                                                               kMaxDx,
                                                                                                                                                                               constraints,
                                                                                                                                                                               startVelocity,
                                                                                                                                                                               endVelocity,
                                                                                                                                                                               maximumVelocity,
                                                                                                                                                                               maximumAcceleration);

            return timedTrajectory;
        }

        bool DriveMotionPlanner::isDone(void)
        {
            return mCurrentTrajectory != NULL && mCurrentTrajectory->isDone();
        }

        void DriveMotionPlanner::reset(void)
        {
            mError = geometry::Pose2d::identity();
            mOutput = new Output();
            mLastTime = ck::math::POS_INF_F;
        }

        void DriveMotionPlanner::setFollowerType(FollowerType type)
        {
            mFollowerType = type;
        }

        void DriveMotionPlanner::setTrajectory(trajectory::TrajectoryIterator<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> trajectory)
        {
            *mCurrentTrajectory = trajectory;
            *mSetpoint = trajectory.getState();

            for (int i = 0; i < trajectory.trajectory().length(); ++i)
            {
                if (trajectory.trajectory().getState(i).velocity() > ck::math::kEpsilon)
                {
                    mIsReversed = false;
                    break;
                }
                else if (trajectory.trajectory().getState(i).velocity() < -ck::math::kEpsilon)
                {
                    mIsReversed = true;
                    break;
                }
            }
        }

    } // namespace planners
} // namespace ck