#include <cmath>
#include <vector>
#include <iostream>
#include "Constants.hpp"
#include "geometry/Geometry.hpp"
#include "utils/CKMath.hpp"
#include "utils/Singleton.hpp"

class RobotState : public Singleton<RobotState> {
    friend Singleton;
public:

    typedef typename std::map<double, ck::geometry::Pose2d>::const_iterator Iter;

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    void reset(double start_time, ck::geometry::Pose2d initial_field_to_vehicle);
    void resetDistanceDriven();

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    ck::geometry::Pose2d getFieldToVehicle(double timestamp);
    ck::geometry::Pose2d getPredictedFieldToVehicle(double lookahead_time);
    ck::geometry::Pose2d getFieldToLidar(double timestamp);
    ck::geometry::Pose2d getLatestFieldToVehicle();
    void addFieldToVehicleObservation(double timestamp, ck::geometry::Pose2d observation);
    void addObservations(double timestamp, ck::geometry::Twist2d measured_velocity,
                                                ck::geometry::Twist2d predicted_velocity);
    ck::geometry::Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, ck::geometry::Rotation2d current_gyro_angle);
    double getDistanceDriven();
    ck::geometry::Twist2d getPredictedVelocity();
    ck::geometry::Twist2d getMeasuredVelocity();

private:
    static const int kObservationBufferSize = 100;
    const ck::geometry::Translation2d t2d_;
    const ck::geometry::Rotation2d r2d_;
    const ck::geometry::Pose2d kVehicleToLidar;

    // FPGATimestamp -> Pose2d or Rotation2d
    std::map<double, ck::geometry::Pose2d> field_to_vehicle_;
    ck::geometry::Twist2d vehicle_velocity_predicted_;
    ck::geometry::Twist2d vehicle_velocity_measured_;
    double distance_driven_;
    RobotState();
};
