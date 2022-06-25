#include "RobotState.hpp"
#include "subsystems/Drive.hpp"
#include "utils/Kinematics.hpp"

void RobotState::reset(double start_time, ck::geometry::Pose2d initial_field_to_vehicle) {
    field_to_vehicle_.clear();
    field_to_vehicle_.insert({start_time, initial_field_to_vehicle});
    Drive::getInstance().setHeading(initial_field_to_vehicle.getRotation());
    vehicle_velocity_predicted_ = ck::geometry::Twist2d::identity();
    vehicle_velocity_measured_ = ck::geometry::Twist2d::identity();
    distance_driven_ = 0.0;
}

void RobotState::resetDistanceDriven() {
    distance_driven_ = 0.0;
}

/**
 * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
 * to fill in the gaps.
 */
ck::geometry::Pose2d RobotState::getFieldToVehicle(double timestamp) {
    return ck::math::interpolateGeometry2d<double, ck::geometry::Pose2d> (field_to_vehicle_, timestamp);
}

ck::geometry::Pose2d RobotState::getLatestFieldToVehicle() {
    RobotState::Iter it = field_to_vehicle_.end();
    return it->second;
}

ck::geometry::Pose2d RobotState::getPredictedFieldToVehicle(double lookahead_time) {

    RobotState::Iter it = field_to_vehicle_.end();
    return it->second.transformBy(Pose2d::exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
}

ck::geometry::Pose2d RobotState::getFieldToLidar(double timestamp) {
    return getFieldToVehicle(timestamp).transformBy(kVehicleToLidar);
}

void RobotState::addFieldToVehicleObservation(double timestamp, Pose2d observation) {
    field_to_vehicle_.insert({timestamp, observation});
}

void RobotState::addObservations(double timestamp, Twist2d measured_velocity,
                                            Twist2d predicted_velocity) {
    RobotState::Iter it = field_to_vehicle_.end();
    addFieldToVehicleObservation(timestamp,
            ck::math::Kinematics::integrateForwardKinematics(it->second, measured_velocity));
    vehicle_velocity_measured_ = measured_velocity;
    vehicle_velocity_predicted_ = predicted_velocity;
}

ck::geometry::Twist2d RobotState::generateOdometryFromSensors(double left_encoder_delta_distance, double
        right_encoder_delta_distance, Rotation2d current_gyro_angle) {
//		final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
    RobotState::Iter it = field_to_vehicle_.end();
    const ck::geometry::Twist2d delta = ck::math::Kinematics::forwardKinematics(it->second.getRotation(),
            left_encoder_delta_distance, right_encoder_delta_distance,
            current_gyro_angle);
    distance_driven_ += delta.dx; //do we care about dy here?
    return delta;
}

double RobotState::getDistanceDriven() {
    return distance_driven_;
}

ck::geometry::Twist2d RobotState::getPredictedVelocity() {
    return vehicle_velocity_predicted_;
}

ck::geometry::Twist2d RobotState::getMeasuredVelocity() {
    return vehicle_velocity_measured_;
}

RobotState::RobotState() :
t2d_ {K_LIDAR_X_OFFSET, K_LIDAR_Y_OFFSET},
r2d_ {ck::geometry::Rotation2d::fromDegrees(K_LIDAR_YAW_ANGLE_DEGREES)},
kVehicleToLidar { t2d_, r2d_}
{
    ck::geometry::Pose2d reset_pose_;
    reset(0, reset_pose_);
}
