#include "RobotState.hpp"
#include "subsystems/Drive.hpp"
#include "subsystems/RobotStateEstimator.hpp"
#include "utils/Kinematics.hpp"

DataReporter* RobotStateEstimator::logReporter = &NetworkDataReporter::getInstance();

RobotStateEstimator::RobotStateEstimator()
{
    mObjList.reserve(10);
    mPeriodicIO.left_encoder_prev_distance_ = 0.0;
    mPeriodicIO.right_encoder_prev_distance_ = 0.0;
}

void RobotStateEstimator::stop() {
    // No-op
}

bool RobotStateEstimator::isSystemFaulted() {
    return false;
}

bool RobotStateEstimator::runDiagnostics() {
    return true;
}

void RobotStateEstimator::registerEnabledLoops(ILooper & enabledLooper) {
    enabledLooper.registerLoop(*this);
}

void RobotStateEstimator::onFirstStart(double timestamp) {}

void RobotStateEstimator::onStart(double timestamp) {
    mPeriodicIO.left_encoder_prev_distance_ = Drive::getInstance().getLeftEncoderDistance();
    mPeriodicIO.right_encoder_prev_distance_ = Drive::getInstance().getRightEncoderDistance();
}

void RobotStateEstimator::onStop(double timestamp) {
    // no-op
}

RobotStateEstimator::PeriodicIO::PeriodicIO()
:DECLARE_REPORTED(logReporter,left_distance)
,DECLARE_REPORTED(logReporter,right_distance)
,DECLARE_REPORTED(logReporter,delta_left)
,DECLARE_REPORTED(logReporter,delta_right)
// TODO after network types for Twist and Rotation
//,DECLARE_REPORTED(logReporter,gyro_angle)
//,DECLARE_REPORTED(logReporter,odometry_velocity)
//,DECLARE_REPORTED(logReporter,predicted_velocity)
,DECLARE_REPORTED(logReporter,left_encoder_prev_distance_)
,DECLARE_REPORTED(logReporter,right_encoder_prev_distance_)
,DECLARE_REPORTED(logReporter, robot_state_loop_time)
{

}

void RobotStateEstimator::onLoop(double timestamp) {
    mPeriodicIO.left_distance = Drive::getInstance().getLeftEncoderDistance();
    mPeriodicIO.right_distance = Drive::getInstance().getRightEncoderDistance();
    mPeriodicIO.delta_left = mPeriodicIO.left_distance - mPeriodicIO.left_encoder_prev_distance_;
    mPeriodicIO.delta_right = mPeriodicIO.right_distance - mPeriodicIO.right_encoder_prev_distance_;    
    mPeriodicIO.gyro_angle = Drive::getInstance().getHeading();
    mPeriodicIO.odometry_velocity = RobotState::getInstance().generateOdometryFromSensors(
            mPeriodicIO.delta_left, mPeriodicIO.delta_right, mPeriodicIO.gyro_angle);
    mPeriodicIO.predicted_velocity = ck::math::Kinematics::forwardKinematics(Drive::getInstance().getLeftLinearVelocity(),
            Drive::getInstance().getRightLinearVelocity());
    RobotState::getInstance().addObservations(timestamp, mPeriodicIO.odometry_velocity,
            mPeriodicIO.predicted_velocity);
    mPeriodicIO.left_encoder_prev_distance_ = mPeriodicIO.left_distance;
    mPeriodicIO.right_encoder_prev_distance_ = mPeriodicIO.right_distance;

    //mPeriodicIO.robot_state_loop_time = loopTimer.hasElapsed();
}

std::string RobotStateEstimator::getName() {
    return "RobotStateEstimator";
}

std::vector<std::string> RobotStateEstimator::generateReport() {

    ck::geometry::Pose2d odometry = RobotState::getInstance().getLatestFieldToVehicle();

    mObjList.clear();

    mObjList.push_back("RobotPoseX");
    mObjList.push_back(std::to_string(odometry.getTranslation().x()));

    mObjList.push_back("RobotPoseY");
    mObjList.push_back(std::to_string(odometry.getTranslation().y()));

    mObjList.push_back("RobotPoseTheta");
    mObjList.push_back(std::to_string(odometry.getRotation().getDegrees()));

    mObjList.push_back("RobotLinearVelocity");
    mObjList.push_back(std::to_string(RobotState::getInstance().getMeasuredVelocity().dx));

    mObjList.push_back("RobotStateEstimatorLoopTime");
    mObjList.push_back(std::to_string(mPeriodicIO.robot_state_loop_time));

    return mObjList;
}
