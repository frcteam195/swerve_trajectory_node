#include "swerve_trajectory_node.hpp"
#include "swerve_trajectory_node/StartTrajectory.h"
#include "swerve_trajectory_node/StopTrajectory.h"
#include "swerve_trajectory_node/OutputTrajectory.h"
#include "swerve_trajectory_node/GetAutonomousInfo.h"
#include "swerve_trajectory_node/ResetPoseWithConfirmation.h"

#include "get_odom_msg.hpp"

#include "ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h"
#include "ck_ros_msgs_node/Trajectory_Status.h"

#include "frc_robot_utilities/frc_robot_utilities.hpp"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/team254_swerve/SwerveDriveKinematics.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/TimedView.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"

#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "boost/filesystem.hpp"
#include "boost/filesystem/fstream.hpp"
#include "nlohmann/json.hpp"

#include "parsers/JsonParser.hpp"
#include "trajectory_utils.hpp"

#include <map>
#include <string>
#include <utility>
#include <atomic>

namespace fs = boost::filesystem;

using std::vector;
using std::pair;

using namespace ck::team254_geometry;
using namespace ck::trajectory;
using namespace ck::trajectory::timing;
using namespace ck::planners;

using namespace swerve_trajectory_node;

ros::NodeHandle *node;

static ros::Publisher *path_publisher;
static ros::Publisher *reset_pose_publisher;
static ros::Publisher *status_publisher;

// std::map<std::string, swerve_trajectory_node::OutputTrajectory> traj_map;
// std::map<std::string, std::pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path> traj_map;
// std::map<std::string, pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path>> traj_map;
std::map<std::string, vector<pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path>>> traj_map;

DriveMotionPlanner *motion_planner;

std::atomic_bool traj_running{false};
Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> current_trajectory;
TimedView<Pose2dWithCurvature, Rotation2d> timed_view;
Pose2d current_pose;
double current_timestamp = 0.0;
double persistHeadingRads = 0.0;
std::string active_trajectory_name = "";

ck_ros_msgs_node::Trajectory_Status trajectory_status;

nav_msgs::Path package_trajectory(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
{
    ros::Time stamp = ros::Time::now();

    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "odom";

    for (int i = 0; i < trajectory.length(); i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = stamp;
        pose_stamped.header.frame_id = "odom";

        geometry::Pose pose;
        pose.position.x(ck::math::inches_to_meters(trajectory.getState(i).state().getTranslation().x()));
        pose.position.y(ck::math::inches_to_meters(trajectory.getState(i).state().getTranslation().y()));
        pose.orientation.yaw(trajectory.getHeading(i).state().getRadians());

        // std::cout << std::setw(5) << std::left << pose.position.x();
        // std::cout << std::setw(5) << std::left << pose.position.y();
        // std::cout << std::setw(5) << std::left << pose.orientation.yaw();
        // std::cout << trajectory.getHeading(i).state().getRotation().getDegrees() << std::endl;


        pose_stamped.pose = geometry::to_msg(pose);
        path.poses.push_back(pose_stamped);
    }

    // std::cout << "\n\n\n";

    timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(trajectory);
    TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(&timed_view);

    return path;
}

void generate_trajectories(void)
{
    ck::log_info << "Generating all trajectories defined in: " << trajectory_directory << std::flush;

    fs::path directory_path(trajectory_directory);

    if (!fs::exists(directory_path))
    {
        ck::log_error << directory_path << " does not exist!" << std::flush;
        return;
    }

    for (const fs::directory_entry &trajectory_configuration : fs::directory_iterator(trajectory_directory))
    {
        ck::log_info << "Generating trajectory from: " << trajectory_configuration.path() << std::flush;

        fs::ifstream trajectory_buffer{trajectory_configuration.path()};
        nlohmann::json trajectory_json = nlohmann::json::parse(trajectory_buffer);

        std::string auto_name = trajectory_json["name"];

        if (!trajectory_json.contains("paths"))
        {
            ck::log_warn << "Invalid trajectory - skipping." << std::endl;
            continue;
        }

        vector<pair<vector<Pose2d>, vector<Rotation2d>>> paths = ck::json::parse_json_paths(trajectory_json["paths"]);

        vector<pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path>> traj_paths;

        for (auto path : paths)
        {
            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generated_trajectory;
            generated_trajectory = motion_planner->generateTrajectory(false,
                                                                      path.first,
                                                                      path.second,
                                                                      robot_max_fwd_vel,
                                                                      robot_max_fwd_accel,
                                                                      max_voltage);

            nav_msgs::Path output_path = package_trajectory(generated_trajectory);

            traj_paths.push_back(std::make_pair(generated_trajectory, output_path));
        }

        traj_map.insert({auto_name, traj_paths});
    }
}

void robot_odometry_subscriber(const nav_msgs::Odometry &odom)
{
    geometry::Pose drivetrain_pose = geometry::to_pose(odom.pose.pose);

    double x = ck::math::meters_to_inches(drivetrain_pose.position.x());
    double y = ck::math::meters_to_inches(drivetrain_pose.position.y());
    double heading = drivetrain_pose.orientation.yaw();

    current_pose = Pose2d(x, y, Rotation2d::fromRadians(heading));
}

bool get_autonomous_info(swerve_trajectory_node::GetAutonomousInfo::Request &request, swerve_trajectory_node::GetAutonomousInfo::Response &response)
{
    ck::log_info << "Autonomous Info Requested for: " << request.autonomous_name << std::endl;

    try
    {
        auto traj_list = traj_map.at(request.autonomous_name);
        response.number_of_trajectories = (int)traj_list.size();

        auto traj = traj_list.at(0).first;

        response.x_inches = traj.getFirstState().state().getTranslation().x();
        response.y_inches = traj.getFirstState().state().getTranslation().y();
        response.heading_degrees = traj.getFirstHeading().state().getDegrees();
    }
    catch(const std::out_of_range& e)
    {
        ck::log_error << e.what() << std::endl;
        std::cerr << e.what() << '\n';
        response.valid = false;
        response.number_of_trajectories = -1;
        response.x_inches = -1;
        response.y_inches = -1;
        response.heading_degrees = -1;

        return false;
    }

    response.valid = true;
    return true;
}

bool reset_pose_confirmation_service(swerve_trajectory_node::ResetPoseWithConfirmation::Request &request, swerve_trajectory_node::ResetPoseWithConfirmation::Response &response)
{
    static std::atomic_bool reset_pose_service_running = false;
    if (!reset_pose_service_running)
    {
        try
        {
            reset_pose_service_running = true;
            bool success = reset_robot_pose(robot_status.get_alliance(), request.x_inches, request.y_inches, request.heading_degrees);

            if (!success)
            {
                reset_pose_service_running = false;
                return false;
            }

            ros::Time start_time = ros::Time::now();
            double timeout_s = 0.25;
            double elapsed_time_s;
            double heading_rad = ck::math::deg2rad(request.heading_degrees);
            if (robot_status.get_alliance() == Alliance::BLUE)
            {
                heading_rad += M_PI;
            }
            bool reset_still_running = true;
            Pose2d requested_pose(request.x_inches, request.y_inches, Rotation2d::fromRadians(heading_rad));
            do {
                ros::spinOnce();
                if (current_pose.epsilonAllEquals(requested_pose, 1))
                {
                    reset_still_running = false;
                }
                elapsed_time_s = (ros::Time::now() - start_time).toSec();
            } while(reset_still_running && elapsed_time_s < timeout_s);

            if (elapsed_time_s > timeout_s)
            {
                ROS_ERROR("Timeout occurred during pose reset!");
                response.completed = false;
            }
            else
            {
                response.completed = true;
            }
        }
        catch (const std::exception& e)
        {
            ck::log_error << e.what() << std::endl;
            response.completed = false;
            reset_pose_service_running = false;
            return false;
        }
        reset_pose_service_running = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool stop_trajectory(swerve_trajectory_node::StopTrajectory::Request &request, swerve_trajectory_node::StopTrajectory::Response &response)
{
    (void)request;
    try
    {
        traj_running = false;
        motion_planner->reset();

        //TODO: Not sure what we should do on force stop or if we should publish this
        trajectory_status.trajectory_name = "";
        trajectory_status.is_running = false;
        trajectory_status.is_completed = false;
        trajectory_status.trajectory_index = 0;
        trajectory_status.progress = 0.0;

        status_publisher->publish(trajectory_status);

        response.accepted = true;
    }
    catch (const std::exception& e)
    {
        ck::log_error << e.what() << std::endl;
        response.accepted = false;
        return false;
    }
    return true;
}

bool start_trajectory(swerve_trajectory_node::StartTrajectory::Request &request, swerve_trajectory_node::StartTrajectory::Response &response)
{
    ck::log_info << "Start trajectory requested!" << std::endl;
    ck::log_info << "Request to start trajectory: " << request.autonomous_name << std::endl;

    if (traj_running) return false;

    try
    {
        current_trajectory = traj_map.at(request.autonomous_name).at(request.trajectory_index).first;
        timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(current_trajectory);
        TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(&timed_view);
        motion_planner->reset();
        motion_planner->setTrajectory(traj_it);
        traj_running = true;
        response.accepted = true;

        trajectory_status.trajectory_name = request.autonomous_name;
        trajectory_status.is_running = true;
        trajectory_status.is_completed = false;
        trajectory_status.trajectory_index = request.trajectory_index;
        trajectory_status.progress = 0.0;

        status_publisher->publish(trajectory_status);

        // double start_x = current_trajectory.getFirstState().state().getTranslation().x();
        // double start_y = current_trajectory.getFirstState().state().getTranslation().y();
        // double start_heading = current_trajectory.getFirstHeading().state().getDegrees();

        // reset_pose_publisher->publish(get_odom_msg(start_x, start_y, start_heading));

        path_publisher->publish(traj_map.at(request.autonomous_name).at(request.trajectory_index).second);
        active_trajectory_name = request.autonomous_name;
    }
    catch (const std::out_of_range &exception)
    {
        ck::log_info << "it bad" << std::flush;
        ck::log_error << exception.what() << std::flush;
        response.accepted = false;
        return false;
    }

    ck::log_info << "Accepted trajectory" << std::flush;
    return true;
}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "swerve_trajectory_node");

    ros::NodeHandle n;

    node = &n;

    // std::vector<Translation2d> translations = {
    //     Translation2d(-1, 1),
    //     Translation2d(1, 1),
    //     Translation2d(-1, -1),
    //     Translation2d(1, -1)};

    // ck::team254_swerve::SwerveDriveKinematics kinematics(translations);

    register_for_robot_updates(node);

    bool required_params_found = true;
    required_params_found &= n.getParam(CKSP(robot_max_fwd_accel), robot_max_fwd_accel);
    required_params_found &= n.getParam(CKSP(robot_max_fwd_vel), robot_max_fwd_vel);
    required_params_found &= n.getParam(CKSP(max_voltage), max_voltage);
    required_params_found &= n.getParam(CKSP(trajectory_directory), trajectory_directory);
    required_params_found &= n.getParam(CKSP(robot_max_fwd_vel_mult), robot_max_fwd_vel_mult);

    robot_max_fwd_vel *= robot_max_fwd_vel_mult;

    if (!required_params_found)
    {
        ck::log_error << "Missing required parameters for node " << ros::this_node::getName() << "." << std::flush;
        ck::log_error << "Please check the list and make sure all required parameters are included." << std::flush;
        return 1;
    }

    motion_planner = new DriveMotionPlanner(robot_max_fwd_vel);

    robot_max_fwd_accel = ck::math::meters_to_inches(robot_max_fwd_accel);
    robot_max_fwd_vel = ck::math::meters_to_inches(robot_max_fwd_vel);
    // std::cout << "pre sub" << std::endl;

    // ros::ServiceServer service_generate = node->advertiseService("get_trajectory", get_trajectory);
    static ros::ServiceServer service_start = node->advertiseService("start_trajectory", start_trajectory);
    static ros::ServiceServer service_get_autonomous_info = node->advertiseService("get_autonomous_info", get_autonomous_info);
    static ros::ServiceServer service_stop = node->advertiseService("stop_trajectory", start_trajectory);
    static ros::ServiceServer reset_pose_confirmation = node->advertiseService("reset_pose_with_confirmation", reset_pose_confirmation_service);
	static ros::Subscriber odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());
    static ros::Publisher swerve_auto_control_publisher = node->advertise<ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control>("/SwerveAutoControl", 10);
    static ros::Publisher path_publisher_ = node->advertise<nav_msgs::Path>("/CurrentPath", 10, true);
    path_publisher = &path_publisher_;
    static ros::Publisher reset_pose_publisher_ = node->advertise<nav_msgs::Odometry>("/ResetHeading", 10);
    reset_pose_publisher = &reset_pose_publisher_;
    static ros::Publisher status_publisher_ = node->advertise<ck_ros_msgs_node::Trajectory_Status>("/TrajectoryStatus", 10);
    status_publisher = &status_publisher_;

    generate_trajectories();

    // debug_trajectory(traj_map.at("correct_start").first);
    // debug_trajectory_iterator(traj_map.at("correct_start").first, 0.01);

    // Send traj updates on /SwerveAutoControl
    // send Swerve_Drivetrain_Auto_Control

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        if (robot_status.get_mode() != RobotMode::AUTONOMOUS && traj_running)
        {
            traj_running = false;
            motion_planner->reset();
        }

        ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control swerve_auto_control;

        trajectory_status.is_running = traj_running;

        if (traj_running)
        {
            trajectory_status.trajectory_name = active_trajectory_name;
            static double traj_start_time = ros::Time::now().toSec();
            if (motion_planner->isDone())
            {
                trajectory_status.is_completed = true;
                std::cout << "TRAJ TIME = " << ros::Time::now().toSec() - traj_start_time << std::endl;
                traj_running = false;
                persistHeadingRads = motion_planner->getHeadingSetpoint().state().getRadians();
                continue;
            }

            current_timestamp = ros::Time::now().toSec();

            ChassisSpeeds output = motion_planner->update(current_timestamp, current_pose);

            Pose2d robot_pose_vel(output.vxMetersPerSecond * 0.01, output.vyMetersPerSecond * 0.01, Rotation2d::fromRadians(output.omegaRadiansPerSecond * 0.01));
            Twist2d twist_vel = Pose2d::log(robot_pose_vel);
            ChassisSpeeds updated_output(twist_vel.dx / 0.01, twist_vel.dy / 0.01, twist_vel.dtheta / 0.01);

            // std::cout << "Twist: " << updated_output.omegaRadiansPerSecond << std::endl;

            swerve_auto_control.twist.linear.x = output.vxMetersPerSecond;//updated_output.vxMetersPerSecond;
            swerve_auto_control.twist.linear.y = output.vyMetersPerSecond;//updated_output.vyMetersPerSecond;
            swerve_auto_control.twist.linear.z = 0.0;

            swerve_auto_control.twist.angular.x = 0.0;
            swerve_auto_control.twist.angular.y = 0.0;
            swerve_auto_control.twist.angular.z = updated_output.omegaRadiansPerSecond;

            swerve_auto_control.pose.position.x = 0.0;
            swerve_auto_control.pose.position.y = 0.0;
            swerve_auto_control.pose.position.z = 0.0;

            tf2::Quaternion heading;
            heading.setRPY(0.0, 0.0, motion_planner->getHeadingSetpoint().state().getRadians());
            heading.normalize();
            swerve_auto_control.pose.orientation = tf2::toMsg(heading);

            trajectory_status.progress = motion_planner->getCurrentProgress();
        }
        else
        {
            geometry::Twist blank_twist;
            geometry::Pose blank_pose;
            blank_pose.orientation.yaw(persistHeadingRads);

            // blank_pose.orientation.yaw(ck::math::PI / 2.0);
            swerve_auto_control.twist = geometry::to_msg(blank_twist);
            swerve_auto_control.pose = geometry::to_msg(blank_pose);
        }

        swerve_auto_control_publisher.publish(swerve_auto_control);
        status_publisher->publish(trajectory_status);
        rate.sleep();
    }

    return 0;
}
