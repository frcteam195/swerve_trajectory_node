#include "trajectory_generator_node.hpp"
#include "trajectory_generator_node/StartTrajectory.h"
#include "trajectory_generator_node/OutputTrajectory.h"

#include "ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h"

#include "frc_robot_utilities/frc_robot_utilities.hpp"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
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

#include <map>
#include <string>
#include <utility>
#include <atomic>

namespace fs = boost::filesystem;

using namespace ck::team254_geometry;
using namespace ck::trajectory;
using namespace ck::trajectory::timing;
using namespace ck::planners;

using namespace trajectory_generator_node;

ros::NodeHandle *node;

static ros::Publisher *path_publisher;

// std::map<std::string, trajectory_generator_node::OutputTrajectory> traj_map;
// std::map<std::string, std::pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path> traj_map;
std::map<std::string, std::pair<Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>, nav_msgs::Path>> traj_map;

DriveMotionPlanner motion_planner;

std::atomic_bool traj_running{false};
Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> current_trajectory;
TimedView<Pose2dWithCurvature, Rotation2d> timed_view;
Pose2d current_pose;
double current_timestamp = 0.0;
// double persistHeadingRads = 0.0;
double persistHeadingRads = ck::math::deg2rad(45.0);

nav_msgs::Path package_trajectory(std::string name, Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
{
    (void)name;

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

    // timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(trajectory);
    // TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(&timed_view);

    // std::cout << "Traj len: " << traj_it.trajectory().length() << std::endl;
    // std::cout << traj_it.getRemainingProgress() << std::endl;

    // double totalProg = traj_it.getRemainingProgress();

    // for (double i = 0; i < totalProg; i += 0.1)
    // {
    //     int index = (int)std::floor(trajectory.length() * i / totalProg);
    //     // std::cout << traj_it.getHeading().state().getDegrees() << std::endl;
    //     // traj_it.advance(0.1);
    //     // std::cout << traj_it.preview(i).heading().state().getDegrees() << std::endl;
    //     std::cout << index << ": " << traj_it.trajectory().getHeading(index).state().getDegrees() << std::endl;
    // }

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

        std::pair<std::vector<Pose2d>, std::vector<Rotation2d>> path_points = ck::json::parse_json_waypoints(trajectory_json["waypoints"]);

        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generated_trajectory;
        generated_trajectory = motion_planner.generateTrajectory(trajectory_json["reversed"],
                                                                 path_points.first,
                                                                 path_points.second,
                                                                 max_velocity,
                                                                 max_acceleration,
                                                                 max_voltage);

        // Convert the CK trajectory into a ROS path.
        nav_msgs::Path output_path = package_trajectory(trajectory_json["name"], generated_trajectory);
        // (void)output_trajectory;
        traj_map.insert({trajectory_json["name"], std::make_pair(generated_trajectory, output_path)});
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

bool start_trajectory(trajectory_generator_node::StartTrajectory::Request &request, trajectory_generator_node::StartTrajectory::Response &response)
{
    ck::log_info << "Start trajectory requested!" << std::endl;
    ck::log_info << "Request to start trajectory: " << request.trajectory_name << std::endl;

    if (traj_running) return false;

    try
    {
        current_trajectory = traj_map.at(request.trajectory_name).first;
        timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(current_trajectory);
        TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(&timed_view);
        motion_planner.reset();
        motion_planner.setTrajectory(traj_it);
        traj_running = true;
        response.accepted = true;

        path_publisher->publish(traj_map.at(request.trajectory_name).second);
    }
    catch (const std::out_of_range& exception)
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
    ros::init(argc, argv, "trajectory_generator_node");

    ros::NodeHandle n;

    node = &n;

    register_for_robot_updates(node);

    bool required_params_found = true;
    required_params_found &= n.getParam(CKSP(max_acceleration), max_acceleration);
    required_params_found &= n.getParam(CKSP(max_velocity), max_velocity);
    required_params_found &= n.getParam(CKSP(max_voltage), max_voltage);
    required_params_found &= n.getParam(CKSP(trajectory_directory), trajectory_directory);

    if (!required_params_found)
    {
        ck::log_error << "Missing required parameters for node " << ros::this_node::getName() << "." << std::flush;
        ck::log_error << "Please check the list and make sure all required parameters are included." << std::flush;
        return 1;
    }

    // std::cout << "pre sub" << std::endl;

    // ros::ServiceServer service_generate = node->advertiseService("get_trajectory", get_trajectory);
    static ros::ServiceServer service_start = node->advertiseService("start_trajectory", start_trajectory);
	static ros::Subscriber odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());
    static ros::Publisher swerve_auto_control_publisher = node->advertise<ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control>("/SwerveAutoControl", 10);
    static ros::Publisher path_publisher_ = node->advertise<nav_msgs::Path>("/CurrentPath", 10, true);
    path_publisher = &path_publisher_;

    generate_trajectories();

    // Send traj updates on /SwerveAutoControl
    // send Swerve_Drivetrain_Auto_Control

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        if (robot_status.get_mode() != RobotMode::AUTONOMOUS && traj_running)
        {
            traj_running = false;
            motion_planner.reset();
        }

        ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control swerve_auto_control;

        if (traj_running)
        {
            if (motion_planner.isDone())
            {
                traj_running = false;
                persistHeadingRads = motion_planner.getHeadingSetpoint().state().getRadians();
                continue;
            }

            current_timestamp = ros::Time::now().toSec();

            ChassisSpeeds output = motion_planner.update(current_timestamp, current_pose);

            Pose2d robot_pose_vel(output.vxMetersPerSecond * 0.01, output.vyMetersPerSecond * 0.01, Rotation2d::fromRadians(output.omegaRadiansPerSecond * 0.01));
            Twist2d twist_vel = Pose2d::log(robot_pose_vel);
            ChassisSpeeds updated_output(twist_vel.dx / 0.01, twist_vel.dy / 0.01, twist_vel.dtheta / 0.01);
            
            swerve_auto_control.twist.linear.x = updated_output.vxMetersPerSecond;
            swerve_auto_control.twist.linear.y = updated_output.vyMetersPerSecond;
            swerve_auto_control.twist.linear.z = 0.0;

            swerve_auto_control.twist.angular.x = 0.0;
            swerve_auto_control.twist.angular.y = 0.0;
            swerve_auto_control.twist.angular.z = updated_output.omegaRadiansPerSecond;

            swerve_auto_control.pose.position.x = 0.0;
            swerve_auto_control.pose.position.y = 0.0;
            swerve_auto_control.pose.position.z = 0.0;
            
            tf2::Quaternion heading;
            heading.setRPY(0.0, 0.0, motion_planner.getHeadingSetpoint().state().getRadians());
            heading.normalize();
            swerve_auto_control.pose.orientation = tf2::toMsg(heading);
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

        rate.sleep();
    }

    return 0;
}
