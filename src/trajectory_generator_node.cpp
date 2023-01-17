#include "trajectory_generator_node.hpp"
#include "trajectory_generator_node/StartTrajectory.h"
#include "trajectory_generator_node/OutputTrajectory.h"

#include "ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/TimedView.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
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

namespace fs = boost::filesystem;

using namespace ck::team254_geometry;
using namespace ck::trajectory;
using namespace ck::trajectory::timing;
using namespace ck::planners;

using namespace trajectory_generator_node;

ros::NodeHandle *node;

// std::map<std::string, trajectory_generator_node::OutputTrajectory> traj_map;
std::map<std::string, Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>> traj_map;

DriveMotionPlanner *motion_planner = new DriveMotionPlanner();

bool traj_running = false;
Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> *current_trajectory = nullptr;
TimedView<Pose2dWithCurvature, Rotation2d> *timed_view = new TimedView(*current_trajectory);
Pose2d *current_pose = new Pose2d();
double current_timestamp = 0.0;

OutputTrajectory package_trajectory(std::string name, Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
{
    OutputTrajectory output = OutputTrajectory();

    output.name = name;

    for (int i = 0; i < trajectory.length(); ++i)
    {
        geometry_msgs::Pose waypoint = geometry_msgs::Pose();
        waypoint.position.x = trajectory.getState(i).state().getTranslation().x();
        waypoint.position.y = trajectory.getState(i).state().getTranslation().y();
        waypoint.position.z = 0.0;
        
        std::cout << waypoint.position.x << "," << waypoint.position.y << std::endl;

        tf2::Quaternion track;
        track.setRPY(0.0, 0.0, trajectory.getState(i).state().getRotation().getRadians());
        track.normalize();
        waypoint.orientation = tf2::toMsg(track);

        geometry_msgs::Pose heading = geometry_msgs::Pose();
        heading.position.x = 0.0;
        heading.position.y = 0.0;
        heading.position.z = 0.0;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, trajectory.getHeading(i).state().getRadians());
        orientation.normalize();
        heading.orientation = tf2::toMsg(orientation);

        output.waypoints.push_back(waypoint);
        output.headings.push_back(heading);
    }

    return output;
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
        generated_trajectory = motion_planner->generateTrajectory(trajectory_json["reversed"],
                                                                 path_points.first,
                                                                 path_points.second,
                                                                 max_velocity,
                                                                 max_acceleration,
                                                                 max_voltage);

        // Convert the CK trajectory into a ROS path.
        trajectory_generator_node::OutputTrajectory output_trajectory = package_trajectory(trajectory_json["name"], generated_trajectory);
        (void)output_trajectory;
        traj_map.insert({trajectory_json["name"], generated_trajectory});
    }
}

void robot_odometry_subscriber(const nav_msgs::Odometry &odom)
{
    // geometry_msgs::Pose drivetrain_pose = odom.pose.pose;
    // double x = ck::math::meters_to_inches(drivetrain_pose.position.x);
    // double y = ck::math::meters_to_inches(drivetrain_pose.position.y);

    geometry::Pose drivetrain_pose = geometry::to_pose(odom.pose.pose);
    drivetrain_pose.orientation.x();
    // (void)drivetrain_pose.position.

    // tf2::Quaternion heading(
    //     drivetrain_pose.orientation.x,
    //     drivetrain_pose.orientation.y,
    //     drivetrain_pose.orientation.z,
    //     drivetrain_pose.orientation.w);
    // tf2::Matrix3x3 headingM(heading);
    // double roll, pitch, yaw;
    // headingM.getRPY(roll, pitch, yaw);

    // *current_pose = Pose2d(x, y, Rotation2d::fromRadians(yaw));
	// geometry::Twist drivetrain_twist = geometry::to_twist(odom.twist.twist);
	// drivetrain_diagnostics.field_actual_x_translation_m_s = drivetrain_twist.linear.x();
	// drivetrain_diagnostics.field_actual_y_translation_m_s = drivetrain_twist.linear.y();
	// drivetrain_diagnostics.actual_angular_speed_deg_s = ck::math::rad2deg(drivetrain_twist.angular.yaw());
	// drivetrain_diagnostics.actual_total_speed_m_s = drivetrain_twist.linear.norm();

	// robot_transform.angular = geometry::to_rotation(odom.pose.pose.orientation);
	// robot_transform.linear = geometry::to_translation(odom.pose.pose.position);
}

bool start_trajectory(trajectory_generator_node::StartTrajectory::Request &request, trajectory_generator_node::StartTrajectory::Response &response)
{
    ck::log_info << "Request to start trajectory: " << request.trajectory_name << std::flush;

    if (traj_running) return false;

    try
    {
        current_trajectory = &traj_map.at(request.trajectory_name);
        motion_planner->reset();
        *timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(*current_trajectory);
        TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(timed_view);
        motion_planner->setTrajectory(traj_it);
        traj_running = true;
        response.accepted = true;
    }
    catch (const std::out_of_range& exception)
    {
        ck::log_error << exception.what() << std::flush;
        return false;
    }

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

    // ros::ServiceServer service_generate = node->advertiseService("get_trajectory", get_trajectory);
    ros::ServiceServer service_start = node->advertiseService("start_trajectory", start_trajectory);
	ros::Subscriber odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());
    ros::Publisher swerve_auto_control_publisher = node->advertise<ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control>("/SwerveAutoControl", 10);

    generate_trajectories();

    // Send traj updates on /SwerveAutoControl
    // send Swerve_Drivetrain_Auto_Control

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        if (traj_running)
        {
            current_timestamp = ros::Time::now().toSec();

            ChassisSpeeds output = motion_planner->update(current_timestamp, *current_pose);

            Pose2d robot_pose_vel(output.vxMetersPerSecond * 0.01, output.vyMetersPerSecond * 0.01, Rotation2d::fromRadians(output.omegaRadiansPerSecond * 0.01));
            Twist2d twist_vel = Pose2d::log(robot_pose_vel);
            ChassisSpeeds updated_output(twist_vel.dx / 0.01, twist_vel.dy / 0.01, twist_vel.dtheta / 0.01);
            
            ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control swerve_auto_control;
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
            heading.setRPY(0.0, 0.0, motion_planner->getHeadingSetpoint().state().getRadians());
            heading.normalize();
            swerve_auto_control.pose.orientation = tf2::toMsg(heading);

            swerve_auto_control_publisher.publish(swerve_auto_control);

            if (motion_planner->isDone())
            {
                traj_running = false;
            }

        //            Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
        //         mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
        //         Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
        // Twist2d twist_vel = Pose2d.log(robot_pose_vel);
        // ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
        //         twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);
 
 
        }

        rate.sleep();
    }

    return 0;
}