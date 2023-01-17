#include "trajectory_generator_node.hpp"
#include "trajectory_generator_node/GetTrajectory.h"
#include "trajectory_generator_node/OutputTrajectory.h"

#include "ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
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

using namespace trajectory_generator_node;

ros::NodeHandle *node;

// std::map<std::string, trajectory_generator_node::OutputTrajectory> traj_map;
std::map<std::string, Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>> traj_map;

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

    ck::planners::DriveMotionPlanner motion_planner;

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
        trajectory_generator_node::OutputTrajectory output_trajectory = package_trajectory(trajectory_json["name"], generated_trajectory);
        (void)output_trajectory;
        traj_map.insert({trajectory_json["name"], generated_trajectory});
    }
}

bool get_trajectory(trajectory_generator_node::GetTrajectory::Request &request, trajectory_generator_node::GetTrajectory::Response &response)
{
    // ck::log_info << "Getting Trajectory: " << request.path_name << std::flush;

    // try
    // {
    //     response.trajectory = traj_map.at(request.path_name);
    // }
    // catch(const std::out_of_range& exception)
    // {
    //     ck::log_error << exception.what() << std::flush;
    //     return false;
    // }
    (void)request;
    (void)response;
    
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

    ros::ServiceServer service_generate = node->advertiseService("get_trajectory", get_trajectory);

    generate_trajectories();

    // Send traj updates on /SwerveAutoControl
    // send Swerve_Drivetrain_Auto_Control

    ros::spin();
    return 0;
}