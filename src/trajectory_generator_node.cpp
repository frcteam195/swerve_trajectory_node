#include "trajectory_generator_node.hpp"

#include "ck_utilities/Logger.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "geometry/Pose2dWithCurvature.hpp"
#include "plannners/DriveMotionPlanner.hpp"
#include "trajectory/timing/TimingConstraint.hpp"
#include "trajectory/Trajectory.hpp"
#include "utils/JsonParser.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_generator_node/GetTrajectory.h"

#include "boost/filesystem.hpp"
#include "boost/filesystem/fstream.hpp"
#include "nlohmann/json.hpp"

#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace fs = boost::filesystem;

ros::NodeHandle *node;

std::map<std::string, bool> trajectory_map;

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

        std::vector<ck::geometry::Pose2d> waypoints = ck::json::parse_json_waypoints(trajectory_json["waypoints"]);

        std::vector<ck::trajectory::timing::TimingConstraint<ck::geometry::Pose2dWithCurvature>> constraints;

        ck::trajectory::Trajectory<ck::trajectory::timing::TimedState<ck::geometry::Pose2dWithCurvature>> output_trajectory;
        output_trajectory = motion_planner.generateTrajectory(trajectory_json["reversed"],
                                                              waypoints,
                                                              constraints,
                                                              max_acceleration,
                                                              max_velocity,
                                                              max_voltage);

        trajectory_map.insert(std::pair<std::string, bool>(trajectory_json["name"], true));
    }
}

bool get_trajectory(trajectory_generator_node::GetTrajectory::Request &request, trajectory_generator_node::GetTrajectory::Response &response)
{
    ck::log_info << "Getting trajectory: " << request.path_name << std::flush;

    (void)response;
    return false;
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

    ros::spin();
    return 0;
}