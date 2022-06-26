#include "trajectory_generator_node.hpp"

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

#include <mutex>
#include <string>
#include <thread>

namespace fs = boost::filesystem;

ros::NodeHandle *node;

void generate_trajectories(void)
{
    ROS_INFO("Generating all trajectories defined in: %s", trajectory_directory.c_str());

    ck::planners::DriveMotionPlanner motion_planner;

    for (const fs::directory_entry &trajectory_configuration : fs::directory_iterator(trajectory_directory))
    {
        ROS_INFO("Generating trajectory from: %s", trajectory_configuration.path().c_str());

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
    }
}

bool get_trajectory(trajectory_generator_node::GetTrajectory::Request &request, trajectory_generator_node::GetTrajectory::Response &response)
{
    ROS_INFO("Getting trajectory: %s", request.path_name.c_str());

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
        ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
        return 1;
    }

    ros::ServiceServer service_generate = node->advertiseService("get_trajectory", get_trajectory);

    ros::spin();
    return 0;
}