#include "trajectory_generator_node.hpp"

#include "ck_utilities/ParameterHelper.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_generator_node/GetTrajectory.h"

#include <experimental/filesystem>
#include <mutex>
#include <string>
#include <thread>

namespace fs = std::experimental::filesystem;

ros::NodeHandle* node;

void generate_trajectories(void)
{
    ROS_INFO("Generating all trajectories defined in: %s", trajectory_directory.c_str());

    for (const fs::directory_entry &trajectory_configuration : fs::directory_iterator(trajectory_directory))
    {
        ROS_INFO("Generating trajectory from: %s", trajectory_configuration.path().c_str());
    }
}

bool get_trajectory(trajectory_generator_node::GetTrajectory::Request &request, trajectory_generator_node::GetTrajectory::Response &response)
{
    ROS_INFO("Getting trajectory: %s", request.path_name.c_str());

	response.success = false;
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