#include "trajectory_generator_node.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_generator_node/Generate.h"
#include <ck_utilities/ParameterHelper.hpp>

#include <mutex>
#include <string>
#include <thread>

ros::NodeHandle* node;

bool generate_trajectories(trajectory_generator_node::Generate::Request &request, trajectory_generator_node::Generate::Response &response)
{
    (void)request;
    ROS_INFO("Generating trajectories from: %s", request.paths_directory.c_str());
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
	required_params_found &= n.getParam(CKSP(trajectory_config_dir), trajectory_config_dir);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

	ros::ServiceServer service_generate = node->advertiseService("generate_trajectories", generate_trajectories);

    ros::spin();
    return 0;
}