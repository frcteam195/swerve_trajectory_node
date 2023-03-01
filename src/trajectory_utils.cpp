#include "trajectory_utils.hpp"

std::vector<PathStruct> mirror_paths(std::vector<PathStruct> paths)
{
    std::vector<PathStruct> mirror(paths.size());

    for (size_t i = 0; i < paths.size(); i++)
    {
        mirror.at(i).max_velocity_in_per_sec = paths.at(i).max_velocity_in_per_sec;

        for (size_t j = 0; j < paths.at(i).waypoints.size(); j++)
        {
            double x = paths.at(i).waypoints.at(j).getTranslation().x();
            double y = paths.at(i).waypoints.at(j).getTranslation().y();
            ck::team254_geometry::Rotation2d track = paths.at(i).waypoints.at(j).getRotation();
            ck::team254_geometry::Rotation2d heading = paths.at(i).headings.at(j);

            ck::team254_geometry::Rotation2d mirror_track(-track.cos(), track.sin(), true);
            ck::team254_geometry::Rotation2d mirror_heading(-heading.cos(), heading.sin(), true);


            double mirror_x = (FIELD_LENGTH_INCHES/2.0 - x) * 2.0 + x;
            // double mirror_y = (-FIELD_WIDTH_INCHES/2.0 - y) * 2.0 + y;
            // output_pose.position.y = (mirror_line.lower_right_y - input_pose.position.y) * 2.0 + input_pose.position.y

            // mirror.at(i).waypoints.emplace_back(Translation2d(mirror_x, y), Rotation2d::fromRadians(ck::math::normalize_to_2_pi(track + ck::math::PI)));             
            mirror.at(i).waypoints.emplace_back(Translation2d(mirror_x, y), mirror_track);             
            // mirror.at(i).headings.push_back(Rotation2d::fromRadians(ck::math::normalize_to_2_pi(heading + ck::math::PI)));
            mirror.at(i).headings.push_back(mirror_heading);
        }
    }

    return mirror;
}

void debug_trajectory(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
{
    std::cout << "---------------TRAJECTORY--------------" << std::endl;
    std::cout << "X (in.),Y (in.),Heading (deg.)" << std::endl;

    for (int i = 0; i < trajectory.length(); i++)
    {
        double x = trajectory.getState(i).state().getTranslation().x();
        double y = trajectory.getState(i).state().getTranslation().y();
        double heading = trajectory.getHeading(i).state().getDegrees();        

        std::cout << x << "," << y << "," << heading << std::endl;
    }

    std::cout << "---------------------------------------" << std::endl;
}

void debug_trajectory_iterator(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory, double timestep)
{
    auto timed_view = TimedView<Pose2dWithCurvature, Rotation2d>(trajectory);
    TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> traj_it(&timed_view);

    const double totalProg = traj_it.getRemainingProgress();

    std::cout << "----------TRAJECTORY ITERATOR----------" << std::endl;
    std::cout << "X (in.),Y (in.),Heading (deg.), Velocity (in./s)" << std::endl;

    for (double i = 0.0; i < totalProg; i += timestep)
    {
        auto sample_point = traj_it.preview(i);
        double x = sample_point.state().state().getTranslation().x();
        double y = sample_point.state().state().getTranslation().y();
        double heading = sample_point.heading().state().getDegrees();
        double velocity = sample_point.state().velocity();

        std::cout << x << "," << y << "," << heading << "," << velocity << std::endl;
    }

    std::cout << "---------------------------------------" << std::endl;
}
