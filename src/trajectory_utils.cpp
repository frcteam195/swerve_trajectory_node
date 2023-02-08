#include "trajectory_utils.hpp"

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
    std::cout << "X (in.),Y (in.),Heading (deg.)" << std::endl;

    for (double i = 0.0; i < totalProg; i += timestep)
    {
        auto sample_point = traj_it.preview(i);
        double x = sample_point.state().state().getTranslation().x();
        double y = sample_point.state().state().getTranslation().y();
        double heading = sample_point.heading().state().getDegrees();

        std::cout << x << "," << y << "," << heading << std::endl;
    }

    std::cout << "---------------------------------------" << std::endl;
}
