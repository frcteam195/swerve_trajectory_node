#include "autonomous/modes/TestMode.hpp"
#include "autonomous/actions/DriveTrajectory.hpp"
#include "paths/TrajectoryGenerator.hpp"

void TestMode::routine() {
    //TODO: Revisit running statically declared trajectories or a way to ensure references are not destructed before/during execution
    DriveTrajectory driveAction(ck::paths::TrajectoryGenerator::getInstance().getTrajectorySet()->test90DegPath->get(), true);
    runAction(driveAction);
}