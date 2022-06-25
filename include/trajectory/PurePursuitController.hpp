#pragma once

#include <vector>
#include "geometry/Geometry.hpp"
#include "TrajectoryIterator.hpp"
#include "IPathFollower.hpp"
#include "utils/CKMath.hpp"
#include "Arc.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S>
        class PurePursuitController : public IPathFollower
        {
            static_assert(std::is_base_of<ck::geometry::ITranslation2d<S>, S>::value, "S must inherit from ITranslation2d<S>");

        protected:
            TrajectoryIterator<S> iterator_;
            double sampling_dist_;
            double lookahead_;
            double goal_tolerance_;
            bool done_ = false;

        public:
            PurePursuitController(const DistanceView<S> &path, double sampling_dist, double lookahead, double goal_tolerance)
                : iterator_(path), sampling_dist_(sampling_dist), lookahead_(lookahead), goal_tolerance_(goal_tolerance) {}

            ck::geometry::Twist2d steer(const ck::geometry::Pose2d &current_pose)
            {
                done_ = done_ || (iterator_.isDone() && current_pose.getTranslation().distance(iterator_.getState().getTranslation()) <= goal_tolerance_);
                if (done_)
                {
                    return Twist2d(0.0, 0.0, 0.0);
                }

                double remaining_progress = iterator_.getRemainingProgress();
                double goal_progress = 0.0;
                // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
                for (double progress = 0.0; progress <= remaining_progress; progress = ck::math::min(remaining_progress, progress + sampling_dist_))
                {
                    double dist = current_pose.getTranslation().distance(iterator_.preview(progress).state().getTranslation());
                    if (dist > lookahead_)
                    {
                        if (goal_progress == 0.0 && !iterator_.isDone())
                        {
                            // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
                            // lookahead.
                            goal_progress = progress;
                        }
                        break;
                    }
                    goal_progress = progress;
                    if (progress == remaining_progress)
                    {
                        break;
                    }
                }
                iterator_.advance(goal_progress);
                Arc<S> arc(current_pose, iterator_.getState());
                if (arc.length < Util.kEpsilon)
                {
                    return Twist2d(0.0, 0.0, 0.0);
                }
                else
                {
                    return Twist2d(arc.length, 0.0, arc.length / arc.radius);
                }
            }

            bool isDone() { return done_; }

            static double getDirection(Pose2d pose, S point)
            {
                ck::geometry::Translation2d poseToPoint(pose.getTranslation(), point.getTranslation());
                ck::geometry::Translation2d robot = pose.getRotation().toTranslation();
                double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
                return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
            }
        };
    } // namespace trajectory
} // namespace ck