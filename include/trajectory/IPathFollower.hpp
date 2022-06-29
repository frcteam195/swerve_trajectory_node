#pragma once

#include "ck_utilities/geometry/Pose2d.hpp"
#include "ck_utilities/geometry/Twist2d.hpp"

namespace ck {
    namespace trajectory {
        class IPathFollower {
        public:
            virtual ck::geometry::Twist2d steer(ck::geometry::Pose2d current_pose) = 0;
            virtual bool isDone() = 0;
        };
    }
}