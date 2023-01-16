#pragma once

#include "ck_utilities/team254_geometry/Pose2d.hpp"
#include "nlohmann/json.hpp"

#include <vector>
#include <utility>

namespace ck
{
    namespace json
    {
        std::pair<std::vector<team254_geometry::Pose2d>, std::vector<team254_geometry::Rotation2d>> parse_json_waypoints(nlohmann::json json_waypoints);
    } // namespace json

} // namespace ck
