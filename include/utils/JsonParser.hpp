#pragma once

#include "ck_utilities/geometry/Pose2d.hpp"
#include "nlohmann/json.hpp"

#include <vector>

namespace ck
{
    namespace json
    {

        std::vector<geometry::Pose2d> parse_json_waypoints(nlohmann::json waypoints);

    } // namespace json

} // namespace ck
