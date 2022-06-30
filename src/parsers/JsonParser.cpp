#include "parsers/JsonParser.hpp"

namespace ck
{
    namespace json
    {

        std::vector<geometry::Pose2d> parse_json_waypoints(nlohmann::json json_waypoints)
        {
            std::vector<geometry::Pose2d> waypoints;

            for (auto json_waypoint : json_waypoints)
            {
                geometry::Rotation2d theta = geometry::Rotation2d::fromDegrees(json_waypoint["theta"]);
                geometry::Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], theta);
                waypoints.push_back(waypoint);
            }

            return waypoints;
        }

    } // namespace json

} // namespace ck
