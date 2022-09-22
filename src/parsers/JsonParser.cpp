#include "parsers/JsonParser.hpp"

namespace ck
{
    namespace json
    {

        std::vector<team254_geometry::Pose2d> parse_json_waypoints(nlohmann::json json_waypoints)
        {
            std::vector<team254_geometry::Pose2d> waypoints;

            for (auto json_waypoint : json_waypoints)
            {
                team254_geometry::Rotation2d theta = team254_geometry::Rotation2d::fromDegrees(json_waypoint["theta"]);
                team254_geometry::Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], theta);
                waypoints.push_back(waypoint);
            }

            return waypoints;
        }

    } // namespace json

} // namespace ck
