#include "parsers/JsonParser.hpp"

namespace ck
{
    namespace json
    {
        std::pair<std::vector<team254_geometry::Pose2d>, std::vector<team254_geometry::Rotation2d>> parse_json_waypoints(nlohmann::json json_waypoints)
        {
            std::pair<std::vector<team254_geometry::Pose2d>, std::vector<team254_geometry::Rotation2d>> output;

            for (auto json_waypoint : json_waypoints)
            {
                team254_geometry::Rotation2d track = team254_geometry::Rotation2d::fromDegrees(json_waypoint["track"]);
                team254_geometry::Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], track);
                team254_geometry::Rotation2d heading = team254_geometry::Rotation2d::fromDegrees(json_waypoint["heading"]);
                output.first.push_back(waypoint);
                output.second.push_back(heading);
            }

            return output;
        }

    } // namespace json

} // namespace ck
