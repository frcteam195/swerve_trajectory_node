#include "parsers/JsonParser.hpp"

using std::vector;
using std::pair;
using ck::team254_geometry::Pose2d;
using ck::team254_geometry::Rotation2d;

namespace ck
{
    namespace json
    {
        pair<vector<Pose2d>, vector<Rotation2d>> parse_json_waypoints(nlohmann::json json_waypoints)
        {
            pair<vector<Pose2d>, vector<Rotation2d>> output;

            for (auto json_waypoint : json_waypoints)
            {
                Rotation2d track = Rotation2d::fromDegrees(json_waypoint["theta"]);
                Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], track);
                Rotation2d heading = Rotation2d::fromDegrees(json_waypoint["heading"]);
                output.first.push_back(waypoint);
                output.second.push_back(heading);
            }

            return output;
        }

        vector<pair<vector<Pose2d>, vector<Rotation2d>>> parse_json_paths(nlohmann::json json_paths)
        {
            vector<pair<vector<Pose2d>, vector<Rotation2d>>> output_paths;

            for (auto json_path : json_paths)
            {
                output_paths.push_back(parse_json_waypoints(json_path["waypoints"]));
            }

            return output_paths;
        }
    } // namespace json

} // namespace ck
