#include "parsers/JsonParser.hpp"

using std::vector;
using std::pair;
using ck::team254_geometry::Pose2d;
using ck::team254_geometry::Rotation2d;

namespace ck::json
{
    PathStruct parse_json_waypoints(nlohmann::json json_waypoints)
    {
        // pair<vector<Pose2d>, vector<Rotation2d>> output;
        PathStruct output;
        output.max_velocity_in_per_sec = -1;

        for (auto json_waypoint : json_waypoints)
        {
            Rotation2d track = Rotation2d::fromDegrees(json_waypoint["track"]);
            Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], track);
            Rotation2d heading = Rotation2d::fromDegrees(json_waypoint["heading"]);
            output.waypoints.push_back(waypoint);
            output.headings.push_back(heading);
        }

        return output;
    }

    vector<PathStruct> parse_json_paths(nlohmann::json json_paths)
    {
        vector<PathStruct> output_paths;

        for (auto json_path : json_paths)
        {
            PathStruct path = parse_json_waypoints(json_path["waypoints"]);

            if (json_path.contains("max_velocity"))
            {
                path.max_velocity_in_per_sec = json_path["max_velocity"];
            }

            output_paths.push_back(path);
        }

        return output_paths;
    }
} // namespace ck::json
