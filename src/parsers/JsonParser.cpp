#include "parsers/JsonParser.hpp"

using std::vector;
using std::pair;
using ck::team254_geometry::Pose2d;
using ck::team254_geometry::Rotation2d;

namespace ck::json
{
    PathSet parse_json_waypoints(nlohmann::json json_waypoints)
    {
        // pair<vector<Pose2d>, vector<Rotation2d>> output;
        PathSet output;
        output.max_velocity_in_per_sec = -1;

        for (auto json_waypoint : json_waypoints)
        {
            Rotation2d track = Rotation2d::fromDegrees(json_waypoint["track"]);
            Pose2d waypoint(json_waypoint["x"], json_waypoint["y"], track);
            Rotation2d heading = Rotation2d::fromDegrees(json_waypoint["heading"]);

            PathPoint red_point;
            red_point.waypoint = waypoint;
            red_point.heading = heading;

            PathPoint blue_point = mirror_point(red_point);

            if (json_waypoint.contains("offsets"))
            {
                auto json_offsets = json_waypoint["offsets"];

                if (json_offsets.contains("red"))
                {
                    double x_offset = json_offsets["red"]["x"];
                    double y_offset = json_offsets["red"]["y"];

                    double newX = red_point.waypoint.getTranslation().x() + x_offset;
                    double newY = red_point.waypoint.getTranslation().y() + y_offset;

                    red_point.waypoint = Pose2d(newX, newY, red_point.waypoint.getRotation());
                }

                if (json_offsets.contains("blue"))
                {
                    double x_offset = json_offsets["blue"]["x"];
                    double y_offset = json_offsets["blue"]["y"];

                    double newX = blue_point.waypoint.getTranslation().x() + x_offset;
                    double newY = blue_point.waypoint.getTranslation().y() + y_offset;

                    blue_point.waypoint = Pose2d(newX, newY, blue_point.waypoint.getRotation());
                }
            }

            output.red.waypoints.push_back(red_point.waypoint);
            output.red.headings.push_back(red_point.heading);

            output.blue.waypoints.push_back(blue_point.waypoint);
            output.blue.headings.push_back(blue_point.heading);
        }

        return output;
    }

    vector<PathSet> parse_json_paths(nlohmann::json json_paths)
    {
        vector<PathSet> output_paths;

        for (auto json_path : json_paths)
        {
            PathSet path = parse_json_waypoints(json_path["waypoints"]);

            if (json_path.contains("max_velocity"))
            {
                path.max_velocity_in_per_sec = json_path["max_velocity"];
            }

            output_paths.push_back(path);
        }

        return output_paths;
    }
} // namespace ck::json
