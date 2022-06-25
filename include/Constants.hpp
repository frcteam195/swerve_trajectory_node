#pragma once

#define K_LOOPER_DT 0.01_s
#define K_LOG_REPORT_RATE 0.02_s

// Wheel constants from CalConstants Java file. Should find new home.
#define K_DRIVE_WHEEL_TRACK_WIDTH_INCHES 23.5
#define K_DRIVE_WHEEL_DIAMETER_INCHES 5.0 * 0.9625
#define K_DRIVE_WHEEL_RADIUS_INCHES K_DRIVE_WHEEL_DIAMETER_INCHES / 2.0
#define K_TRACK_SCRUB_FACTOR 1.0 // Tune me!

// Tuned dynamics constants from CalConstants Java file.
#define K_ROBOT_LINEAR_INERTIA 68.946  // Kilograms
#define K_ROBOT_ANGULAR_INERTIA 125  // Kilograms * Meters^2
#define K_ROBOT_ANGULAR_DRAG 0.1  // Newtonmeters per Radians per Second
#define K_DRIVE_V_INTERCEPT 0.30165000 // 0.781046438 Angular Volts
#define K_DRIVE_KV 0.186163041  // Volts per Radians per Second
#define K_DRIVE_KA 0.0086739979  // Volts per Radians per Second^2

// LIDAR constants from...
#define K_LIDAR_X_OFFSET (-3.3211)
#define K_LIDAR_Y_OFFSET 0.0 
#define K_LIDAR_YAW_ANGLE_DEGREES 0.0
