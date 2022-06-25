#include <cmath>
#include <vector>
#include <iostream>
#include "geometry/Geometry.hpp"
#include "reporters/NetworkDataType.hpp"
#include "reporters/DataReporter.hpp"
#include "reporters/NetworkDataReporter.hpp"
#include "utils/CKMath.hpp"
#include "utils/Singleton.hpp"
#include "utils/Subsystem.hpp"
#include "utils/Looper/ILooper.hpp"

class RobotStateEstimator : public Subsystem, public Singleton<RobotStateEstimator>, public Loop {
    friend Singleton;
    friend class PeriodicIO;
public:

    void stop() override;

    bool isSystemFaulted() override;

    bool runDiagnostics() override;

    void registerEnabledLoops(ILooper & enabledLooper) override;

    void onFirstStart(double timestamp) override;
    void onStart(double timestamp) override;
    void onStop(double timestamp) override;
    void onLoop(double timestamp) override;
    std::string getName() override;
    std::vector<std::string> generateReport();

private:

    std::vector<std::string> mObjList;
    static DataReporter* logReporter;    
    RobotStateEstimator();

    class PeriodicIO {
    public:
        PeriodicIO();

        NetworkDouble left_distance;
        NetworkDouble right_distance;
        NetworkDouble delta_left;
        NetworkDouble delta_right;
    
    // TODO Need network types for these 
        ck::geometry::Rotation2d gyro_angle;
        ck::geometry::Twist2d odometry_velocity;
        ck::geometry::Twist2d predicted_velocity;

        NetworkDouble left_encoder_prev_distance_;
        NetworkDouble right_encoder_prev_distance_;

        // Outputs
        NetworkDouble robot_state_loop_time;
    };

    PeriodicIO mPeriodicIO;
};

