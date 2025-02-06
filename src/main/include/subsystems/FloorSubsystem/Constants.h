#include <units/length.h>

namespace FloorConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kEncoderPort = 14;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kAngleMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr units::angle::degree_t kFloorAngleMin{15_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kFloorAngleMax{90_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 6.066743827160493;
    // arm position constants
    constexpr double kStartOffset = 0.0;
    constexpr units::angle::degree_t kStartAngle{90_deg};
    // position deadzone
    constexpr units::angle::degree_t kAngleDeadzone{10_deg};
    // TalonFX config
    constexpr double kP = 6.0;
    constexpr double kRotorToGearbox = 25.0;
}
