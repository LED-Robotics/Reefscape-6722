#include <units/length.h>

namespace CascadeConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kEncoderPort = 14;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr units::length::meter_t kCascadeMeterMin{0.15_m};   // encoder Turns at the Cascade's minimum usable position
    constexpr units::length::meter_t kCascadeMeterMax{0.7_m}; // meters the Cascade is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerMeter = 6.066743827160493;
    // arm position constants
    constexpr double kStartOffset = 0.0;
    constexpr units::length::meter_t kStartPosition{0.0_m};
    // position deadzone
    constexpr units::length::meter_t kPositionDeadzone{3.0_cm};
    // TalonFX config
    constexpr double kP = 6.0;
    constexpr double kRotorToGearbox = 25.0;
}
