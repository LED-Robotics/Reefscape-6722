#include <units/length.h>
#include <units/current.h>

namespace CascadeConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kEncoderPort = 14;

    constexpr units::angle::turn_t kEncoderOffset = 0.498291015625_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    // default power in power mode

    constexpr double kDefaultPower = 0.0;
    constexpr double kStageMultiplier = 1.7825903614458;
    // arm position constants
    constexpr units::length::meter_t kStartPosition{0.645_m};
    // constexpr units::length::meter_t kStartPosition{0.13335_m};
    // arm min/max positions
    constexpr units::length::meter_t kCascadeMeterMin{kStartPosition};   // encoder Turns at the Cascade's minimum usable position
    constexpr units::length::meter_t kCascadeMeterMax{2.05_m}; // meters the Cascade is capable of moving 
    // for arm feed forward trig
    // constexpr double kTurnsPerMeter = 6.066743827160493;
    constexpr double kTurnsPerMeter = 2.71577381;
    // position deadzone
    constexpr units::length::meter_t kPositionDeadzone{10.0_cm};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;
    // TalonFX config
    constexpr double kP = 13.0;
    constexpr double kD = 2.1;
    /*constexpr double kD = 0.0;*/
    constexpr double kG = 0.4;
    /*constexpr double kG = 0.0;*/
    constexpr double kRotorToGearbox = 25.0;
}
