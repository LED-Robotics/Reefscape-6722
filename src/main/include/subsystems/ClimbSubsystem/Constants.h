#include <units/angle.h>
#include <units/time.h>

namespace ClimbConstants {
    // ports
    constexpr int kClimbPort = 8;
    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    constexpr int kDefaultPower = 1.0;
    //  states
    enum ClimbStates {
      kOff,
      kPowerMode,
      kAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kDegreeMin{0_deg};   // encoder Turns at the Climb's minimum usable position
    constexpr units::angle::degree_t kDegreeMax{90_deg}; // meters the Climb's is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kStartOffset = 0.0;
    constexpr units::angle::degree_t kStartAngle{90_deg};
    // position deadzone
    constexpr units::angle::degree_t kAngleDeadzone{2_deg};
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kP = 1.0;
    constexpr double kRotorToGearbox = 9.0; //Will change
    constexpr double kGearboxToMechanism = 42.0 / 18.0; //Will change
    /*constexpr double kTurnsPerDegree = kRotorToGearbox * kGearboxToMechanism;*/
    constexpr double kTurnsPerDegree = 0.05863715277778;
}
