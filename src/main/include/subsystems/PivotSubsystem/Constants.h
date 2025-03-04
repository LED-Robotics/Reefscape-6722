#include <units/angle.h>
#include <units/time.h>

namespace PivotConstants {
    // ports
    constexpr int kPivotPort = 8;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;

    constexpr int kPivotDefaultPower = 1.0;
    // Pivot states
    enum PivotStates {
      kPivotOff,
      kPivotPowerMode,
      kPivotAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kPivotDegreeMin{-180_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kPivotDegreeMax{180_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kPivotStartOffset = 0.0;
    constexpr units::angle::degree_t kPivotStartAngle{90_deg};
    // position deadzone
    constexpr units::angle::degree_t kPivotAngleDeadzone{2_deg};
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPPivot = 1.0;
    constexpr double kPivotRotorToGearbox = 9.0; //Will change
    constexpr double kPivotGearboxToMechanism = 42.0 / 18.0; //Will change
    /*constexpr double kTurnsPerDegree = kPivotRotorToGearbox * kPivotGearboxToMechanism;*/
    constexpr double kTurnsPerDegree = 0.05863715277778;
}