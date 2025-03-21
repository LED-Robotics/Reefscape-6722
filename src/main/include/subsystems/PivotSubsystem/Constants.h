#include <units/angle.h>
#include <units/time.h>
#include <units/current.h>

namespace PivotConstants {
    // ports
    constexpr int kPivotPort = 8;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset =  -0.3671875_tr;

    constexpr units::time::second_t kRampSeconds = 0.25_s;

    constexpr int kPivotDefaultPower = 1.0;
    // Pivot states
    enum PivotStates {
      kPivotOff,
      kPivotPowerMode,
      kPivotAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kPivotDegreeMin{-360.0_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kPivotDegreeMax{180.0_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kPivotStartOffset = 0.0;
    constexpr units::angle::degree_t kPivotStartAngle{10.0_deg};
    // position deadzone
    constexpr units::angle::degree_t kPivotAngleDeadzone{10.0_deg};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;

    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPPivot = 140.0;
    constexpr double kDPivot = 12.0;
    constexpr double kPivotRotorToGearbox = 136.0 / 5.0; //Will change
    constexpr double kPivotGearboxToMechanism = 17.0 / 4.0; //Will change
    // constexpr double kTurnsPerDegree = (kPivotRotorToGearbox * kPivotGearboxToMechanism) / 360.0;
    constexpr double kTurnsPerDegree = 1.0 / 360.0;
}
