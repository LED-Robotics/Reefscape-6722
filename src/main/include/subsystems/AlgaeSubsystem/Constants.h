#include <units/angle.h>
#include <units/time.h>

namespace AlgaeConstants {
    // ports
    constexpr int kWristPort = 14;
    constexpr int kIntakePort = 15;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;
  
    constexpr double kIntakeDefaultPower = 0.25;

    constexpr int kWristDefaultPower = 1.0;
    // Intake states
    enum IntakeStates {
      kIntakeOff,
      kIntakePowerMode,
      kIntakeSensorMode
    };
    // Wrist states
    enum WristStates {
      kWristOff,
      kWristPowerMode,
      kWristAngleMode
    };
    // wrist min/max positions
    constexpr double kWristDegreeMin = -80.0;   // encoder Turns at the wrist's minimum usable position
    constexpr int kMinAngleTurns = -4000;   // encoder Turns at the wrist's minimum usable position
    constexpr int kMaxAngleTurns = 4000;   // encoder Turns at the wrist's max usable position
    constexpr double kWristDegreeMax = 80.0; // degrees the wrist is capable of moving 
    // for wrist feed forward trig
    constexpr double kTurnsPerDegree = 0.00278;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.04;
    // arm position constants
    constexpr double kWristStartOffset = 80.0;
    constexpr double kWristStartAngle = 80.0;
    // position deadzone
    constexpr double kWristPositionDeadzone = 2.0;
    // TalonFX config
    constexpr double kPWrist = 0.0;
    constexpr double kWristRotorToGearbox = 25.0; //Will change
}