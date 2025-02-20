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
    constexpr units::angle::degree_t kWristDegreeMin{-80_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kWristDegreeMax{80_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 6.066743827160493;
    // arm position constants
    constexpr double kWristStartOffset = 0.0;
    constexpr units::angle::degree_t kWristStartAngle{0_deg};
    // position deadzone
    constexpr units::angle::degree_t kWristAngleDeadzone{2_deg};
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.04;
    // TalonFX config
    constexpr double kPWrist = 0.0;
    constexpr double kWristRotorToGearbox = 25.0; //Will change
}