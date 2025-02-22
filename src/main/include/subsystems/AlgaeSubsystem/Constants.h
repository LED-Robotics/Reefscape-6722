#include <units/angle.h>
#include <units/time.h>

namespace AlgaeConstants {
    // ports
    constexpr int kWristPort = 8;
    constexpr int kIntakePort = 9;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset = -0.46044921875_tr;

    constexpr units::time::second_t kRampSeconds = 0.5_s;
  
    constexpr double kIntakeDefaultPower = 0.0;

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
    constexpr units::angle::degree_t kWristDegreeMin{-45_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kWristDegreeMax{90_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kWristStartOffset = 0.0;
    constexpr units::angle::degree_t kWristStartAngle{90_deg};
    // position deadzone
    constexpr units::angle::degree_t kWristAngleDeadzone{2_deg};
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPWrist = 1.0;
    constexpr double kWristRotorToGearbox = 9.0; //Will change
    constexpr double kWristGearboxToMechanism = 42.0 / 18.0; //Will change
    /*constexpr double kTurnsPerDegree = kWristRotorToGearbox * kWristGearboxToMechanism;*/
    constexpr double kTurnsPerDegree = 0.05863715277778;
}
