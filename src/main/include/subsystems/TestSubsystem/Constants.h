#include <units/angle.h>
#include <units/time.h>
#include <units/current.h>

namespace TestConstants {
    // ports
    constexpr int kTestPort = 8;

    constexpr int kEncoderPort = 12;

    constexpr units::angle::turn_t kEncoderOffset =  -0.248779296875_tr;

    constexpr units::time::second_t kRampSeconds = 0.25_s;

    constexpr int kTestDefaultPower = 1.0;
    // Test states
    enum TestStates {
      kTestOff,
      kTestPowerMode,
      kTestAngleMode
    };
    // wrist min/max positions
    constexpr units::angle::degree_t kTestDegreeMin{-360.0_deg};   // encoder Turns at the Floor's minimum usable position
    constexpr units::angle::degree_t kTestDegreeMax{180.0_deg}; // meters the Floor is capable of moving 
    // for arm feed forward trig
    // arm position constants
    constexpr double kTestStartOffset = 0.0;
    constexpr units::angle::degree_t kTestStartAngle{10.0_deg};
    // position deadzone
    constexpr units::angle::degree_t kTestAngleDeadzone{10.0_deg};
    constexpr units::current::ampere_t kCurrentLimit = 30.0_A;

    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.352;
    // TalonFX config
    constexpr double kPTest = 140.0;
    constexpr double kDTest = 12.0;
    constexpr double kTestRotorToGearbox = 136.0 / 5.0; //Will change
    constexpr double kTestGearboxToMechanism = 68.0 / 18.0; //Will change
    // constexpr double kTurnsPerDegree = (kTestRotorToGearbox * kTestGearboxToMechanism) / 360.0;
    constexpr double kTurnsPerDegree = 1.0 / 360.0;
}
