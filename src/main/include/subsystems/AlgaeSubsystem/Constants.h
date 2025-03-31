#include <units/angle.h>
#include <units/time.h>

namespace AlgaeConstants {
    // ports
    constexpr int kIntakePort = 9;
  
    constexpr int kUltrasonicPort = 2;

    constexpr double kIntakeDefaultPower = 0.0;

    constexpr int kWristDefaultPower = 1.0;

    constexpr double kUltrasonicThreshold = 0.5;

    constexpr double kHoldingPower = -0.2;


    // Intake states
    enum IntakeStates {
      kIntakeOff,
      kIntakePowerMode,
      kIntakeSensorMode
    };
}
