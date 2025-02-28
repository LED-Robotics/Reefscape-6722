#include <units/angle.h>
#include <units/time.h>

namespace AlgaeConstants {
    // ports
    constexpr int kIntakePort = 9;
  
    constexpr double kIntakeDefaultPower = 0.0;

    constexpr int kWristDefaultPower = 1.0;
    // Intake states
    enum IntakeStates {
      kIntakeOff,
      kIntakePowerMode,
      kIntakeSensorMode
    };
}
