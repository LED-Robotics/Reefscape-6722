#include <units/length.h>

namespace IntakeConstants {
    // Ports
    constexpr int kLeftPort = 0;
    constexpr int kRightPort = 0;
    constexpr int kBeamBreakPort = 1;

    // States
    enum IntakeStates {
      kOff,
      kPowerMode,
      kSensorMode
    };

    // Default power in power mode
    constexpr double kDefaultPower = 1.0;
}
