#include <units/length.h>

namespace CoralConstants {
    // Ports
    constexpr int kIntakePort = 2;
    constexpr int kBeamBreakPort = 1;

    // States
    enum CoralStates {
      kOff,
      kPowerMode,
      kSensorMode
    };

    // Default power in power mode
    constexpr double kDefaultPower = 1.0;
}
