#include <units/length.h>

namespace CoralConstants {
    // Ports
    constexpr int kIntake1Port = 10;
    constexpr int kIntake2Port = 11;
    constexpr int kBeamBreakPort = 1;
    constexpr double kHoldingPower = -0.03;

    // States
    enum CoralStates {
      kOff,
      kPowerMode,
      kSensorMode
    };

    // Default power in power mode
    constexpr double kDefaultPower = 1.0;
}
