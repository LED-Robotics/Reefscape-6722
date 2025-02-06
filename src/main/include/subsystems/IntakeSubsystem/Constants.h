#include <units/length.h>

namespace IntakeConstants {
    // Ports
    constexpr int kLeftPort = 1;
    constexpr int kRightPort = 2;

    // States
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kSensorMode = 2;

    // Default power in power mode
    constexpr double kDefaultPower = 1.0;
}
