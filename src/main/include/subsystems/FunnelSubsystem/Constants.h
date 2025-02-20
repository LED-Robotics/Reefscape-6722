#include <units/angle.h>
#include <units/time.h>


namespace FunnelConstants {
  // ports
  constexpr int kFunnelMotorPort = 12;
  constexpr int kFunnelDefaultPower = 1.0;
  
  constexpr units::time::second_t kRampSeconds = 0.5_s;
  // states
  enum FunnelStates {
    kFunnelOff,
    kFunnelPowerMode,
    kFunnelAngleMode
  };
  // Funnel min/max positions
  constexpr units::angle::degree_t kFunnelAngleMin{-60_deg};   // encoder Turns at the Funnel's minimum usable position
  constexpr units::angle::degree_t kFunnelAngleMax{0_deg}; // meters the Funnels is capable of moving 

  // for Funnel feed forward trig
  constexpr double kTurnsPerDegree = 0.00278;
  // feed forward at max gravity i.e. 90 degrees off the floor
  constexpr double kMaxFeedForward = 0.04;
  // arm position constants
  constexpr units::angle::degree_t kStartAngle{0_deg};
  constexpr double kStartOffset = 0.0;
  // position deadzone
  constexpr units::angle::degree_t kAngleDeadzone{3_deg};
  // TalonFX config
  constexpr double kP = 0.0;
  constexpr double kRotorToGearbox = 25.0; //Will change
}
