#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* CLIMB FUNCTIONS */

  /**
   * Turns the Intake state to kAngleMode.
   */
  void On();

  /**
   * Turns the Intake state to kOff.
   */
  void Off();

  /**
   * Sets the power for the Climb to use when in kPowerMode.
   *
   * @param power the power for the Climb to use
   */
  void SetPower(double newPower);
  
  /**
   * Get the current power used by the Climb.
   * 
   * @return current Climb power
   */
  double GetPower();

  /**
   * Sets the target angle of the Climb.
   * 
   * @param newAngle new angle for the Climb
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the Climb.
   * 
   * @return current Climb angle
   */
  units::angle::degree_t GetAngle();

  /**
   * Returns the position from the TalonFX motor controller.
   *
   * @return the TalonFX reported position
   */
  double GetPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the Climb is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Climb.
   * 
   * @param newState the new state for the Climb.
   */
  void SetState(int newState);
  
  /**
   * Returns the current state of the Climb.
   *
   * @return The current state of the Climb
   */
  int GetState();

  /**
   * Sets Climb brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotor();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // While the state is kOn the Climb will run on the angle mode.
  int state = ClimbConstants::ClimbStates::kAngleMode;
  double power = ClimbConstants::kDefaultPower;
  units::angle::degree_t angle = ClimbConstants::kStartAngle;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX climb;

  // hardware::CANcoder encoder;

  controls::PositionVoltage positionController{0_tr};
};