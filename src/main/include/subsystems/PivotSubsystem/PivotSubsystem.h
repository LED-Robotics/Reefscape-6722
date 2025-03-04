// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class PivotSubsystem : public frc2::SubsystemBase {
 public:
  PivotSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* PIVOT FUNCTIONS */

  /**
   * Turns the Intake state to kAngleMode.
   */
  void PivotOn();

  /**
   * Turns the Intake state to kOff.
   */
  void PivotOff();

  /**
   * Sets the power for the Pivot to use when in kPowerMode.
   *
   * @param power the power for the pivot to use
   */
  void SetPivotPower(double newPower);
  
  /**
   * Get the current power used by the Pivot.
   * 
   * @return current pivot power
   */
  double GetPivotPower();

  /**
   * Sets the target angle of the Pivot.
   * 
   * @param newAngle new angle for the pivot
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the pivot.
   * 
   * @return current pivot angle
   */
  units::angle::degree_t GetAngle();

  /**
   * Returns the position from the TalonFX motor controller.
   *
   * @return the TalonFX reported position
   */
  double GetPivotPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the pivot is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Pivot.
   * 
   * @param newState the new state for the Pivot.
   */
  void SetPivotState(int newState);
  
  /**
   * Returns the current state of the Pivot.
   *
   * @return The current state of the Pivot
   */
  int GetPivotState();

  /**
   * Sets Pivot brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetPivotBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigPivot();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // While the state is kOn the pivot will run on the angle mode.
  int state = PivotConstants::PivotStates::kPivotAngleMode;
  double power = PivotConstants::kPivotDefaultPower;
  units::angle::degree_t angle{90_deg};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX pivot;

  hardware::CANcoder pivotEncoder;

  controls::PositionVoltage pivotPosition{0_tr};
};