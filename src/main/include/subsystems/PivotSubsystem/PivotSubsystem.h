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
#include "led_libraries/PositionalSubsystem.h"
#include "led_libraries/TalonSmartMotor.h"

using namespace frc;
using namespace ctre::phoenix6;

class PivotSubsystem : public PositionalSubsystem {
 public:
  PivotSubsystem();

  units::angle::degree_t ToDegrees(units::angle::turn_t turns);

  units::angle::turn_t ToTurns(units::angle::degree_t degrees);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* PIVOT FUNCTIONS */

  /**
   * Sets the target angle of the Pivot.
   * 
   * @param newAngle new angle for the pivot
   */
  void SetTargetDegrees(units::angle::degree_t newAngle, double feedForward = 0.0);

  /**
   * Returns the current estimated angle of the pivot.
   * 
   * @return current pivot angle
   */
  units::angle::degree_t GetAngleDegrees();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the pivot is at it's target
   */
  bool IsAtTarget();

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
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  TalonSmartMotor pivot;

  hardware::CANcoder pivotEncoder;
};
