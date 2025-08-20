// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include "led_libraries/SmartMotor.h"

using namespace frc;
using namespace ctre::phoenix6;

class PositionalSubsystem : public frc2::SubsystemBase {
 public:
  PositionalSubsystem(std::vector<SmartMotor*> motors);

  enum States { kOff, kPowerMode, kPositionMode };
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Cascade state to kOff.
   */
  void Off();
  
  /**
   * Turns the Cascade state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Cascade to use when in kPowerMode.
   *
   * @param power the power for the Cascade to use
   */
  void SetPower(double newPower);

  /**
   * @return The current state of the Cascade
   */
  int GetState();

  /**
   * Sets the current state of the Cascade.
   */
  void SetState(int newState);
  
  /**
   * Returns the current position of the left Cascade's KrakenX60.
   */
  units::angle::turn_t GetLeftPosition();

  /**
   * Returns the current position of the right Cascade's KrakenX60.
   */
  units::angle::turn_t GetRightPosition();

  /**
   * Returns the current estimated angle of the Cascade Subsystem.
   */
  units::angle::turn_t GetPosition();

  /**
   * Sets the target angle of the Cascade.
   */
  void SetTargetPosition(units::angle::turn_t newPosition);

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Sets the state of the Cascade brakes.
   *
   * @param state the state of the brakes.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::turn_t target);
    
 private:
  // while the state is kOn the Cascade will run at the current power setting
  int state = States::kOff;
  double power = 0.0;
  units::angle::turn_t position{0.0_tr};
  units::angle::turn_t microAdjust{0.0_tr};
  std::vector<SmartMotor*> motors;
};
