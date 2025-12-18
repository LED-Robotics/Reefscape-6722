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
#include "ctre/phoenix6/CANBus.hpp"
#include "led_libraries/Subsystems/PositionalSubsystem.h"
#include "led_libraries/SmartMotor/TalonSmartMotor.h"
#include "units/angle.h"

using namespace frc;
using namespace ctre::phoenix6;

class CascadeSubsystem : public PositionalSubsystem {
 public:
  CascadeSubsystem();

  units::length::meter_t ToMeters(units::angle::turn_t turns);

  units::angle::turn_t ToTurns(units::length::meter_t meters);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Returns the current position of the left Cascade's KrakenX60.
   */
  units::length::meter_t GetLeftPosition();

  /**
   * Returns the current position of the right Cascade's KrakenX60.
   */
  units::length::meter_t GetRightPosition();

  /**
   * Returns the current estimated angle of the Cascade Subsystem.
   */
  units::length::meter_t GetPositionMeters();

  /**
   * Sets the target angle of the Cascade.
   */
  void SetTargetMeters(units::length::meter_t newPosition);

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
  frc2::CommandPtr GetMoveCommand(units::length::meter_t target);
    
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers

  TalonSmartMotor left;
  TalonSmartMotor right;

  // hardware::CANcoder encoder;
};
