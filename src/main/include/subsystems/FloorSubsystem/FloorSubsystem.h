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

class FloorSubsystem : public frc2::SubsystemBase {
 public:
  FloorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Floor state to kOff.
   */
  void Off();
  
  /**
   * Turns the Floor state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Floor to use when in kPowerMode.
   *
   * @param power the power for the Floor to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the Floor.
   *
   * @return The current state of the Floor
   */
  int GetState();

  /**
   * Sets the current state of the Floor.
   */
  void SetState(int newState);
  
  /**
   * Returns the current position of the left Floor's KrakenX60.
   */
  units::angle::degree_t GetLeftAngle();

  /**
   * Returns the current position of the right Floor's KrakenX60.
   */
  units::angle::degree_t GetRightAngle();

  /**
   * Returns the current estimated angle of the Floor Subsystem.
   */
  units::angle::degree_t GetAngle();

  /**
   * Sets the target angle of the Floor.
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Sets the state of the Floor brakes.
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
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // while the state is kOn the Floor will run at the current power setting
  int state = FloorConstants::kAngleMode;
  double power = FloorConstants::kDefaultPower;
  units::angle::degree_t angle{FloorConstants::kStartAngle};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX left;
  hardware::TalonFX right;

  // hardware::CANcoder encoder;

  controls::PositionVoltage angleController{0_tr};
  // controls::MotionMagicVoltage positionController{0_tr};
};
