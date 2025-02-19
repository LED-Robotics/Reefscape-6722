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

class CascadeSubsystem : public frc2::SubsystemBase {
 public:
  CascadeSubsystem();

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
   * Returns the current state of the Cascade.
   *
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
  units::length::meter_t GetLeftPosition();

  /**
   * Returns the current position of the right Cascade's KrakenX60.
   */
  units::length::meter_t GetRightPosition();

  /**
   * Returns the current estimated angle of the Cascade Subsystem.
   */
  units::length::meter_t GetPosition();

  /**
   * Sets the target angle of the Cascade.
   */
  void SetTargetPosition(units::length::meter_t newPosition);

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
  // while the state is kOn the Cascade will run at the current power setting
  int state = CascadeConstants::CascadeStates::kPositionMode;
  double power = CascadeConstants::kDefaultPower;
  units::length::meter_t position{CascadeConstants::kStartPosition};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX left;
  hardware::TalonFX right;

  // hardware::CANcoder encoder;

  controls::PositionVoltage positionController{0_tr};
  // controls::MotionMagicVoltage positionController{0_tr};
};
