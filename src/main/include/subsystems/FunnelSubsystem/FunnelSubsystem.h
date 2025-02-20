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

class FunnelSubsystem : public frc2::SubsystemBase {
 public:
  FunnelSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Intake state to kAngleMode.
   */
  void On();

  /**
   * Turns the Intake state to kOff.
   */
  void Off();

  /**
   * Sets the power for the Funnel to use when in kPowerMode.
   *
   * @param power the power for the Funnel to use
   */
  void SetPower(double newPower);
  
  /**
   * Get the current power used by the Funnel.
   * 
   * @return current Funnel power
   */
  double GetPower();

  /**
   * Sets the target angle of the Funnel.
   * 
   * @param newAngle new angle for the Funnel
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the Funnel.
   * 
   * @return current Funnel angle
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
   * @return If the Funnel is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Funnel.
   * 
   * @param newState the new state for the Funnel.
   */
  void SetState(int newState);
  
  /**
   * Returns the current state of the Funnel.
   *
   * @return The current state of the Funnel
   */
  int GetState();

  /**
   * Sets Funnel brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetBrakeMode(bool state);

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotor();
    
 private:
  // While the state is kOn the Funnel will run on the angle mode.
  int funnelState = FunnelConstants::FunnelStates::kFunnelAngleMode;
  double funnelPower = FunnelConstants::kFunnelDefaultPower;
  units::angle::degree_t funnelAngle{FunnelConstants::kStartAngle};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX funnel;

  // hardware::CANcoder encoder;

  controls::PositionVoltage positionController{0_tr};
  // controls::MotionMagicVoltage positionController{0_tr};
};
