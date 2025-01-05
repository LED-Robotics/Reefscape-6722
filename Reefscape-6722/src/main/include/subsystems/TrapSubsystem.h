// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

using namespace TrapConstants;

class TrapSubsystem : public frc2::SubsystemBase {
 public:
  TrapSubsystem(ArmSubsystem *armRef, Orchestra *orcRef);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Arm state to kOff.
   */
  void Off();
  
  /**
   * Turns the Arm state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Arm to use when in kPowerMode.
   *
   * @param power the power for the Arm to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the Arm.
   *
   * @return The current state of the Arm
   */
  int GetState();

  /**
   * Sets the current state of the Arm.
   */
  void SetState(int newState);
  
  /**
   * Returns the current position of the left Arm's Falon500.
   */
  double GetPosition();

  /**
   * Returns the current estimated angle of the Arm Subsystem.
   */
  double GetAngle();

  /**
   * Sets the target angle of the Arm.
   */
  void SetTargetAngle(double newAngle);

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Returns the current angle of the Arm.
   *
   * @return The current Arm angle
   */
  double GetCurrentAngle();

  /**
   * Sets the state of the Arm brakes.
   *
   * @param state the state of the brakes.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();
    
 private:
  //  while the state is kOn the Arm will run at the current power setting
  int state = kFollowMode;
  double power = kDefaultPower;
  double angle = kStartAngle;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX motor;

  controls::PositionVoltage ctrePosition{0_tr};
  Orchestra *orca;
  ArmSubsystem *arm;

};
