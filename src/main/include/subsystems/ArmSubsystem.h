// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem(std::function<units::length::meter_t()> getDistFunc, int *targetRef, Orchestra *orcRef);

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
  double GetLeftPosition();

  /**
   * Returns the current position of the right Arm's Falon500.
   */
  double GetRightPosition();

  /**
   * Returns the current estimated angle of the arm Subsystem.
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

  void AngleFromDistance(units::length::meter_t distance);

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
  int state = ArmConstants::kAngleMode;
  int *target;
  double power = ArmConstants::kDefaultPower;
  double angle = ArmConstants::kStartAngle;
  double autoAngle = ArmConstants::kStartAngle;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_TalonSRX left;
  hardware::TalonFX left;
  // WPI_TalonSRX right;
  hardware::TalonFX right;

  hardware::CANcoder encoder;

  std::function<units::length::meter_t()> getTargetDistance;

  controls::PositionVoltage position{0_tr};
  // controls::MotionMagicVoltage position{0_tr};
  Orchestra *orca;
};
