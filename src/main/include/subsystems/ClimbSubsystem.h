// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "ArmSubsystem.h"

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem(Orchestra *orcRef);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Deflector state to kOff.
   */
  void Off();
  
  /**
   * Turns the Deflector state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Deflector to use when in kPowerMode.
   *
   * @param power the power for the Deflector to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the Deflector.
   *
   * @return The current state of the Deflector
   */
  int GetState();

  /**
   * Sets the current state of the Deflector.
   */
  void SetState(int newState);
  
  /**
   * Returns the current position of the left Deflector's Falon500.
   */
  double GetLeftPosition();

  /**
   * Returns the current position of the right Deflector's Falon500.
   */
  double GetRightPosition();

  /**
   * Returns the current estimated angle of the Deflector Subsystem.
   */
  double GetAngle();

  /**
   * Sets the target angle of the Deflector.
   */
  void SetTargetPosition(double newPosition);

  /**
   * Returns whether the subsystem is at its intended target position.
   */
  bool IsAtTarget();

  /**
   * Returns the current angle of the Deflector.
   *
   * @return The current Deflector angle
   */
  double GetCurrentAngle();

  /**
   * Sets the state of the Deflector brakes.
   *
   * @param state the state of the brakes.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();

  void AngleFromDistance(units::length::meter_t distance);
    
 private:
  //  while the state is kOn the Deflector will run at the current power setting
  int state = ClimbConstants::kPositionMode;
  int *target;
  double power = ClimbConstants::kDefaultPower;
  double position = ClimbConstants::kStartPosition;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  // WPI_TalonSRX left;
  // hardware::TalonFX left;
  // WPI_TalonSRX right;
  // hardware::TalonFX right;

  controls::PositionVoltage ctrePosition{0_tr};
  // controls::MotionMagicVoltage ctrePosition{0_tr};
  Orchestra *orca;

};
