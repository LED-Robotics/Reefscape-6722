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
   * Sends control signals to motors
   * This function MUST be called in Periodic of the derived class
   */
  void RunMotors();
  
  /**
   * Turns the motors' state to kOff
   */
  void Off();
  
  /**
   * Turns the motors' state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the motors to use when in kPowerMode
   *
   * @param power the power for the motors to use
   */
  void SetPower(double newPower);

  /**
   * @return The current state of the motors
   */
  int GetState();

  /**
   * Sets the current state of the motors
   */
  void SetState(int newState);
  
  /**
   * Returns the current estimated angle of the motors Subsystem
   */
  units::angle::turn_t GetPosition();

  /**
   * Sets the target turns of the motors.
   */
  void SetTargetPosition(units::angle::turn_t newPosition, double feedForward = 0.0);

  /**
   * Sets the target turns of the motors.
   */
  void SetNudge(units::angle::turn_t newNudge);

  /**
   * Sets the feed forward of the subsystem.
   */
  void SetFeedForward(double power);

 protected:
  // while the state is kOn the motors will run at the current power setting
  int state = States::kOff;
  double power = 0.0;
  double feedForward = 0.0;
  units::angle::turn_t position{0.0_tr};
  units::angle::turn_t nudge{0.0_tr};
  std::vector<SmartMotor*> motors;
};
