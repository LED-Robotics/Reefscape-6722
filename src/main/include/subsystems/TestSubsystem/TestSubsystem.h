// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "led_libraries/PositionalSubsystem.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class TestSubsystem : public PositionalSubsystem {
 public:
  TestSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* test FUNCTIONS */

  /**
   * Turns the Intake state to kAngleMode.
   */
  void TestOn();

  /**
   * Turns the Intake state to kOff.
   */
  void TestOff();

  /**
   * Sets the power for the Test to use when in kPowerMode.
   *
   * @param power the power for the test to use
   */
  void SetTestPower(double newPower);
  
  /**
   * Get the current power used by the Test.
   * 
   * @return current test power
   */
  double GetTestPower();

  /**
   * Sets the target angle of the Test.
   * 
   * @param newAngle new angle for the test
   */
  void SetTargetAngle(units::angle::degree_t newAngle);

  /**
   * Returns the current estimated angle of the test.
   * 
   * @return current test angle
   */
  units::angle::degree_t GetAngle();

  /**
   * Returns the position from the TalonFX motor controller.
   *
   * @return the TalonFX reported position
   */
  double GetTestPosition();

  /**
   * Returns whether the subsystem is at its intended target position.
   * 
   * @return If the test is at it's target
   */
  bool IsAtTarget();

  /**
   * Sets the current state of the Test.
   * 
   * @param newState the new state for the Test.
   */
  void SetTestState(int newState);
  
  /**
   * Returns the current state of the Test.
   *
   * @return The current state of the Test
   */
  int GetTestState();

  /**
   * Sets Test brake mode.
   * 
   * @param state turn the brakes on or off
   */
  void SetTestBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigTest();

  /**
   * Create command to move Subsystem
   */
  frc2::CommandPtr GetMoveCommand(units::angle::degree_t target);
    
 private:
  // While the state is kOn the test will run on the angle mode.
  int state = TestConstants::TestStates::kTestAngleMode;
  double power = TestConstants::kTestDefaultPower;
  units::angle::degree_t angle{90_deg};
  units::angle::degree_t microAdjust{0_deg};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX test;

  hardware::CANcoder testEncoder;

  controls::PositionVoltage testPosition{0_tr};
};
