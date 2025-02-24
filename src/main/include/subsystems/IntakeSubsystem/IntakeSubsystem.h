// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <iostream>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;
using namespace rev::spark;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Turns the intake state to kOff.
   */
  void Off();

  /**
   * Turns the intake state to kPowerMode.
   * The intake uses the subsystems current
   * power setting while in this mode.
   */
  void UsePowerMode();

  /**
   * Sets the power to use while in kPowerMode.
   */
  void SetPower(double newPower);

  /**
   * Returns the current power setting of the intake.
   *
   * @return The current power setting of the intake
   */
  double GetPower();

  /**
   * Returns the current state of the intake.
   *
   * @return The current state of the intake
   */  
  int GetState();

  /**
   * Sets the current state of the intake.
   */  
  void SetState(int newState);

  /**
   * Sets Intake brake mode.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();

  /**
  * Gets if a piece of coral is indexed.
  */
  bool IsCoralIndexed();
    
 private:
  int state = IntakeConstants::IntakeStates::kSensorMode;
  double power = 0.0;
  bool previousVal = false;
  bool sensorTripped;
  bool powerOff;
  int trippedPower;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  /*hardware::TalonFX left;*/
  /*hardware::TalonFX right;*/

  SparkFlex left;
  SparkFlex right;
  DigitalInput beamBreak;
};
