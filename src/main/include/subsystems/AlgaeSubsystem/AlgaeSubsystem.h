// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc/AnalogInput.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;

class AlgaeSubsystem : public frc2::SubsystemBase {
 public:
  AlgaeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
            /* INTAKE FUNCTIONS */
    
  /**
   * Turns the Intake state to kPowerMode.
   */
  void IntakeOn();

  /**
   * Turns the Intake state to kOff.
   */
  void IntakeOff();

  /**
   * Sets the power for the Intake to use when in kPowerMode.
   *
   * @param power the power for the intake to use
   */
  void SetIntakePower(double newPower);

  /**
   * Get the current power used by the intake.
   * 
   * @return current intake power
   */
  double GetIntakePower();

  /**
   * Sets the current state of the Intake.
   * 
   * @param newState the new state for the Intake.
   */
  void SetIntakeState(int newState);
  
  /**
   * Returns the current state of the Intake.
   *
   * @return The current state of the Intake
   */
  int GetIntakeState();

  /**
   * Sets Intake brake mode.
   */
  void SetIntakeBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigIntake();

  /**
  * Gets if a piece of coral is indexed.
  */
  bool IsAlgaeIndexed();
    
 private:
  // While the state is kOn the intake will run at the current power setting
  int intakeState = AlgaeConstants::IntakeStates::kIntakePowerMode;
  double intakePower = AlgaeConstants::kIntakeDefaultPower;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  hardware::TalonFX intakeMotor;
  frc::AnalogInput detector;
};
