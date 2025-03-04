// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace AlgaeConstants;
using namespace frc;

AlgaeSubsystem::AlgaeSubsystem()
  : intakeMotor{kIntakePort} {
      ConfigIntake();
}

void AlgaeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  //Intake Control
  if(intakeState == IntakeStates::kIntakeOff) {
    intakeMotor.Set(0.0);
  } else {
    intakeMotor.Set(intakePower);
  }
}


void AlgaeSubsystem::IntakeOn() {
  intakeState = IntakeStates::kIntakePowerMode;
}

void AlgaeSubsystem::IntakeOff() {
  intakeState = IntakeStates::kIntakeOff;
}

void AlgaeSubsystem::SetIntakePower(double newPower) {
  intakePower = newPower;
}

double AlgaeSubsystem::GetIntakePower() {
  return intakePower;
}

void AlgaeSubsystem::SetIntakeState(int newState) {
  intakeState = newState;
}

int AlgaeSubsystem::GetIntakeState() {
  return intakeState;
}

void AlgaeSubsystem::SetIntakeBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  intakeMotor.GetConfigurator().Apply(updated, 50_ms);
}

void AlgaeSubsystem::ConfigIntake() {
  configs::TalonFXConfiguration algaeIntakeConfig{};

  algaeIntakeConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  algaeIntakeConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  algaeIntakeConfig.MotorOutput.Inverted = true;

  intakeMotor.GetConfigurator().Apply(algaeIntakeConfig);
}