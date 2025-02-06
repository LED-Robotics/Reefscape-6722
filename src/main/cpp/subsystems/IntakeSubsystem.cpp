// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;
using namespace frc;

IntakeSubsystem::IntakeSubsystem()
    /*: left{kLeftPort},*/
    /*  right{kRightPort} {*/
    : left{kLeftPort, SparkMax::MotorType::kBrushless},
      right{kRightPort, SparkMax::MotorType::kBrushless} {
    // : intakeMotor{kIntakePort, CANSparkLowLevel::MotorType::kBrushless} {
      right.SetInverted(false);
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Intake power control
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    // power limiting 
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    left.Set(power);
    right.Set(power);
    // else intakeMotor.Set(0.0);
  } else if(state == kSensorMode) {
    /*if(!shooter->IsNoteIndexed()) intakeMotor.Set(-IntakeConstants::kIntakeSpeed);*/
    /*else intakeMotor.Set(0.0);*/
  }
}

void IntakeSubsystem::Off() {
  state = kOff;
}

void IntakeSubsystem::UsePowerMode() {
  state = kPowerMode;
}

void IntakeSubsystem::SetPower(double newPower) {
  power = newPower;
  // if(power < kIntakeDeadzone) power = 0.0;
}

double IntakeSubsystem::GetPower() {
  return power;
}

int IntakeSubsystem::GetState() {
  return state;
}

void IntakeSubsystem::SetState(int newState) {
  state = newState;
}

void IntakeSubsystem::SetBrakeMode(bool state) {
  /*signals::NeutralModeValue mode;*/
  /*if(state) mode = signals::NeutralModeValue::Brake;*/
  /*else mode = signals::NeutralModeValue::Coast;*/
  /*configs::MotorOutputConfigs updated;*/
  /*updated.WithNeutralMode(mode);*/
  /**/
  /*intakeMotor.GetConfigurator().Apply(updated, 50_ms);*/
}

void IntakeSubsystem::ConfigMotors() {
  // wristMotor.Config_kP(0, kP, 100);
}
