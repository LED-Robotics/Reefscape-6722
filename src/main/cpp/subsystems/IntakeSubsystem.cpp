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
    : right{kRightPort, SparkMax::MotorType::kBrushless},
      left{kLeftPort, SparkMax::MotorType::kBrushless},
      beamBreak{kBeamBreakPort} {
    // : intakeMotor{kIntakePort, CANSparkLowLevel::MotorType::kBrushless} {
      right.SetInverted(false);
      left.SetInverted(true);
      previousVal = IsCoralIndexed();
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Intake power control
  SmartDashboard::PutNumber("Indexed", IsCoralIndexed());
  SmartDashboard::PutNumber("Previous", previousVal);
  SmartDashboard::PutNumber("powerOff", powerOff);
  if(state == IntakeStates::kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == IntakeStates::kPowerMode) {
    // power limiting 
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    left.Set(power);
    right.Set(power);
    // else intakeMotor.Set(0.0);
  } else if(state == IntakeStates::kSensorMode) {
    bool indexed = IsCoralIndexed();
    if(indexed && !previousVal) {
      powerOff = true;
      trippedPower = fabs(power);
    } 
    SmartDashboard::PutNumber("tripped", trippedPower);

    if(powerOff && fabs(power) < trippedPower) {
      trippedPower = fabs(power);
    }
    SmartDashboard::PutNumber("power", fabs(power));
    SmartDashboard::PutNumber("Skibidi", fabs(power) - trippedPower);

    if(fabs(power) < 0.10 && powerOff) {
      powerOff = false;
      power = 0.0;
    }
    if(powerOff) {
      left.Set(0);
      right.Set(0);
    } else {
      left.Set(power * 0.3);
      right.Set(power * 0.3);
    }

    previousVal = indexed;
  }
}

void IntakeSubsystem::Off() {
  state = IntakeStates::kOff;
}

void IntakeSubsystem::UsePowerMode() {
  state = IntakeStates::kPowerMode;
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

bool IntakeSubsystem::IsCoralIndexed() {
  return !beamBreak.Get();
}
