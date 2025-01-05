// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/TrapSubsystem.h"

using namespace frc;

TrapSubsystem::TrapSubsystem(ArmSubsystem *armRef, Orchestra *orcRef)
  : motor{kHatchPort} {
    orca = orcRef;
    arm = armRef;
    orca->AddInstrument(motor);
    SmartDashboard::PutNumber("Claw Position", angle);
}

void TrapSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetTargetAngle(SmartDashboard::GetNumber("Claw Position", angle));
  if(state == kOff) {
    motor.Set(0.0);
  } else if(state == kPowerMode) {
    motor.Set(power);
  } else if(state == kAngleMode) {
    // feed forward should be a changing constant that increases as the deflector moves further. It should be a static amount of power to overcome gravity.
    // double deflectorAngle = GetAngle() * (M_PI/180);
    // double feedForward = 0.0;
    // double feedForward = sin(deflectorAngle) * kMaxFeedForward;
    // SmartDashboard::PutNumber("wristAngle", GetCurrentAngle());  // print to Shuffleboard
    // SmartDashboard::PutNumber("wristPos", wristMotor.GetSelectedSensorPosition());  // print to Shuffleboard
    // units::angle::turn_t target{state == kPositionMode ? position : (angle * kTurnsPerDegree) + (arm->GetAngle() * kTurnsPerDegree)};
    // motor.SetControl(ctrePosition
    //   .WithPosition(units::angle::turn_t{angle * kTurnsPerDegree}));
      // .WithFeedForward(units::volt_t{feedForward}));
  } else if(state == kFollowMode) {
    // double newTarget = 190.0 - arm->GetAngle();
    // motor.SetControl(ctrePosition
    //   .WithPosition(units::angle::turn_t{newTarget * kTurnsPerDegree}));
  }
}

void TrapSubsystem::Off() {
  state = kOff;
}

void TrapSubsystem::On() {
  state = kPowerMode;
}

void TrapSubsystem::SetPower(double newPower) {
  power = newPower;
}

void TrapSubsystem::SetState(int newState) {
  state = newState;
}

int TrapSubsystem::GetState() {
  return state;
}

double TrapSubsystem::GetPosition() {
  return motor.GetPosition().GetValue().value();
}

double TrapSubsystem::GetAngle() {
  return GetPosition() / kTurnsPerDegree;
}

void TrapSubsystem::SetTargetAngle(double newAngle) {
  angle = newAngle;
  if(angle < kArmDegreeMin) angle = kArmDegreeMin;
  if(angle > kArmDegreeMax) angle = kArmDegreeMax;
}

bool TrapSubsystem::IsAtTarget() {
  double target = angle * kTurnsPerDegree;
  double pos = GetPosition();
  bool atTarget = pos > target - (kPositionDeadzone / 2) && pos < target + (kPositionDeadzone / 2);
  return atTarget;
}

void TrapSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  motor.GetConfigurator().Apply(updated, 50_ms);
}

void TrapSubsystem::ConfigMotors() {
  // left.Config_kP(0, kP, 100);
  // right.Config_kP(0, kP, 100);
}