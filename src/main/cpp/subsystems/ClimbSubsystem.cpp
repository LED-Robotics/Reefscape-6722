// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimbConstants;
using namespace frc;

ClimbSubsystem::ClimbSubsystem(Orchestra *orcRef)
    // : left{kLeftMotorPort},
    // right{kRightMotorPort} 
    {
      orca = orcRef;
      configs::TalonFXConfiguration climbConfig{};
      SmartDashboard::PutNumber("Climb Position", position);

      // climbConfig.Slot0.kP = kP;  

}

void ClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetTargetPosition(SmartDashboard::GetNumber("Climb Position", position));
  if(state == kOff) {
    // left.Set(0.0);
    // right.Set(0.0);
  } else if(state == kPowerMode) {
    // left.Set(power);
    // right.Set(power);
  } else if(state == kPositionMode) {
    // feed forward should be a changing constant that increases as the deflector moves further. It should be a static amount of power to overcome gravity.
    // double deflectorAngle = GetAngle() * (M_PI/180);
    // double feedForward = 0.0;
    // double feedForward = sin(deflectorAngle) * kMaxFeedForward;
    // SmartDashboard::PutNumber("wristAngle", GetCurrentAngle());  // print to Shuffleboard
    // SmartDashboard::PutNumber("wristPos", wristMotor.GetSelectedSensorPosition());  // print to Shuffleboard
    // units::angle::turn_t target{state == kPositionMode ? position : (angle * kTurnsPerDegree) + (arm->GetAngle() * kTurnsPerDegree)};
    
    // double targAngle = 0.0;
    // if(*target == GlobalConstants::kArbitrary) {
    //   targAngle = angle;
    // } else {
    //   AngleFromDistance(getTargetDistance());
    //   targAngle = autoAngle;
    // }
    // units::angle::turn_t posTarget{position};
    // left.SetControl(ctrePosition
    //   .WithPosition(posTarget));
    // right.SetControl(ctrePosition
    //   .WithPosition(posTarget));
    // right.SetControl(ctrePosition
      // .WithPosition(posTarget));
      // right.SetControl(controls::StrictFollower{left.GetDeviceID()});
  }
}

void ClimbSubsystem::Off() {
  state = kOff;
}

void ClimbSubsystem::On() {
  state = kPowerMode;
}

void ClimbSubsystem::SetPower(double newPower) {
  power = newPower;
}

void ClimbSubsystem::SetState(int newState) {
  state = newState;
}

int ClimbSubsystem::GetState() {
  return state;
}

void ClimbSubsystem::SetTargetPosition(double newPosition) {
  position = newPosition;
}

double ClimbSubsystem::GetLeftPosition() {
  // return left.GetPosition().GetValue().value();
  return 0.0;
}

double ClimbSubsystem::GetRightPosition() {
  // return right.GetPosition().GetValue().value();
  return 0.0;
}

double ClimbSubsystem::GetAngle() {
  double left = GetLeftPosition() / kTurnsPerDegree;
  double right = GetRightPosition() / kTurnsPerDegree;
  return (left + right) / 2;
}


bool ClimbSubsystem::IsAtTarget() {
  double target = position * kTurnsPerDegree;
  double leftPos = GetLeftPosition();
  double rightPos = GetRightPosition();
  bool leftAtTarget = leftPos > target - (kPositionDeadzone / 2) && leftPos < target + (kPositionDeadzone / 2);
  bool rightAtTarget = rightPos > target - (kPositionDeadzone / 2) && rightPos < target + (kPositionDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void ClimbSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  // left.GetConfigurator().Apply(updated, 50_ms);
  // right.GetConfigurator().Apply(updated, 50_ms);
}

void ClimbSubsystem::ConfigMotors() {
  // left.Config_kP(0, kP, 100);
  // right.Config_kP(0, kP, 100);
}