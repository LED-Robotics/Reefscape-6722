// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FunnelSubsystem/FunnelSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace FunnelConstants;
using namespace frc;

FunnelSubsystem::FunnelSubsystem()
  : funnel{kFunnelMotorPort, "canCan"} {
      SmartDashboard::PutNumber("Funnel Angle", funnelAngle.value());
      ConfigMotor();

      SetTargetAngle(kFunnelAngleMin);
}

void FunnelSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // funnel Control
  SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Funnel Angle", funnelAngle.value())});
  SmartDashboard::PutNumber("Funnel Actual", GetAngle().value());
  if(funnelState == FunnelStates::kFunnelOff) {
    funnel.Set(0.0);
  } else if(funnelState == FunnelStates::kFunnelPowerMode) {
    funnel.Set(funnelPower);
  } else if(funnelState == FunnelStates::kFunnelAngleMode) {
    // feed forwards should be a changing constant that increases as the funnel moves further. It should be a static amount of power to overcome gravity.

    SmartDashboard::PutNumber("funnelTr", funnel.GetPosition().GetValue().value());  // print to Shuffleboard

    SmartDashboard::PutNumber("funnelAngle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(funnelAngle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("Angle Target", funnelAngle.value());
    
    units::angle::turn_t posTarget{(funnelAngle.value() - kStartOffset) * kTurnsPerDegree};
    funnel.SetControl(positionController
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true)
      .WithFeedForward(units::volt_t{feedForward}));
  }
}

void FunnelSubsystem::On() {
  funnelState = FunnelStates::kFunnelAngleMode;
}

void FunnelSubsystem::Off() {
  funnelState = FunnelStates::kFunnelOff;
}

void FunnelSubsystem::SetPower(double newPower) {
  funnelPower = newPower;
}

double FunnelSubsystem::GetPower() {
  return funnelPower;
}

void FunnelSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  funnelAngle = newAngle;
  if(funnelAngle < kFunnelAngleMin) funnelAngle = kFunnelAngleMin;
  if(funnelAngle > kFunnelAngleMax) funnelAngle = kFunnelAngleMax;
}

units::angle::degree_t FunnelSubsystem::GetAngle() {
  return units::angle::degree_t{GetPosition() / kTurnsPerDegree};
}

double FunnelSubsystem::GetPosition() {
  return funnel.GetPosition().GetValueAsDouble();
}

bool FunnelSubsystem::IsAtTarget() {
  auto target = funnelAngle;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kAngleDeadzone / 2) && angle < target + (kAngleDeadzone / 2);
  return atTarget;
}

void FunnelSubsystem::SetState(int newState) {
  funnelState = newState;
}

int FunnelSubsystem::GetState() {
  return funnelState;
}

void FunnelSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  funnel.GetConfigurator().Apply(updated, 50_ms);
}

frc2::CommandPtr FunnelSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
}

void FunnelSubsystem::ConfigMotor() {
  configs::TalonFXConfiguration funnelConfig{};

  funnelConfig.Slot0.kP = kP;
  // funnelConfig.Slot0.kS = 0.28;
  // funnelConfig.Slot0.kV = 8.5;
  // funnelConfig.Slot0.kA = 3.0;
  // funnelConfig.Slot0.kP = 8.0;

  // funnelConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // funnelConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // funnelConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  funnelConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  funnelConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  funnelConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  funnelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  funnelConfig.Feedback.SensorToMechanismRatio = 1.0;
  funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  funnelConfig.Audio.AllowMusicDurDisable = true;
  
  funnel.GetConfigurator().Apply(funnelConfig);
}