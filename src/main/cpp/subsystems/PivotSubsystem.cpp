// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PivotSubsystem/PivotSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace PivotConstants;
using namespace frc;

PivotSubsystem::PivotSubsystem()
  : pivot{kPivotPort},
    pivotEncoder{kEncoderPort} {
      /*pivot.SetPosition(0.0_tr);*/
      SmartDashboard::PutNumber("Pivot Angle", 90.0);
      ConfigPivot();

      SetTargetAngle(angle);

}

void PivotSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Pivot Control
  SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Pivot Angle", GetAngle().value())});
  SmartDashboard::PutNumber("Pivot Actual", GetAngle().value());
  if(state == PivotStates::kPivotOff) {
    pivot.Set(0.0);
  } else if(state == PivotStates::kPivotPowerMode) {
    pivot.Set(power);
  } else if(state == PivotStates::kPivotAngleMode) {
    // feed forwards should be a changing constant that increases as the pivot moves further. It should be a static amount of power to overcome gravity.

    SmartDashboard::PutNumber("pivotPivotTr", pivot.GetPosition().GetValue().value());  // print to Shuffleboard
    SmartDashboard::PutNumber("angle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(angle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("Angle Target", angle.value());
    units::angle::turn_t posTarget{(angle - kPivotStartAngle).value() * kTurnsPerDegree};
    SmartDashboard::PutNumber("wrTurnTarget", posTarget.value());
    pivot.SetControl(pivotPosition
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true));
      /*.WithFeedForward(units::volt_t{feedForward}));*/
  }
}

void PivotSubsystem::PivotOn() {
  state = PivotStates::kPivotAngleMode;
}

void PivotSubsystem::PivotOff() {
  state = PivotStates::kPivotOff;
}

void PivotSubsystem::SetPivotPower(double newPower) {
  power = newPower;
}

double PivotSubsystem::GetPivotPower() {
  return power;
}

void PivotSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if(angle < kPivotDegreeMin) angle = kPivotDegreeMin;
  if(angle > kPivotDegreeMax) angle = kPivotDegreeMax;
  SmartDashboard::PutNumber("Pivot Angle", angle.value());
}

units::angle::degree_t PivotSubsystem::GetAngle() {
  return units::angle::degree_t{(GetPivotPosition() / kTurnsPerDegree)} + kPivotStartAngle;
}

double PivotSubsystem::GetPivotPosition() {
  return pivot.GetPosition().GetValueAsDouble();
}

bool PivotSubsystem::IsAtTarget() {
  auto target = angle;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kPivotAngleDeadzone / 2) && angle < target + (kPivotAngleDeadzone / 2);
  return atTarget;
}

void PivotSubsystem::SetPivotState(int newState) {
  state = newState;
}

int PivotSubsystem::GetPivotState() {
  return state;
}

frc2::CommandPtr PivotSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
  /*return frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this});*/
}
void PivotSubsystem::SetPivotBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  pivot.GetConfigurator().Apply(updated, 50_ms);
}

void PivotSubsystem::ConfigPivot() {
  configs::TalonFXConfiguration pivotPivotConfig{};

  pivotPivotConfig.Slot0.kP = kPPivot;
  pivotPivotConfig.MotorOutput.Inverted = false;
  // pivotPivotConfig.Slot0.kS = 0.28;
  // pivotPivotConfig.Slot0.kV = 8.5;
  // pivotPivotConfig.Slot0.kA = 3.0;
  // pivotPivotConfig.Slot0.kP = 8.0;

  // pivotPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // pivotPivotConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // pivotPivotConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // pivotPivotConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  pivotPivotConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  pivotPivotConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  pivotPivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
  pivotPivotConfig.Feedback.RotorToSensorRatio = kPivotRotorToGearbox;
  pivotPivotConfig.Feedback.SensorToMechanismRatio = kPivotGearboxToMechanism;
  pivotPivotConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  pivotPivotConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // pivotPivotConfig.Feedback.SensorToMechanismRatio = 1.0;
  pivotPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  pivotPivotConfig.Audio.AllowMusicDurDisable = true;

  pivotPivotConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  pivot.GetConfigurator().Apply(pivotPivotConfig);

  configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  
  pivotEncoder.GetConfigurator().Apply(encoderConfig);
}
