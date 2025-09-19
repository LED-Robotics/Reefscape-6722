// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"
#include "led_libraries/PositionalSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CascadeConstants;
using namespace frc;

CascadeSubsystem::CascadeSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&leftController, &rightController}},
  left{kLeftMotorPort},
  right{kRightMotorPort}
  {
    ConfigMotors();
    SetTargetMeters(kStartPosition);
    SetState(kPositionMode);

    SmartDashboard::PutNumber("SetCascadeTarget", position.value());
    SmartDashboard::PutNumber("NudgeCascade", 0.0);  // print to Shuffleboard
}


units::length::meter_t CascadeSubsystem::ToMeters(units::angle::turn_t turns) {
  return units::length::meter_t{turns.value() / kTurnsPerMeter};
}

units::angle::turn_t CascadeSubsystem::ToTurns(units::length::meter_t meters) {
  return units::angle::turn_t{meters.value() * kTurnsPerMeter};
}


void CascadeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetNudge(ToTurns(units::length::meter_t{SmartDashboard::GetNumber("NudgeCascade", 0.0)}));  // print to Shuffleboard
  SetTargetMeters(units::length::meter_t{SmartDashboard::GetNumber("SetCascadeTarget", ToMeters(position).value())});

  SmartDashboard::PutNumber("CascadeActual", (ToMeters(GetPosition()) + kStartPosition).value());  // print to Shuffleboard
  SmartDashboard::PutNumber("CascadeActualTr", GetPosition().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("CascadeTarget", (ToMeters(position) + kStartPosition).value());
  SmartDashboard::PutNumber("CascadeTargetTr", position.value() + nudge.value());

  
  RunMotors();
}

units::length::meter_t CascadeSubsystem::GetPositionMeters() {
  return ToMeters(GetPosition());
}

void CascadeSubsystem::SetTargetMeters(units::length::meter_t newPosition) {
  if(newPosition < kCascadeMeterMin) newPosition = kCascadeMeterMin;
  if(newPosition > kCascadeMeterMax) newPosition = kCascadeMeterMax;
  newPosition -= kStartPosition;
  SetTargetPosition(ToTurns(newPosition));
  newPosition += kStartPosition;
  SmartDashboard::PutNumber("SetCascadeTarget", newPosition.value());
}

bool CascadeSubsystem::IsAtTarget() {
  auto target = ToMeters(position + nudge);
  auto pos = GetPositionMeters();
  
  bool atTarget = pos > target - (kPositionDeadzone / 2) && pos < target + (kPositionDeadzone / 2);
  return atTarget;
}

void CascadeSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  left.GetConfigurator().Apply(updated, 50_ms);
  right.GetConfigurator().Apply(updated, 50_ms);
}

void CascadeSubsystem::ConfigMotors() {
  configs::TalonFXConfiguration cascadeConfig{};
  
  cascadeConfig.Slot0.kP = kP;
  cascadeConfig.Slot0.kD = kD;
  cascadeConfig.Slot0.kG = kG;
  // cascadeConfig.Slot0.kS = 0.28;
  // cascadeConfig.Slot0.kV = 8.5;
  // cascadeConfig.Slot0.kA = 3.0;
  // cascadeConfig.Slot0.kP = 8.0;

  // cascadeConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // cascadeConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // cascadeConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  cascadeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  cascadeConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  cascadeConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  cascadeConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  cascadeConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  cascadeConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  cascadeConfig.Feedback.SensorToMechanismRatio = 16.0;
  cascadeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  cascadeConfig.Audio.AllowMusicDurDisable = true;
  
  cascadeConfig.MotorOutput.Inverted = false;
  // cascadeConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  left.GetConfigurator().Apply(cascadeConfig);
  // cascadeConfig.DifferentialSensors.DifferentialSensorSource = signals::DifferentialSensorSourceValue::RemoteTalonFX_Diff;
  // cascadeConfig.DifferentialSensors.DifferentialTalonFXSensorID = kLeftMotorPort;
  cascadeConfig.MotorOutput.Inverted = true;

  right.GetConfigurator().Apply(cascadeConfig);

  // configs::CANcoderConfiguration encoderConfig{};
  // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  // encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  // encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  // encoder.GetConfigurator().Apply(encoderConfig);
}

frc2::CommandPtr CascadeSubsystem::GetMoveCommand(units::length::meter_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetMeters(target);
      }, {this}),
      frc2::cmd::WaitUntil([this](){
        return IsAtTarget();
      }));
}

// For debug
units::length::meter_t CascadeSubsystem::GetLeftPosition() {
  auto base = units::length::meter_t{left.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}

units::length::meter_t CascadeSubsystem::GetRightPosition() {
  auto base = units::length::meter_t{right.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  return base + kStartPosition;
}
