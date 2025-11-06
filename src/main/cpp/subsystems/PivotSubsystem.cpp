// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PivotSubsystem/PivotSubsystem.h"
#include "units/angle.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace PivotConstants;
using namespace frc;

PivotSubsystem::PivotSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&pivot}},
    pivot{kPivotPort},
    pivotEncoder{kEncoderPort} {
      ConfigPivot();
      SetTargetDegrees(ToDegrees(position));
      SetState(kPositionMode);

      SmartDashboard::PutNumber("SetPivotTarget", 90.0);
      SmartDashboard::PutNumber("NudgePivot", 0.0);  // print to Shuffleboard
}


units::angle::degree_t PivotSubsystem::ToDegrees(units::angle::turn_t turns) {
  return units::angle::degree_t{turns.value() / kTurnsPerDegree};
}

units::angle::turn_t PivotSubsystem::ToTurns(units::angle::degree_t degrees) {
  return units::angle::turn_t{degrees.value() * kTurnsPerDegree};
}

void PivotSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Pivot Control
  SetNudge(ToTurns(units::angle::degree_t{SmartDashboard::GetNumber("NudgePivot", 0.0)}));  // print to Shuffleboard

  double feedForward = fabs(sin(ToDegrees(position).value())) * kMaxFeedForward;
  SetTargetDegrees(units::angle::degree_t{SmartDashboard::GetNumber("SetPivotTarget", GetAngleDegrees().value())}, feedForward);
    // feed forwards should be a changing constant that increases as the pivot moves further. It should be a static amount of power to overcome gravity.
  SmartDashboard::PutNumber("PivotActual", GetAngleDegrees().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("PivotTr", GetPosition().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("PivotTarget", ToDegrees(position).value());
  SmartDashboard::PutNumber("PivotTargetTr", position.value());

  RunMotors();
}

void PivotSubsystem::SetTargetDegrees(units::angle::degree_t newAngle, double feedForward) {
  if(newAngle < kPivotDegreeMin) newAngle = kPivotDegreeMin;
  if(newAngle > kPivotDegreeMax) newAngle = kPivotDegreeMax;
  SetTargetPosition(ToTurns(newAngle), feedForward);
  SmartDashboard::PutNumber("SetPivotTarget", newAngle.value());
}

units::angle::degree_t PivotSubsystem::GetAngleDegrees() {
  return ToDegrees(GetPosition()) + kPivotStartAngle;
}

bool PivotSubsystem::IsAtTarget() {
  auto target = ToDegrees(position + nudge);
  auto angle = GetAngleDegrees();
  bool atTarget = angle > target - (kPivotAngleDeadzone / 2) && angle < target + (kPivotAngleDeadzone / 2);
  return atTarget;
}

frc2::CommandPtr PivotSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetDegrees(target);
      }, {this}),
      frc2::cmd::WaitUntil([this](){
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
  pivot.motor.GetConfigurator().Apply(updated, 50_ms);
}

void PivotSubsystem::ConfigPivot() {
  configs::TalonFXConfiguration pivotConfig{};

  pivotConfig.Slot0.kP = kPPivot;
  pivotConfig.Slot0.kD = kDPivot;
  pivotConfig.MotorOutput.Inverted = false;
  // pivotConfig.Slot0.kS = 0.28;
  // pivotConfig.Slot0.kV = 8.5;
  // pivotConfig.Slot0.kA = 3.0;
  // pivotConfig.Slot0.kP = 8.0;

  // pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // pivotConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // pivotConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // pivotConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  pivotConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::SyncCANcoder;
  pivotConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
  pivotConfig.Feedback.RotorToSensorRatio = kPivotRotorToGearbox;
  pivotConfig.Feedback.SensorToMechanismRatio = kPivotGearboxToMechanism;
  pivotConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  pivotConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // pivotConfig.Feedback.SensorToMechanismRatio = 1.0;
  pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  pivotConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  pivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  pivotConfig.Audio.AllowMusicDurDisable = true;

  pivotConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  pivot.motor.GetConfigurator().Apply(pivotConfig);

  configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  
  pivotEncoder.GetConfigurator().Apply(encoderConfig);
}
