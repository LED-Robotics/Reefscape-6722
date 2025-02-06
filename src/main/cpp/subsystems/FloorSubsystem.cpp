/*// Copyright (c) FIRST and other WPILib contributors.*/
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FloorSubsystem/FloorSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace FloorConstants;
using namespace frc;

FloorSubsystem::FloorSubsystem()
    : left{kLeftMotorPort},
    right{kRightMotorPort}
    // encoder{kEncoderPort} 
    {
      SmartDashboard::PutNumber("Floor Angle", angle.value());

      configs::TalonFXConfiguration floorConfig{};
      
      floorConfig.Slot0.kP = kP;
      // floorConfig.Slot0.kS = 0.28;
      // floorConfig.Slot0.kV = 8.5;
      // floorConfig.Slot0.kA = 3.0;
      // floorConfig.Slot0.kP = 8.0;

      // floorConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
      // floorConfig.MotionMagic.MotionMagicAcceleration = 2.0;
      // floorConfig.MotionMagic.MotionMagicJerk = 200.0;
      
      floorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
      floorConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
      floorConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
      floorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
      floorConfig.Feedback.SensorToMechanismRatio = 16.0;
      floorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
      floorConfig.Audio.AllowMusicDurDisable = true;
      
      floorConfig.MotorOutput.Inverted = true;
      // floorConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
      
      left.GetConfigurator().Apply(floorConfig);
      floorConfig.MotorOutput.Inverted = false;
      // floorConfig.DifferentialSensors.DifferentialSensorSource = signals::DifferentialSensorSourceValue::RemoteTalonFX_Diff;
      // floorConfig.DifferentialSensors.DifferentialTalonFXSensorID = kLeftMotorPort;
      right.GetConfigurator().Apply(floorConfig);

      // configs::CANcoderConfiguration encoderConfig{};
      // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
      // encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Angleitive;
      // encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
      // encoder.GetConfigurator().Apply(encoderConfig);
      SetTargetAngle(kFloorAngleMin);

}

void FloorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  /*SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Floor Angle", position.value())});*/
  SmartDashboard::PutNumber("Left Actual Floor", GetLeftAngle().value());
  SmartDashboard::PutNumber("Right Actual Floor", GetRightAngle().value());
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kAngleMode) {

  SmartDashboard::PutNumber("leftFloorTr", left.GetPosition().GetValue().value());
  SmartDashboard::PutNumber("rightFloorTr", right.GetPosition().GetValue().value());
  SmartDashboard::PutNumber("floorAngle", ((GetLeftAngle().value()) + (GetRightAngle().value())) / 2);  // print to Shuffleboard
    
    SmartDashboard::PutNumber("Angle Target", angle.value());
    units::angle::turn_t angleTarget{(angle.value() - kStartOffset) * kTurnsPerDegree};
    left.SetControl(angleController
      .WithPosition(units::angle::turn_t{angleTarget})
      .WithEnableFOC(true));
    right.SetControl(angleController
      .WithPosition(units::angle::turn_t{angleTarget})
      .WithEnableFOC(true));

    // Test Motion Magic
    // left.SetControl(position
    //   .WithAngle(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
    // right.SetControl(position
    //   .WithAngle(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
  }
}

void FloorSubsystem::Off() {
  state = kOff;
}

void FloorSubsystem::On() {
  state = kPowerMode;
}

void FloorSubsystem::SetPower(double newPower) {
  power = newPower;
}

void FloorSubsystem::SetState(int newState) {
  state = newState;
}

int FloorSubsystem::GetState() {
  return state;
}

units::angle::degree_t FloorSubsystem::GetLeftAngle() {
  return units::angle::degree_t{left.GetPosition().GetValue().value() / kTurnsPerDegree};
}

units::angle::degree_t FloorSubsystem::GetRightAngle() {
  return units::angle::degree_t{right.GetPosition().GetValue().value() / kTurnsPerDegree};
}

units::angle::degree_t FloorSubsystem::GetAngle() {
  auto left = GetLeftAngle();
  auto right = GetRightAngle();
  return (left + right) / 2;
}

void FloorSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if(angle < kFloorAngleMin) angle = kFloorAngleMin;
  if(angle > kFloorAngleMax) angle = kFloorAngleMax;
}

bool FloorSubsystem::IsAtTarget() {
  auto target = angle;
  auto leftAngle = GetLeftAngle();
  auto rightAngle = GetRightAngle();
  bool leftAtTarget = leftAngle > target - (kAngleDeadzone / 2) && leftAngle < target + (kAngleDeadzone / 2);
  bool rightAtTarget = rightAngle > target - (kAngleDeadzone / 2) && rightAngle < target + (kAngleDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void FloorSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  left.GetConfigurator().Apply(updated, 50_ms);
  right.GetConfigurator().Apply(updated, 50_ms);
}

void FloorSubsystem::ConfigMotors() {
  // left.Config_kP(0, kP, 100);
  // right.Config_kP(0, kP, 100);
}

frc2::CommandPtr FloorSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
}
