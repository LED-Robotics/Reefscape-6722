// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimbSubsystem/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimbConstants;
using namespace frc;

ClimbSubsystem::ClimbSubsystem()
  : climb{kClimbPort} {
      // climb.SetPosition(0.0_tr);
      SmartDashboard::PutNumber("Climb Angle", GetAngle().value());
      ConfigMotor();

      SetTargetAngle(kStartAngle);
}

void ClimbSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  //  Control
  /*SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Climb Angle", GetAngle().value())});*/
  SmartDashboard::PutNumber("Climb Actual", GetAngle().value());
  if(state == ClimbStates::kOff) {
    climb.Set(0.0);
  } else if(state == ClimbStates::kPowerMode) {
    climb.Set(power);
  } else if(state == ClimbStates::kAngleMode) {
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.

    SmartDashboard::PutNumber("climbTr", climb.GetPosition().GetValue().value());  // print to Shuffleboard
    SmartDashboard::PutNumber("climbAngle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(angle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("Climb Angle Target", angle.value());
    units::angle::turn_t posTarget{(angle - kStartAngle) / kTurnsPerDegree};
    SmartDashboard::PutNumber("wrTurnTarget", posTarget.value());
    climb.SetControl(positionController
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true)
      .WithFeedForward(units::volt_t{feedForward}));
  }
}

void ClimbSubsystem::On() {
  state = ClimbStates::kAngleMode;
}

void ClimbSubsystem::Off() {
  state = ClimbStates::kOff;
}

void ClimbSubsystem::SetPower(double newPower) {
  power = newPower;
}

double ClimbSubsystem::GetPower() {
  return power;
}

void ClimbSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if(angle < kDegreeMin) angle = kDegreeMin;
  if(angle > kDegreeMax) angle = kDegreeMax;
}

units::angle::degree_t ClimbSubsystem::GetAngle() {
  return units::angle::degree_t{(GetPosition() / kTurnsPerDegree)} + kStartAngle;
}

double ClimbSubsystem::GetPosition() {
  return climb.GetPosition().GetValueAsDouble();
}

bool ClimbSubsystem::IsAtTarget() {
  auto target = angle;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kAngleDeadzone / 2) && angle < target + (kAngleDeadzone / 2);
  return atTarget;
}

void ClimbSubsystem::SetState(int newState) {
  state = newState;
}

int ClimbSubsystem::GetState() {
  return state;
}

frc2::CommandPtr ClimbSubsystem::GetMoveCommand(units::angle::degree_t target) {
  /*return frc2::cmd::Sequence(*/
  /*    frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this}),*/
  /*    frc2::cmd::WaitUntil([this, target](){*/
  /*      return IsAtTarget();*/
  /*    }));*/
  return frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this});
}
void ClimbSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  climb.GetConfigurator().Apply(updated, 50_ms);
}

void ClimbSubsystem::ConfigMotor() {
  configs::TalonFXConfiguration climbConfig{};

  climbConfig.Slot0.kP = kP;
  climbConfig.MotorOutput.Inverted = true;
  // climbConfig.Slot0.kS = 0.28;
  // climbConfig.Slot0.kV = 8.5;
  // climbConfig.Slot0.kA = 3.0;
  // climbConfig.Slot0.kP = 8.0;

  // climbConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // climbConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // climbConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  climbConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  // climbConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  // climbConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  climbConfig.Feedback.SensorToMechanismRatio = kRotorToGearbox * kGearboxToMechanism;
  climbConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  climbConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  climbConfig.Feedback.SensorToMechanismRatio = 1.0;
  climbConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  climbConfig.Audio.AllowMusicDurDisable = true;

  climbConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  climb.GetConfigurator().Apply(climbConfig);

  // configs::CANcoderConfiguration encoderConfig{};
  // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  // encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  // encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  // encoder.GetConfigurator().Apply(encoderConfig);
}
