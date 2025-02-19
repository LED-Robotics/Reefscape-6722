// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace AlgaeConstants;
using namespace frc;

AlgaeSubsystem::AlgaeSubsystem()
  : wristMotor{kWristPort},
    intakeMotor{kIntakePort},
    wristEncoder{kEncoderPort} {
      ConfigIntake();
      ConfigWrist();
      SmartDashboard::PutNumber("Algae Position", wristAngle);
}

void AlgaeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Wrist Control
  SetTargetAngle(SmartDashboard::GetNumber("Algae Angle", wristAngle));
  SmartDashboard::PutNumber("Algae Actual", GetWristPosition());
  if(wristState == WristStates::kWristOff) {
    wristMotor.Set(0.0);
  } else if(wristState == WristStates::kWristPowerMode) {
    wristMotor.Set(wristPower);
  } else if(wristState == WristStates::kWristAngleMode) {
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.

    SmartDashboard::PutNumber("wristAngle", (GetWristPosition() / kTurnsPerDegree));  // print to Shuffleboard
    double targAngle = 0.0;
    targAngle = wristAngle;
    double feedForward = fabs(sin(wristAngle)) * kMaxFeedForward;
    SmartDashboard::PutNumber("Angle Target", targAngle);
    units::angle::turn_t posTarget{(targAngle - kWristStartOffset) * kTurnsPerDegree};
    wristMotor.SetControl(wristPosition
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true)
      .WithFeedForward(units::volt_t{feedForward}));
    //Intake Control
    if(intakeState == IntakeStates::kIntakeOff) {
      intakeMotor.Set(0.0);
    } else {
      intakeMotor.Set(intakePower);
    }
  }
}

void AlgaeSubsystem::IntakeOn() {
  intakeState = IntakeStates::kIntakePowerMode;
}

void AlgaeSubsystem::IntakeOff() {
  intakeState = IntakeStates::kIntakeOff;
}

void AlgaeSubsystem::SetIntakePower(double newPower) {
  intakePower = newPower;
}

double AlgaeSubsystem::GetIntakePower() {
  return intakePower;
}

void AlgaeSubsystem::SetIntakeState(int newState) {
  intakeState = newState;
}

int AlgaeSubsystem::GetIntakeState() {
  return intakeState;
}

void AlgaeSubsystem::SetIntakeBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  intakeMotor.GetConfigurator().Apply(updated, 50_ms);
}

void AlgaeSubsystem::ConfigIntake() {
  configs::TalonFXConfiguration algaeIntakeConfig{};

  algaeIntakeConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  algaeIntakeConfig.MotorOutput.PeakForwardDutyCycle = 1.0;

  intakeMotor.GetConfigurator().Apply(algaeIntakeConfig);
}

bool AlgaeSubsystem::IsAlgaeIndexed() {
  // Add when limit switch is added
  return false;
}

void AlgaeSubsystem::WristOn() {
  wristState = WristStates::kWristAngleMode;
}

void AlgaeSubsystem::WristOff() {
  wristState = WristStates::kWristOff;
}

void AlgaeSubsystem::SetWristPower(double newPower) {
  wristPower = newPower;
}

double AlgaeSubsystem::GetWristPower() {
  return wristPower;
}

void AlgaeSubsystem::SetTargetAngle(double newAngle) {
  wristAngle = newAngle;
  if(wristAngle < kWristDegreeMin) wristAngle = kWristDegreeMin;
  if(wristAngle > kWristDegreeMax) wristAngle = kWristDegreeMax;
}

double AlgaeSubsystem::GetAngle() {
  return GetWristPosition() / kTurnsPerDegree;
}

double AlgaeSubsystem::GetWristPosition() {
  return wristMotor.GetPosition().GetValueAsDouble();
}

bool AlgaeSubsystem::IsAtTarget() {
  double target = wristAngle * kTurnsPerDegree;
  double wristPos = GetWristPosition();
  bool wristAtTarget = wristPos > target - (kWristPositionDeadzone / 2) && wristPos < target + (kWristPositionDeadzone / 2);
  return wristAtTarget;
}

void AlgaeSubsystem::SetWristState(int newState) {
  wristState = newState;
}

int AlgaeSubsystem::GetWristState() {
  return wristState;
}

void AlgaeSubsystem::SetWristBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  wristMotor.GetConfigurator().Apply(updated, 50_ms);
}

void AlgaeSubsystem::ConfigWrist() {
  configs::TalonFXConfiguration algaeWristConfig{};

  algaeWristConfig.Slot0.kP = kPWrist;
  // algaeWristConfig.Slot0.kS = 0.28;
  // algaeWristConfig.Slot0.kV = 8.5;
  // algaeWristConfig.Slot0.kA = 3.0;
  // algaeWristConfig.Slot0.kP = 8.0;

  // algaeWristConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // algaeWristConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // algaeWristConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  algaeWristConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  algaeWristConfig.Feedback.RotorToSensorRatio = kWristRotorToGearbox;
  algaeWristConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  algaeWristConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  algaeWristConfig.Feedback.SensorToMechanismRatio = 1.0;
  algaeWristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  algaeWristConfig.Audio.AllowMusicDurDisable = true;

  algaeWristConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  wristMotor.GetConfigurator().Apply(algaeWristConfig);

  // configs::CANcoderConfiguration encoderConfig{};
  // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  // encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  // encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  // encoder.GetConfigurator().Apply(encoderConfig);
}