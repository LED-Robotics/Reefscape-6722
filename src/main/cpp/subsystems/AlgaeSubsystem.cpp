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
      /*wristMotor.SetPosition(0.0_tr);*/
      SmartDashboard::PutNumber("Algae Angle", 80.0);
      ConfigIntake();
      ConfigWrist();

      /*SetTargetAngle(kWristStartAngle);*/

}

void AlgaeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Wrist Control
  /*SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Algae Angle", GetAngle().value())});*/
  SmartDashboard::PutNumber("Algae Actual", GetAngle().value());
  if(wristState == WristStates::kWristOff) {
    wristMotor.Set(0.0);
  } else if(wristState == WristStates::kWristPowerMode) {
    wristMotor.Set(wristPower);
  } else if(wristState == WristStates::kWristAngleMode) {
    // feed forwards should be a changing constant that increases as the wrist moves further. It should be a static amount of power to overcome gravity.

    SmartDashboard::PutNumber("algaeWristTr", wristMotor.GetPosition().GetValue().value());  // print to Shuffleboard
    SmartDashboard::PutNumber("algaeAngle", GetAngle().value());  // print to Shuffleboard
    double feedForward = fabs(sin(wristAngle.value())) * kMaxFeedForward;
    SmartDashboard::PutNumber("Angle Target", wristAngle.value());
    units::angle::turn_t posTarget{(wristAngle - kWristStartAngle) / kTurnsPerDegree};
    SmartDashboard::PutNumber("wrTurnTarget", posTarget.value());
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
  algaeIntakeConfig.MotorOutput.Inverted = true;

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

void AlgaeSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  wristAngle = newAngle;
  if(wristAngle < kWristDegreeMin) wristAngle = kWristDegreeMin;
  if(wristAngle > kWristDegreeMax) wristAngle = kWristDegreeMax;
}

units::angle::degree_t AlgaeSubsystem::GetAngle() {
  return units::angle::degree_t{(GetWristPosition() / kTurnsPerDegree)} + kWristStartAngle;
}

double AlgaeSubsystem::GetWristPosition() {
  return wristMotor.GetPosition().GetValueAsDouble();
}

bool AlgaeSubsystem::IsAtTarget() {
  auto target = wristAngle;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kWristAngleDeadzone / 2) && angle < target + (kWristAngleDeadzone / 2);
  return atTarget;
}

void AlgaeSubsystem::SetWristState(int newState) {
  wristState = newState;
}

int AlgaeSubsystem::GetWristState() {
  return wristState;
}

frc2::CommandPtr AlgaeSubsystem::GetMoveCommand(units::angle::degree_t target) {
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
  algaeWristConfig.MotorOutput.Inverted = true;
  // algaeWristConfig.Slot0.kS = 0.28;
  // algaeWristConfig.Slot0.kV = 8.5;
  // algaeWristConfig.Slot0.kA = 3.0;
  // algaeWristConfig.Slot0.kP = 8.0;

  // algaeWristConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // algaeWristConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // algaeWristConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  algaeWristConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  // algaeWristConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  // algaeWristConfig.Feedback.RotorToSensorRatio = kWristRotorToGearbox;
  algaeWristConfig.Feedback.SensorToMechanismRatio = kWristRotorToGearbox * kWristGearboxToMechanism;
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
