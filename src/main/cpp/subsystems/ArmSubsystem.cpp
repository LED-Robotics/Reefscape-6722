// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ArmConstants;
using namespace frc;

ArmSubsystem::ArmSubsystem(std::function<units::length::meter_t()> getDistFunc, int *targetRef, Orchestra *orcRef)
    : left{kLeftMotorPort},
    right{kRightMotorPort},
    encoder{kEncoderPort},
    getTargetDistance{getDistFunc} {
      orca = orcRef;
      orca->AddInstrument(left);
      orca->AddInstrument(right);
      target = targetRef;
      SmartDashboard::PutNumber("Arm Position", angle);

      configs::TalonFXConfiguration armConfig{};
      
      armConfig.Slot0.kP = kP;
      // armConfig.Slot0.kS = 0.28;
      // armConfig.Slot0.kV = 8.5;
      // armConfig.Slot0.kA = 3.0;
      // armConfig.Slot0.kP = 8.0;

      // armConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
      // armConfig.MotionMagic.MotionMagicAcceleration = 2.0;
      // armConfig.MotionMagic.MotionMagicJerk = 200.0;
      
      armConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
      armConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
      armConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
      armConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
      armConfig.Feedback.SensorToMechanismRatio = 1.0;
      armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
      armConfig.Audio.AllowMusicDurDisable = true;

      armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
<<<<<<< Updated upstream
=======
      
>>>>>>> Stashed changes
      left.GetConfigurator().Apply(armConfig);
      armConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
      right.GetConfigurator().Apply(armConfig);

      configs::CANcoderConfiguration encoderConfig{};
<<<<<<< Updated upstream
      encoderConfig.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
      encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
      encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;

      encoder.GetConfigurator().Apply(encoderConfig);


=======
      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
      encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
      encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
      encoder.GetConfigurator().Apply(encoderConfig);

>>>>>>> Stashed changes
}

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  SetTargetAngle(SmartDashboard::GetNumber("Arm Position", angle));
  SmartDashboard::PutNumber("Left Actual Arm", GetLeftPosition());
  SmartDashboard::PutNumber("Right Actual Arm", GetRightPosition());
  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kAngleMode) {
    // feed forwards should be a changing constant that increases as the arm moves further. It should be a static amount of power to overcome gravity.

    // SmartDashboard::PutNumber("armAngle", ((GetLeftPosition() / kCountsPerDegree) + (GetRightPosition() / kCountsPerDegree)) / 2);  // print to Shuffleboard
    double targAngle = 0.0;
    if(*target == GlobalConstants::kArbitrary) {
      targAngle = angle;
    } else {
      AngleFromDistance(getTargetDistance());
      targAngle = autoAngle;
    }
    double feedForward = sin(angle) * kMaxFeedForward;
    SmartDashboard::PutNumber("Angle Target", targAngle);
    units::angle::turn_t posTarget{(targAngle - kStartOffset) * kTurnsPerDegree};
    left.SetControl(position
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true)
      .WithFeedForward(units::volt_t{feedForward}));
    right.SetControl(position
      .WithPosition(units::angle::turn_t{posTarget})
      .WithEnableFOC(true)
      .WithFeedForward(units::volt_t{feedForward}));

    // Test Motion Magic
    // left.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
    // right.SetControl(position
    //   .WithPosition(units::angle::turn_t{posTarget})
    //   .WithEnableFOC(true));
  }
}

void ArmSubsystem::Off() {
  state = kOff;
}

void ArmSubsystem::On() {
  state = kPowerMode;
}

void ArmSubsystem::SetPower(double newPower) {
  power = newPower;
}

void ArmSubsystem::SetState(int newState) {
  state = newState;
}

int ArmSubsystem::GetState() {
  return state;
}

double ArmSubsystem::GetLeftPosition() {
  return left.GetPosition().GetValue().value();
}

double ArmSubsystem::GetRightPosition() {
  return right.GetPosition().GetValue().value();
}

double ArmSubsystem::GetAngle() {
  double left = GetLeftPosition() / kTurnsPerDegree;
  double right = GetRightPosition() / kTurnsPerDegree;
  return (left + right) / 2;
}

void ArmSubsystem::SetTargetAngle(double newAngle) {
  angle = newAngle;
  if(angle < kArmDegreeMin) angle = kArmDegreeMin;
  if(angle > kArmDegreeMax) angle = kArmDegreeMax;
}

bool ArmSubsystem::IsAtTarget() {
  double target = angle * kTurnsPerDegree;
  double leftPos = GetLeftPosition();
  double rightPos = GetRightPosition();
  bool leftAtTarget = leftPos > target - (kPositionDeadzone / 2) && leftPos < target + (kPositionDeadzone / 2);
  bool rightAtTarget = rightPos > target - (kPositionDeadzone / 2) && rightPos < target + (kPositionDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void ArmSubsystem::AngleFromDistance(units::length::meter_t distance) {
  const double (*targetVals)[2] = kSpeakerTarget;
  double dist = distance.value();
  switch(*target) {
    case GlobalConstants::kSpeaker:
      targetVals = kSpeakerTarget;
      break;
    case GlobalConstants::kAmp:
      targetVals = kAmpTarget;
      break;
    case GlobalConstants::kSource:
      targetVals = kSourceTarget;
      break;
    case GlobalConstants::kStage:
      targetVals = kStageTarget;
      break;
    default:
      autoAngle = 50.0;
      return;
  }
  unsigned int size = 0;
  double temp = 0.0;
  do {
    temp = targetVals[size++][0];
  } while(temp != 69.0);
  for(unsigned int i = 0; i < size; i++) {
    if(dist >= targetVals[i][0]) {
      // greater than previous point
      if(dist < targetVals[i + 1][0] && i + 1 < size) {
      // less than next point
        double slope = (targetVals[i + 1][1] - targetVals[i][1]) / (targetVals[i + 1][0] - targetVals[i][0]);
        autoAngle = (dist - targetVals[i][0]) * slope + targetVals[i][1];
        break;
      }
    } else {
      autoAngle = targetVals[i][1];
    }
  }
}

void ArmSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  left.GetConfigurator().Apply(updated, 50_ms);
  right.GetConfigurator().Apply(updated, 50_ms);
}

void ArmSubsystem::ConfigMotors() {
  // left.Config_kP(0, kP, 100);
  // right.Config_kP(0, kP, 100);
}