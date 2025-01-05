// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem(std::function<units::length::meter_t()> getDistFunc, int *targetRef, Orchestra *orcRef)
    : left{kLeftMotorPort},
    right{kRightMotorPort},
    // indexer{kIndexerPort},
    getTargetDistance{getDistFunc},
    indexer{kIndexerPort, SparkMax::MotorType::kBrushless},
    color{frc::I2C::Port::kMXP, 0x39},
    shooterBeamBreakReceiver{kShooterBeamBreakReceiverPort} {
      orca = orcRef;
      orca->AddInstrument(left);
      orca->AddInstrument(right);
      // orca->AddInstrument(indexer);
      target = targetRef;
      color.setGain(60);
      // left.SetInverted(true);
      indexerConfig
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
      indexerConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .OutputRange(-1.0, 1.0)
        .Pid(kPIndexer, 0.0, 0.0);
      /** Old 2024 Code
       * indexerController.SetP(kPIndexer);
       * indexerEncoder.SetPosition(kIndexerOffset / kIndexerReduction);
       * indexerController.SetOutputRange(-1.0, 1.0); 
       */
      indexer.Configure(indexerConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
      indexer.GetEncoder().SetPosition(kIndexerOffset / kIndexerReduction);
      SmartDashboard::PutNumber("Shooter RPM", 0.0);
      SmartDashboard::PutNumber("Top Offset", 1.0);
      SmartDashboard::PutNumber("Bottom Offset", 0.85);
      SmartDashboard::PutBoolean("autoIndex", true);
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here

  velocity = SmartDashboard::GetNumber("Shooter RPM", 0.0) / 60.0;
  // SmartDashboard::PutNumber("Actual RPM", GetVel() * 60.0);
  SmartDashboard::PutNumber("Left RPM", GetLeftVelocity() * 60.0);
  SmartDashboard::PutNumber("Right RPM", GetRightVelocity() * 60.0);
  double topOffset = SmartDashboard::GetNumber("Top Offset", 1.0);
  double bottomOffset = SmartDashboard::GetNumber("Bottom Offset", 0.85);
  SmartDashboard::PutBoolean("Beam Break", IsNoteIndexed());

  if(state == kOff) {
    left.Set(0.0);
    right.Set(0.0);
  } else if(state == kPowerMode) {
    left.Set(power);
    right.Set(power);
  } else if(state == kRpmMode) {
    units::angular_velocity::turns_per_second_t targetVel{0.0_tps};
    
    if(*target == GlobalConstants::kArbitrary) {
      targetVel = units::angular_velocity::turns_per_second_t{velocity};
    } else if(*target == GlobalConstants::kAmp) {
      topOffset = 0.4;
      bottomOffset = 1.0;
      targetVel = units::angular_velocity::turns_per_second_t{kAmpTarget[0][1] / 60};
    } 
    else {
      RpmFromDistance(getTargetDistance());
      targetVel = autoVel;
      switch(*target) {
        case GlobalConstants::kSpeaker:
          topOffset = 1.0;
          bottomOffset = 0.85;
          SmartDashboard::PutNumber("Top Offset", topOffset);
          SmartDashboard::PutNumber("Bottom Offset", bottomOffset);
          break;
        case GlobalConstants::kAmp:
          topOffset = 0.4;
          bottomOffset = 1.0;
          SmartDashboard::PutNumber("Top Offset", topOffset);
          SmartDashboard::PutNumber("Bottom Offset", bottomOffset);
          break;
        case GlobalConstants::kStage:
          topOffset = 1.0;
          bottomOffset = 1.0;
          SmartDashboard::PutNumber("Top Offset", topOffset);
          SmartDashboard::PutNumber("Bottom Offset", bottomOffset);
          break;
        default:
          topOffset = 1.0;
          bottomOffset = 0.85;
          SmartDashboard::PutNumber("Top Offset", topOffset);
          SmartDashboard::PutNumber("Bottom Offset", bottomOffset);
          break;
      }
    }
    left.SetControl(ctreVelocity
      .WithVelocity(targetVel * topOffset));
    right.SetControl(ctreVelocity
      .WithVelocity(targetVel * bottomOffset));
  }

  bool noteIndexed = IsNoteIndexed();
  SmartDashboard::PutBoolean("noteIndexed", noteIndexed);
  indexerAutoEnabled = SmartDashboard::GetBoolean("autoIndex", false);
  if(indexerAutoEnabled && noteIndexed) {
    indexerPosition = kIndexerPrimed;
  }

  // Indexer Control
  /** Old 2024 Code
   * indexerController.SetReference(indexerPosition / kIndexerReduction, SparkLowLevel::ControlType::kPosition); 
  */ 
  indexer.GetClosedLoopController().SetReference(indexerPosition / kIndexerReduction, SparkLowLevel::ControlType::kPosition);
  // indexer.SetControl(ctrePosition
  //   .WithPosition(units::angle::turn_t{indexerPosition / kIndexerReduction}));
}

void ShooterSubsystem::Off() {
  state = kOff;
}

void ShooterSubsystem::On() {
  state = kPowerMode;
}

void ShooterSubsystem::SetPower(double newPower) {
  power = newPower;
}

void ShooterSubsystem::SetState(int newState) {
  state = newState;
}

int ShooterSubsystem::GetState() {
  return state;
}

double ShooterSubsystem::GetLeftVelocity() {
  return left.GetVelocity().GetValue().value();
}

double ShooterSubsystem::GetRightVelocity() {
  return right.GetVelocity().GetValue().value();
}

double ShooterSubsystem::GetVel() {
  double left = GetLeftVelocity();
  double right = GetRightVelocity();
  return (left + right) / 2;
}

void ShooterSubsystem::SetTargetVelocity(double newVel) {
  velocity = newVel;
}

bool ShooterSubsystem::IsAtTarget() {
  double leftPos = GetLeftVelocity();
  double rightPos = GetRightVelocity();
  bool leftAtTarget = leftPos > velocity - (kVelDeadzone / 2) && leftPos < velocity + (kVelDeadzone / 2);
  bool rightAtTarget = rightPos > velocity - (kVelDeadzone / 2) && rightPos < velocity + (kVelDeadzone / 2);
  return leftAtTarget && rightAtTarget;
}

void ShooterSubsystem::RpmFromDistance(units::length::meter_t distance) {
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
      autoVel = units::angular_velocity::turns_per_second_t{53};
      return;
  }
  units::angular_velocity::turns_per_second_t rpm{0.0_tps};
  unsigned int size = 0;
  double temp = 0.0;
  do {
    temp = targetVals[size++][0];
  } while(temp != 69.0);
  for(unsigned int i = 0; i < size; i++) {
    if(dist >= targetVals[i][0]) {
      if(i + 1 < size) {
      }
      // greater than previous point
      if(dist < targetVals[i + 1][0] && i + 1 < size) {
      // less than next point
        double slope = (targetVals[i + 1][1] - targetVals[i][1]) / (targetVals[i + 1][0] - targetVals[i][0]);
        autoVel = units::angular_velocity::turns_per_second_t{(dist - targetVals[i][0]) * slope + targetVals[i][1]};
        autoVel /= 60.0;
        break;
      }
    } else {
      autoVel = units::angular_velocity::turns_per_second_t{targetVals[i][1]};
    }
  }
}

void ShooterSubsystem::OffsetIndexer(double offset) {
  indexer.GetEncoder().SetPosition(indexer.GetEncoder().GetPosition() - (offset / kIndexerReduction));
  // indexer.SetPosition(units::angle::turn_t{indexer.GetPosition().GetValueAsDouble() - (offset / kIndexerReduction)});
}

void ShooterSubsystem::SetIndexerPosition(double newPos) {
  indexerPosition = newPos;
}

bool ShooterSubsystem::IndexerIsAtTarget() {
  double current = indexer.GetEncoder().GetPosition() * kIndexerReduction;
  // double current = indexer.GetPosition().GetValueAsDouble() * kIndexerReduction;
  return  current > indexerPosition - kIndexerDeadzone / 2 &&
  current < indexerPosition + kIndexerDeadzone / 2;
}

bool ShooterSubsystem::GetIndexerAuto() {
  return indexerAutoEnabled;
}

void ShooterSubsystem::SetIndexerAuto(bool state) {
  SmartDashboard::PutBoolean("autoIndex", state);
  indexerAutoEnabled = state;
}

bool ShooterSubsystem::IsNoteIndexed() {
  // double red = color.getRed() - ((color.getRed() + color.getBlue() + color.getGreen()) / 3);
  // red += 1000.0;
  // SmartDashboard::PutNumber("Red Value", red);
  // if(color.getRed() == 0) return false;
  // return  red > kRedThreshold;
  return !shooterBeamBreakReceiver.Get();
}


void ShooterSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  left.GetConfigurator().Apply(updated, 50_ms);
  right.GetConfigurator().Apply(updated, 50_ms);
}

void ShooterSubsystem::ConfigMotors() {
  // left.Config_kP(0, kP, 100);
  // right.Config_kP(0, kP, 100);
}