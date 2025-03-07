// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem/CoralSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CoralConstants;
using namespace frc;

CoralSubsystem::CoralSubsystem()
    : intake1{kIntake1Port},
      intake2{kIntake2Port},
      beamBreak{kBeamBreakPort} {
      ConfigMotors();
      previousVal = IsCoralIndexed();
}

void CoralSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Coral power control
  SmartDashboard::PutNumber("Coral Indexed", IsCoralIndexed());
  SmartDashboard::PutNumber("Coral Previous", previousVal);
  SmartDashboard::PutNumber("Coral powerOff", powerOff);
  if(state == CoralStates::kOff) {
    intake1.Set(0.0);
    intake2.Set(0.0);
  } else if(state == CoralStates::kPowerMode) {
    // power limiting 
    // if(intakeMotor.GetOutputCurrent() < kCurrentLimit && power > 0.0) intakeMotor.Set(power);
    intake1.Set(power);
    intake2.Set(power);
    // else intakeMotor.Set(0.0);
  } else if(state == CoralStates::kSensorMode) {
    bool indexed = IsCoralIndexed();
    if(indexed && !previousVal) {
      powerOff = true;
      trippedPower = fabs(power);
    } 
    SmartDashboard::PutNumber("tripped", trippedPower);

    if(powerOff && fabs(power) < trippedPower) {
      trippedPower = fabs(power);
    }
    SmartDashboard::PutNumber("power", fabs(power));
    SmartDashboard::PutNumber("Skibidi", fabs(power) - trippedPower);

    if(fabs(power) < 0.10 && powerOff) {
      powerOff = false;
      power = 0.0;
    }
    if(powerOff) {
      intake1.Set(0);
      intake2.Set(0);
    } else {
      intake1.Set(power * 0.3);
      intake2.Set(power * 0.3);
    }

    previousVal = indexed;
  }
}

void CoralSubsystem::Off() {
  state = CoralStates::kOff;
}

void CoralSubsystem::UsePowerMode() {
  state = CoralStates::kPowerMode;
}

void CoralSubsystem::SetPower(double newPower) {
  power = newPower;
  // if(power < kCoralDeadzone) power = 0.0;
}

double CoralSubsystem::GetPower() {
  return power;
}

int CoralSubsystem::GetState() {
  return state;
}

void CoralSubsystem::SetState(int newState) {
  state = newState;
}

void CoralSubsystem::SetBrakeMode(bool state) {
  /*signals::NeutralModeValue mode;*/
  /*if(state) mode = signals::NeutralModeValue::Brake;*/
  /*else mode = signals::NeutralModeValue::Coast;*/
  /*configs::MotorOutputConfigs updated;*/
  /*updated.WithNeutralMode(mode);*/
  /**/
  /*intakeMotor.GetConfigurator().Apply(updated, 50_ms);*/
}

void CoralSubsystem::ConfigMotors() {
  configs::TalonFXConfiguration coralConfig{};

  // coralConfig.Slot0.kP = kP;
  coralConfig.MotorOutput.Inverted = true;
  // coralConfig.Slot0.kS = 0.28;
  // coralConfig.Slot0.kV = 8.5;
  // coralConfig.Slot0.kA = 3.0;
  // coralConfig.Slot0.kP = 8.0;

  // coralConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // coralConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // coralConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // coralConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  // coralConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  // coralConfig.Feedback.RotorToSensorRatio = kRotorToGearbox;
  // coralConfig.Feedback.SensorToMechanismRatio = kRotorToGearbox * kGearboxToMechanism;
  // coralConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  // coralConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // coralConfig.Feedback.SensorToMechanismRatio = 1.0;
  // coralConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  // coralConfig.Audio.AllowMusicDurDisable = true;

  // coralConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  intake1.GetConfigurator().Apply(coralConfig);
  coralConfig.MotorOutput.Inverted = false;
  intake2.GetConfigurator().Apply(coralConfig);
}

bool CoralSubsystem::IsCoralIndexed() {
  return !beamBreak.Get();
}
