// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightSubsystem.h"

#include <iostream>

using namespace LimelightConstants;
using namespace frc;

LimelightSubsystem::LimelightSubsystem(std::string_view targetTable) {
  // Implementation of subsystem constructor goes here
  tableName = targetTable;
  table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);
  SetPipeline(1.0);
  SetLED(kUsePipeline);
}

void LimelightSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);
  // Store Limelight network table values in the Subsystem
  targetFound = table->GetNumber("tv", 0.0) == 1.0;
  if(targetFound) {
    targetXOffset = table->GetNumber("tx", 0.0);
    targetYOffset = table->GetNumber("ty", 0.0);
    targetArea = table->GetNumber("ta", 0.0);
    targetSkew = table->GetNumber("ts", 0.0);
  }
  // botPose = table->GetNumberArray("botpose",std::vector<double>(6));
  targetPose = table->GetNumberArray("targetpose_robotspace",std::vector<double>(6));

  // Old code to use botpose translated based on alliance
  botPose = table->GetNumberArray("botpose_wpiblue",std::vector<double>(6));

  // auto alliance = frc::DriverStation::GetAlliance();
  // if(alliance == frc::DriverStation::Alliance::kBlue) {
  //   botPose = table->GetNumberArray("botpose_wpiblue",std::vector<double>(6));
  // } else if(alliance == frc::DriverStation::Alliance::kRed) {
  //   botPose = table->GetNumberArray("botpose_wpired",std::vector<double>(6));
  // }
}

double LimelightSubsystem::GetPipeline() {
  return table->GetNumber("getpipe", 0.0);
}

void LimelightSubsystem::SetPipeline(double pipeline) {
  table->PutNumber("pipeline", pipeline);
}

void LimelightSubsystem::SetLED(int state) {
  switch(state) {
    case kOff:
      table->PutNumber("ledMode", 1.0);
      break;
    case kOn:
      table->PutNumber("ledMode", 3.0);
      break;
    case kBlink:
      table->PutNumber("ledMode", 2.0);
      break;
    case kUsePipeline:
      table->PutNumber("ledMode", 0.0);
      break;
  }
}

bool LimelightSubsystem::IsTarget() {
  return targetFound;
}

double LimelightSubsystem::GetXOffset() {
  return targetXOffset;
}

double LimelightSubsystem::GetYOffset() {
  return targetYOffset;
}

double LimelightSubsystem::GetTargetArea() {
  return targetArea;
}

double LimelightSubsystem::GetTargetSkew() {
  return targetSkew;
}

std::vector<double> LimelightSubsystem::GetBotPos() {
  return botPose;
}

std::vector<double> LimelightSubsystem::GetTargetPos() {
  return targetPose;
}