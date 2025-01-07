// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/JetsonSubsystem.h"

#include <iostream>

using namespace frc;

JetsonSubsystem::JetsonSubsystem() {
  // Implementation of subsystem constructor goes here
  table = nt::NetworkTableInstance::GetDefault().GetTable("jetson");
}

/**
 * Reframe to handle both tags and game pieces
 * 
 * Input buffer of tag data
 * Still need to send target tags
 * 
 * 
*/ 

void JetsonSubsystem::Periodic() {
  targetFound = table->GetBoolean("tv", false);
  if(targetFound) {
    targetXOffset = table->GetNumber("tx", 0.0);
    targetYOffset = table->GetNumber("ty", 0.0);
    targetSize = table->GetNumber("ts", 0.0);
  }
}

bool JetsonSubsystem::IsTarget() {
  return targetFound;
}

double JetsonSubsystem::GetXOffset() {
  return targetXOffset;
}

double JetsonSubsystem::GetYOffset() {
  return targetYOffset;
}

double JetsonSubsystem::GetTargetSize() {
  return targetSize;
}

void JetsonSubsystem::SetRecording(bool state) {
  table->PutBoolean("recordState", state);
}