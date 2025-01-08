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

std::vector<uint8_t> JetsonSubsystem::GetRawTagInfo() {
  uint8_t* arrayData = &tagBuf[0];
  int bufSize = arrayData[0] + (arrayData[1] << 8);
  for(int i = 2; i < bufSize && i + 1 < bufSize; i++) {
    if(arrayData[i] == 0x69 && arrayData[i + 1] == 0x69) {
      AprilTagFrame parsedData;
      memcpy(&parsedData, arrayData + i + 2, TAG_FRAME_SIZE);

      // Write frames to publishing source
      // Done separately because synced web streams are nice
        std::cout << "Parsed data: " << std::endl;
        std::cout << (int)parsedData.tagId << std::endl;
        std::cout << (int)parsedData.camId << std::endl;
        std::cout << parsedData.tx << std::endl;
        std::cout << parsedData.ty << std::endl;
        std::cout << parsedData.tz << std::endl;
        std::cout << std::endl;
    }
    i += TAG_FRAME_SIZE - 1;
  }
}

