// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/JetsonSubsystem/JetsonSubsystem.h"

#include <iostream>

using namespace frc;

JetsonSubsystem::JetsonSubsystem() {
  // Implementation of subsystem constructor goes here
  table = nt::NetworkTableInstance::GetDefault().GetTable("jetson");
}

void JetsonSubsystem::Periodic() {
  parsedTagData = ParseRawTagInfo(GetRawTagInfo());
  jetsonTagDetections = CreateTagVector(parsedTagData);
  table->PutRaw("requestedTags", requestedTags);

}

std::vector<uint8_t> JetsonSubsystem::GetRawTagInfo() {
  std::vector<uint8_t> tagBuf = table->GetRaw("tagBuf", {});
  return tagBuf;
}

std::vector<AprilTagFrame> JetsonSubsystem::ParseRawTagInfo(std::vector<uint8_t> rawBuf) {
  uint8_t* arrayData = &rawBuf[0]; //Turn the recieved vector into an array for memcpy
  std::vector<AprilTagFrame> tagData; //Init the return vector of data
  int bufSize = arrayData[0] + (arrayData[1] << 8); //Bit shift the first two pieces of data which represent the int of how long the buffer is
  for(int i = 2; i < bufSize && i + 1 < bufSize; i++) {
    if(arrayData[i] == 0x69 && arrayData[i + 1] == 0x69) {
      AprilTagFrame parsedData;
      memcpy(&parsedData, arrayData + i + 2, TAG_FRAME_SIZE);
      tagData.push_back(parsedData);
    }
    i += TAG_FRAME_SIZE - 1;
  }
  return tagData;
}

std::vector<TagDetections> JetsonSubsystem::CreateTagVector(std::vector<AprilTagFrame> parsedData) {
  std::vector<TagDetections> finalResult;
  for(int i = 0; i < (int)parsedData.size(); i++) {
    AprilTagFrame tag = parsedData.at(i);
    Translation3d tagTranslation(units::meter_t(tag.tx), units::meter_t(tag.ty), units::meter_t(tag.tz));
    Rotation3d tagRotation(units::degree_t(tag.rx), units::degree_t(tag.ry), units::degree_t(tag.rz));
    Transform3d tagTransform(tagTranslation, tagRotation);
    TagDetections finalData {
      .tagId = tag.tagId,
      .tagTransform = tagTransform
      
    };
    finalResult.push_back(finalData);
  }
  return finalResult;
}

void JetsonSubsystem::AddRequestedTags(std::vector<uint8_t> wantedIDS) {
  for(uint8_t id : wantedIDS) {
    if(std::count(requestedTags.begin(), requestedTags.end(), id)) continue;
    else (requestedTags.push_back(id));
  }
}

void JetsonSubsystem::RemoveRequestedTags(std::vector<uint8_t> removeIDS) {
    for(int i = 0; i < (int)removeIDS.size(); i++) {
      if(std::count(requestedTags.begin(), requestedTags.end(), removeIDS.at(i))) requestedTags.erase(requestedTags.begin() + i); 
      else continue;
  }
}
