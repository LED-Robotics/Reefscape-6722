// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/JetsonSubsystem/JetsonSubsystem.h"

#include <iostream>

using namespace frc;


JetsonSubsystem::JetsonSubsystem() {
  // Implementation of subsystem constructor goes here
  table = nt::NetworkTableInstance::GetDefault().GetTable("jetson");
  field = AprilTagFieldLayout::LoadField(AprilTagField::k2025Reefscape);
  field.SetOrigin(AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  testCam0 = {0, {0.0_m, 0.371_m, -0.089_m, {0.0_deg, 0.0_deg, 90.0_deg}}};
  testCam1 = {1, {0.0_m, 0.0_m, 0.0_m, {0.0_deg, 0.0_deg, -90.0_deg}}};
  testCam2 = {2, {0.0_m, 0.0_m, 0.0_m, {0.0_deg, 0.0_deg, -90.0_deg}}};
  
  this->AddRequestedTags(std::vector<uint8_t> {6, 7, 8, 9, 10, 11});

  cams = {testCam0, testCam1, testCam2};
}

void JetsonSubsystem::Periodic() {
  parsedTagData = ParseRawTagInfo(GetRawTagInfo());
  jetsonTagDetections = CreateTagVector(parsedTagData);
  fieldRelativePose = AverageRobotPose();  
  
  table->PutRaw("rqsted", requestedTags);

  // Debug printouts
  // if(!jetsonTagDetections.empty()) {
  //   SmartDashboard::PutNumber("testSigma", jetsonTagDetections.at(0).tagId);
  // }
  // else SmartDashboard::PutNumber("testSigma", 0);

  // SmartDashboard::PutNumber("Final Detections", jetsonTagDetections.size());

  // SmartDashboard::PutNumber("Parsed size", parsedTagData.size());
}

std::vector<uint8_t> JetsonSubsystem::GetRawTagInfo() {
  std::vector<uint8_t> tagBuf = table->GetRaw("tagBuf", {});
  return tagBuf;
}

std::vector<AprilTagFrame> JetsonSubsystem::ParseRawTagInfo(std::vector<uint8_t> rawBuf) {
  std::vector<AprilTagFrame> tagData = {};
  bool temp = false;
  int bufSize = 2;
  if(rawBuf.size() > 2){
    uint8_t* arrayData = &rawBuf[0]; //Turn the recieved vector into an array for memcpy
    bufSize = arrayData[0] + (arrayData[1] << 8); //Bit shift the first two pieces of data which represent the int of how long the buffer is
      for(int i = 2; i < bufSize && i + 1 < bufSize; i++) {
        if(arrayData[i] == 0x69 && arrayData[i + 1] == 0x69) {
          AprilTagFrame parsedData;
          memcpy(&parsedData, arrayData + i + 2, TAG_FRAME_SIZE);
          tagData.push_back(parsedData);
        }
        i += TAG_FRAME_SIZE - 1;
        
      }
  }
    // SmartDashboard::PutNumber("Buffer Length", bufSize);
    return tagData;
}

std::vector<TagDetections> JetsonSubsystem::CreateTagVector(std::vector<AprilTagFrame> parsedData) {
  std::vector<TagDetections> finalResult;
  for(int i = 0; i < (int)parsedData.size(); i++) {
    AprilTagFrame tag = parsedData.at(i);
    int camFrameId = tag.camId;

    for(CameraInformation cam : cams) {
      if(cam.camId == camFrameId) camTrans = cam.pose;
    }

    // tag.tx = left/right from tag
    // tag.ty = height from tag
    // tag.tz = forward from tag

    Translation3d tagTranslation(units::meter_t(tag.tz), units::meter_t(-tag.tx), units::meter_t(tag.ty));
    Rotation3d tagRotation(units::degree_t(tag.rx), units::degree_t(tag.ry), units::degree_t(tag.rz));

    Transform3d tagTransform(tagTranslation, tagRotation);

    Transform3d finalTransform = tagTransform + camTrans;

    // std::cout << "tx: " << finalTransform.X().value() << std::endl; 
    // std::cout << "ty: " << finalTransform.Y().value() << std::endl; 
    // std::cout << "tz: " << finalTransform.Z().value() << std::endl; 
    // std::cout << "rz: " << units::degree_t(finalTransform.Rotation().Z()).value() << std::endl; 
    
    std::cout << "Tag tx: " << tag.tx << std::endl; 
    std::cout << "Tag ty: " << tag.ty << std::endl; 
    std::cout << "Tag tz: " << tag.tz << std::endl; 



    auto realTagPose = field.GetTagPose((int)tag.tagId);

    // std::cout << tagTranslation.Y().value() << std::endl;

    int tagId = tag.tagId;
    if(!realTagPose.has_value()) continue;
    Pose3d finalPose = field.GetTagPose(tagId).value().TransformBy(finalTransform);

    TagDetections finalData;
      finalData.tagId = tag.tagId;
      finalData.aprilTagRelativePose = finalTransform;
      finalData.fieldRelativePose = finalPose;

    finalResult.push_back(finalData);


    // SmartDashboard::PutNumber("JetsonX", finalData.aprilTagRelativePose.X().value());
    // SmartDashboard::PutNumber("JetsonY", finalData.aprilTagRelativePose.Y().value());
    // SmartDashboard::PutNumber("JetsonZ", finalData.aprilTagRelativePose.Z().value());
    // SmartDashboard::PutNumber("JetsonRot", units::degree_t{finalData.aprilTagRelativePose.Rotation().Angle()}.value());

  }
  if(finalResult.empty()) poseAvailable = false;
  else poseAvailable = true;
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
      auto remove = std::remove(requestedTags.begin(), requestedTags.end(), i);
      requestedTags.erase(remove, requestedTags.end());
    }
}

frc::Pose2d JetsonSubsystem::AverageRobotPose() {
  if(poseAvailable) {
    Translation2d finalFieldTrans;
    Rotation2d finalFieldRot;
    int count = 0;
    for(TagDetections& detections : jetsonTagDetections) {
      finalFieldTrans = finalFieldTrans + detections.fieldRelativePose.ToPose2d().Translation();
      count++;
    }
    finalFieldTrans = finalFieldTrans / count;
    return {finalFieldTrans, finalFieldRot};
  }
  else return {};
}

bool JetsonSubsystem::IsPoseAvailable() {
  return poseAvailable;
}