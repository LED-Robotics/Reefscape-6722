// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/JetsonSubsystem/JetsonSubsystem.h"

#include <iostream>

using namespace frc;
using namespace JetsonConstants;


JetsonSubsystem::JetsonSubsystem() {
  // Implementation of subsystem constructor goes here
  table = nt::NetworkTableInstance::GetDefault().GetTable("jetson");
  field = AprilTagFieldLayout::LoadField(AprilTagField::k2025ReefscapeWelded);
  field.SetOrigin(AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  
  this->AddRequestedTags(std::vector<uint8_t> {6, 7, 8, 9, 10, 11});
}

void JetsonSubsystem::Periodic() {
  parsedTagData = ParseRawTagInfo(GetRawTagInfo());
  mlDetections = ParseDetections(GetMLInfo());

  for(auto& det : mlDetections) {
    std::cout << "Found label: " << (int)det.label << std::endl;
    std::cout << "At: " << det.x << ", " << det.y << std::endl;
    std::cout << "W: " << det.w << "H: " << det.h << std::endl;
  }
  jetsonTagDetections = CreateTagVector(parsedTagData);
  fieldRelativePose = AverageRobotPose();  
  
  table->PutRaw("rqsted", requestedTags);
  SmartDashboard::PutBoolean("IsPoseAvailable", IsPoseAvailable());

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

std::vector<uint8_t> JetsonSubsystem::GetMLInfo() {
  std::vector<uint8_t> mlBuf = table->GetRaw("mlBuf", {});
  return mlBuf;
}

std::vector<AprilTagFrame> JetsonSubsystem::ParseRawTagInfo(std::vector<uint8_t> rawBuf) {
  std::vector<AprilTagFrame> tagData = {};
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

std::vector<MLDetectionFrame> JetsonSubsystem::ParseDetections(std::vector<uint8_t> rawBuf) {
  std::vector<MLDetectionFrame> detData = {};
  int bufSize = 2;
  if(rawBuf.size() > 2){
    uint8_t* arrayData = &rawBuf[0]; //Turn the recieved vector into an array for memcpy
    bufSize = arrayData[0] + (arrayData[1] << 8); //Bit shift the first two pieces of data which represent the int of how long the buffer is
      for(int i = 2; i < bufSize && i + 1 < bufSize; i++) {
        if(arrayData[i] == 0x69 && arrayData[i + 1] == 0x69) {
          MLDetectionFrame parsedData;
          memcpy(&parsedData, arrayData + i + 2, ML_FRAME_SIZE);
          detData.push_back(parsedData);
        }
        i += ML_FRAME_SIZE - 1;
      }
  }
    // SmartDashboard::PutNumber("Buffer Length", bufSize);
    return detData;
}

std::vector<TagDetections> JetsonSubsystem::CreateTagVector(std::vector<AprilTagFrame> parsedData) {
  std::vector<TagDetections> finalResult;
  poseAvailable = false;
  for(int i = 0; i < (int)parsedData.size(); i++) {
    AprilTagFrame tag = parsedData.at(i);
    // Detection confidence calculation
    double angRaw = fabs(tag.rz) / kAngularConfThresh;
    double angularConf = Constrain(1 - pow(fabs(angRaw), kAngularConfCurveExtent), 0.0, 1.0);
    angularConf *= kAngularConfWeight;
    /*std::cout << "Skibidi Ang:" << std::endl;*/
    /*std::cout << (double)angularConf << std::endl;*/
    /*std::cout << (double)tag.rz << std::endl;*/
    /*std::cout << std::endl;*/

    double distRaw = tag.tz / kDistanceConfThresh;
    double distConf = Constrain(1 - pow(fabs(distRaw), kDistanceConfCurveExtent), 0.0, 1.0);
    distConf *= kDistanceConfWeight;
    /*std::cout << "Skibidi Dist:" << std::endl;*/
    /*std::cout << (double)distConf << std::endl;*/
    /*std::cout << (double)tag.tz << std::endl;*/
    /*std::cout << std::endl;*/

    // Ensure conf weights make angle + dist = 1.0
    double conf = distConf + angularConf;
    
    // std::cout << (int)tag.tagId << std::endl;
    // std::cout << distConf << std::endl;
    // std::cout << angularConf << std::endl;
    // std::cout << conf << std::endl;
    // std::cout << std::endl;
    
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
    
    // std::cout << "Tag tx: " << tag.tx << std::endl; 
    // std::cout << "Tag ty: " << tag.ty << std::endl; 
    // std::cout << "Tag tz: " << tag.tz << std::endl; 

    auto realTagPose = field.GetTagPose((int)tag.tagId);

    // std::cout << tagTranslation.Y().value() << std::endl;

    int tagId = tag.tagId;
    if(!realTagPose.has_value()) continue;
    Pose3d finalPose = field.GetTagPose(tagId).value().TransformBy(finalTransform);
    bool poseFiltered = conf < kPoseConfidenceThresh;
    TagDetections finalData;
      finalData.tagId = tag.tagId;
      finalData.aprilTagRelativePose = finalTransform;
      finalData.fieldRelativePose = finalPose;
      finalData.poseAvailable = !poseFiltered;
      finalData.relativeAvailable = !poseFiltered;

    finalResult.push_back(finalData);

    if(!poseFiltered) poseAvailable = true;


    // SmartDashboard::PutNumber("JetsonX", finalData.aprilTagRelativePose.X().value());
    // SmartDashboard::PutNumber("JetsonY", finalData.aprilTagRelativePose.Y().value());
    // SmartDashboard::PutNumber("JetsonZ", finalData.aprilTagRelativePose.Z().value());
    // SmartDashboard::PutNumber("JetsonRot", units::degree_t{finalData.aprilTagRelativePose.Rotation().Angle()}.value());

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

double JetsonSubsystem::Min(double val, double min) {
  return val < min ? min : val;
}

double JetsonSubsystem::Max(double val, double max) {
  return val > max ? max : val;
}

double JetsonSubsystem::Constrain(double val, double floor, double ceiling) {
  return Min(Max(val, ceiling), floor);
}
