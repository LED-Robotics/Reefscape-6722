// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/DriverStation.h>

#include "Constants.h"


using namespace frc;

struct AprilTagFrame {
  uint8_t tagId = -1;
  uint8_t camId = -1;
  uint32_t timeCaptured;
  double tx;
  double ty;
  double tz;
  double rx;
  double ry;
  double rz;
};

struct TagDetections {
  int tagId;
  frc::Transform3d aprilTagRelativePose;
  frc::Pose3d fieldRelativePose;
};

struct CameraInformation {
  int camId;
  frc::Transform3d pose;
};

class JetsonSubsystem : public frc2::SubsystemBase {
 public:
  JetsonSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
    
  /**
   * Get the raw data from the jetson.
   * 
   * @return Raw vector of tag information
   */
  std::vector<uint8_t> GetRawTagInfo();

  /**
   * Turn the raw data from the jetson into usable information.
   * @return A vector of structures of each april tag requested and detected on each camera
   */
  std::vector<AprilTagFrame> ParseRawTagInfo(std::vector<uint8_t> rawBuf);

  /**
   * Add the requested tag(s) to the global vector
   */
  void AddRequestedTags(std::vector<uint8_t> wantedIDS);

  /**
   * Remove the input tag(s) from the global vector
   */
  void RemoveRequestedTags(std::vector<uint8_t> removeIDS);

  /**
   * Add all relevant tag information to the global vector
   * 
   * @return The new vector of Tag Detections
   */
  std::vector<TagDetections> CreateTagVector(std::vector<AprilTagFrame> parsedData);

  /**
   * Grab the average robot pos from all tags detected on field 
   */
  frc::Pose2d AverageRobotPose();

  bool IsPoseAvailable();

 private:
  const size_t TAG_FRAME_SIZE = sizeof(AprilTagFrame);
  bool poseAvailable = false;

  std::shared_ptr<nt::NetworkTable> table;
  
  std::vector<uint8_t> requestedTags;
  std::vector<AprilTagFrame> parsedTagData;
  std::vector<TagDetections> jetsonTagDetections; 

  frc::Transform3d camTrans;
  AprilTagFieldLayout field;

  CameraInformation testCam0;
  CameraInformation testCam1;
  CameraInformation testCam2;

  std::vector<CameraInformation> cams;

  Pose2d fieldRelativePose;
};
