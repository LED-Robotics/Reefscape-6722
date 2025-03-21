// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <utility>

#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "GlobalConstants.h"

// return current Alliance from either FMS or Driver Station
bool RobotContainer::IsBlue() {
  return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
}

// INPUT A BLUE POSE
frc::Pose2d RobotContainer::HandleAlliancePose(frc::Pose2d pose) {
  if(!IsBlue()) {
    return pose.RotateAround(fieldMiddle, 180_deg);
  } else {
    return pose;
  }
}

// INPUT A BLUE POSE
frc::Pose2d RobotContainer::SwapToRed(frc::Pose2d pose) {
  return pose.RotateAround(fieldMiddle, 180_deg);
}

void RobotContainer::HandlePartnerCommands(frc2::CommandPtr solo, frc2::CommandPtr partner) {
  if(controller2.IsConnected()) {
    ManuallySchedule(std::move(partner));
  } else {
    ManuallySchedule(std::move(solo));
  }
}

frc2::CommandPtr RobotContainer::GetEmptyCommand() {
  return frc2::cmd::RunOnce(
  [&]() {

  }, {});
}

frc2::CommandPtr RobotContainer::SetAllKinematics(RobotContainer::KinematicsPose pose) {
  // Fill with command(s)
  std::vector<frc2::CommandPtr> commands;
  // Initial cascade pose
  units::length::meter_t cascadeTarget = pose.cascadePose;

  // Set cascade to valid height if pose is invalid
  /*if(cascadeTarget < minCoralSweepHeight) {*/
  /*  if(pose.pivotAngle > coralSweepRange[0] && pose.pivotAngle < coralSweepRange[1]) {*/
  /*    cascadeTarget = minCoralSweepHeight;*/
  /*  }*/
  /*}*/
  /*if(cascadeTarget < minAlgaeSweepHeight) {*/
  /*  if(pose.pivotAngle > algaeSweepRange[0] && pose.pivotAngle < algaeSweepRange[1]) {*/
  /*    cascadeTarget = minAlgaeSweepHeight;*/
  /*  }*/
  /*}*/

  auto currentAngle = pivot.GetAngle();
  // Can't sweep if the target isn't changing
  if(currentAngle != pose.pivotAngle) {
    auto top = currentAngle > pose.pivotAngle ? currentAngle : pose.pivotAngle;
    auto bottom = currentAngle < pose.pivotAngle ? currentAngle : pose.pivotAngle;
    // Check if ranges intersect
    auto checkIfSweeping = [&] (units::angle::degree_t range[2], units::length::meter_t min) {
      bool angleCheck = bottom >= range[0] || top <= range[1];
      if(angleCheck && cascadeTarget < min) return true;
      else return false;
    };

    bool coralSweeping = checkIfSweeping(coralSweepRange, minCoralSweepHeight);
    bool algaeSweeping = checkIfSweeping(algaeSweepRange, minAlgaeSweepHeight);
    
    SmartDashboard::PutBoolean("coralSweeping", coralSweeping);
    SmartDashboard::PutBoolean("algaeSweeping", algaeSweeping);
    // Add an initial command to prevent collision
    if(coralSweeping || algaeSweeping) {
      SmartDashboard::PutBoolean("preventedExplosion", true);
      // Get to safe sweep height
      auto preventExplosion = cascade.GetMoveCommand(coralSweeping ? minCoralSweepHeight : minAlgaeSweepHeight);
      commands.push_back(std::move(preventExplosion));
      // Do sweep at safe height
      auto doSweep = pivot.GetMoveCommand(pose.pivotAngle);
      commands.push_back(std::move(doSweep));
    }
  }
  
  // Set subsystems to final targets
  auto setTargets = frc2::cmd::RunOnce(
  [&, cascadeTarget, pose]() {
    cascade.SetTargetPosition(cascadeTarget);
    pivot.SetTargetAngle(pose.pivotAngle);
  }, {&cascade, &pivot});

  commands.push_back(std::move(setTargets));

  // Return command vector
  /*return GetEmptyCommand();*/
  return frc2::cmd::Sequence(std::move(commands));
}

bool RobotContainer::ManipulatorIsAtTarget() {
  return cascade.IsAtTarget() && pivot.IsAtTarget();
}

void RobotContainer::ManuallySchedule(frc2::CommandPtr&& cmd) {
  frc2::CommandScheduler::GetInstance().Schedule(std::move(cmd));
}

void RobotContainer::SetMLTarget(int label) {
  mlTrackingTarget = label;
  switch(label) {
    case MLLabels::Coral:
      autonReefLineup = false;
      autonCoralLoad = true;
      drive.SetTransAdjust(true);
      break;
    case MLLabels::Reef:
      autonReefLineup = true;
      autonCoralLoad = false;
      drive.SetTransAdjust(true);
      break;
    case MLLabels::Algae:
      autonReefLineup = false;
      autonCoralLoad = false;
      drive.SetTransAdjust(false);
      break;
    default:
      autonReefLineup = false;
      autonCoralLoad = false;
      drive.SetTransAdjust(false);
      break;
  }
}

// Helper to disqualify nonviable reef detections
bool RobotContainer::IsReefDisqualified(JetsonSubsystem::MLDetectionFrame &reef) {
  // Calculate relevant data
  double heightRatio = reef.h / reef.w;
  double centerY = reef.y + (reef.h / 2.0);
  double area = reef.h * reef.w;
  
  // Disqualifying conditions for a reef detection
  bool wrongCamera = reef.camId != mlReefCamId;
  bool taller = heightRatio > (ReefHeightLevel == 4 ? reefL4HeightRatioThreshold : reefHeightRatioThreshold);
  /*bool lower = centerY > reefYPosMax;*/
  bool lower = true;
  bool bigEnough = area > (ReefHeightLevel == 4 ? reefL4AreaMin : reefAreaMin);
  if(wrongCamera || !taller || !lower || !bigEnough) return true;
  else return false;
}

// Check if detection is within persistence deadzones
bool RobotContainer::IsViableCoralPersistenceTarget(JetsonSubsystem::MLDetectionFrame &coral) {
  if(!persistenceDataSet) return true;
  
  // Width/height persistence threshold check
  if(fabs(coral.w - mlLastWidth) > coralMaxWidthDrift) return false;
  if(fabs(coral.h - mlLastHeight) > coralMaxHeightDrift) return false;
  
  // Current drivetrain speed
  auto speeds = drive.GetChassisSpeeds();
  double timePassed = (coral.timeCaptured - mlLastCaptureTime) / 1000.0;

  // X coordinate persistence check
  double allowableXDrift = coralMaxXDrift;
  // Scale drift by chassis speed and camera latency
  allowableXDrift += (speeds.vy.value() * timePassed) * coralXSpeedMultiplier;
  if(fabs(coral.x - mlLastX) > allowableXDrift) return false;

  // Y coordinate persistence check
  double allowableYDrift = coralMaxYDrift;
  // Scale drift by chassis speed and camera latency
  allowableYDrift += (speeds.vx.value() * timePassed) * coralYSpeedMultiplier;
  if(fabs(coral.y - mlLastY) > allowableYDrift) return false;

  // Congrats! You were not filtered
  return true;
}

// Helper to disqualify nonviable coral detections
bool RobotContainer::IsCoralDisqualified(JetsonSubsystem::MLDetectionFrame &coral) {
  // Calculate relevant data
  double area = coral.h * coral.w;
  
  // Disqualifying conditions for a reef detection
  bool wrongCamera = coral.camId != mlReefCamId;
  bool bigEnough = area > 250;
  bool tooBig = area > coralAreaMax;
  if(wrongCamera || !bigEnough || tooBig) return true;
  else return false;
}

// Select reef tracking target using confidence and persistence data
JetsonSubsystem::MLDetectionFrame RobotContainer::GetCoralTrackingTarget(std::vector<JetsonSubsystem::MLDetectionFrame> &dets) {
  int detSize = dets.size();
  if(!detSize) {
    noCoralFound = true;
    return {};
  } else {
    noCoralFound = false;
  }
  // Pointer array to sort detections
  std::vector<JetsonSubsystem::MLDetectionFrame*> sorted(detSize, nullptr);
  for(int i = 0; i < detSize; i++) {
    sorted[i] = &dets[i];
  }
  int numViable = detSize;
  // Move everything past index to the left by one
  auto pop = [&] (int index) {
    for(int i = index; i < numViable; i++) {
      if(i + 1 < numViable) { // Out of bounds prevention
        sorted[i] = sorted[i + 1];
      }
    }
    // Duh
    numViable--;
  };

  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto coral = *sorted[i];
    // Clear non-viable reefs
    if(IsCoralDisqualified(coral)) {
      pop(i--); // Next index is now current index
      continue;
    }
    /*if(persistenceDataSet && !IsViableCoralPersistenceTarget(coral)) {*/
    /*  pop(i--); // Next index is now current index*/
    /*}*/
  }

  if(!numViable){
    noCoralFound = true;
    return {
        MLLabels::Coral,
        mlReefCamId,
        mlLastCaptureTime,
        mlLastX,
        mlLastY,
        mlLastWidth,
        mlLastHeight
      };
  } else if(numViable) {
      noCoralFound = false;
  }
  
  // Find reef closest to target x coordinate
  double closest = 10000.0;
  double centerTarget = camFrameWidth / 2.0;
  JetsonSubsystem::MLDetectionFrame *target = sorted[0];
  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto coral = *sorted[i];
    double centerX = coral.x + (coral.w / 2.0);

    mlDCenter = fabs(centerX - centerTarget);
    if(mlDCenter > closest) continue;
    closest = mlDCenter;
    target = &coral;
  }
  
  // Store persistence data from selection
  persistenceDataSet = true;
  mlRioLastCaptureTime = frc::Timer::GetFPGATimestamp();
  mlLastX = target->x;
  mlLastY = target->y;
  mlLastWidth = target->w;
  mlLastHeight = target->h;
  mlLastHeightRatio = target->h / target->w;

  return *target;
}

// Check if detection is within persistence deadzones
bool RobotContainer::IsViableReefPersistenceTarget(JetsonSubsystem::MLDetectionFrame &reef) {
  if(!persistenceDataSet) return true;
  
  // Width/height persistence threshold check
  if(fabs(reef.w - mlLastWidth) > reefMaxWidthDrift) return false;
  if(fabs(reef.h - mlLastHeight) > reefMaxHeightDrift) return false;
  
  // Current drivetrain speed
  auto speeds = drive.GetChassisSpeeds();
  double timePassed =  (reef.timeCaptured - mlLastCaptureTime) / 1000.0;

  // X coordinate persisten ce check
  double allowableXDrift = reefMaxXDrift;
  // Scale drift by chassis speed and camera latency
  allowableXDrift += (speeds.vy.value() * timePassed) * reefXSpeedMultiplier;
  if(fabs(reef.x - mlLastX) > allowableXDrift) return false;

  // Y coordinate persistence check
  double allowableYDrift = reefMaxYDrift;
  // Scale drift by chassis speed and camera latency
  allowableYDrift += (speeds.vx.value() * timePassed) * reefYSpeedMultiplier;
  if(fabs(reef.y - mlLastY) > allowableYDrift) return false;

  // Congrats! You were not filtered
  return true;
}


// Select reef tracking target using confidence and persistence data
JetsonSubsystem::MLDetectionFrame RobotContainer::GetReefTrackingTarget(std::vector<JetsonSubsystem::MLDetectionFrame> &dets) {
  int detSize = dets.size();
  if(!detSize) {
    noReefFound = true;
    return {};
  } else {
    noReefFound = false;
  }
  // Pointer array to sort detections
  std::vector<JetsonSubsystem::MLDetectionFrame*> sorted(detSize, nullptr);
  for(int i = 0; i < detSize; i++) {
    sorted[i] = &dets[i];
  }
  int numViable = detSize;
  // Move everything past index to the left by one
  auto pop = [&] (int index) {
    for(int i = index; i < numViable; i++) {
      if(i + 1 < numViable) { // Out of bounds prevention
        sorted[i] = sorted[i + 1];
      }
    }
    // Duh
    numViable--;
  };

  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto reef = *sorted[i];
    // Clear non-viable reefs
    if(IsReefDisqualified(reef)) {
      pop(i--); // Next index is now current index
      continue;
    }
    /*if(persistenceDataSet && !IsViableReefPersistenceTarget(reef)) {*/
    /*  pop(i--); // Next index is now current index*/
    /*}*/
  }

  /*if(!numViable && persistenceDataSet) {*/
  /*  if(++currentRetries > persistenceRetries) {*/
  /*    persistenceDataSet = false;*/
  /*    currentRetries = 0;*/
  /*    noReefFound = true;*/
  /*    return {};*/
  /*  }*/
  /*} else if(!numViable){*/
  /*  noReefFound = true;*/
  /*  return {*/
  /*      MLLabels::Reef,*/
  /*      mlReefCamId,*/
  /*      mlLastCaptureTime,*/
  /*      mlLastX,*/
  /*      mlLastY,*/
  /*      mlLastWidth,*/
  /*      mlLastHeight*/
  /*    };  */
  /*} else if(numViable) {*/
  /*    noReefFound = false;*/
  /*}*/

  if(!numViable){
    noReefFound = true;
    return {
        MLLabels::Reef,
        mlReefCamId,
        mlLastCaptureTime,
        mlLastX,
        mlLastY,
        mlLastWidth,
        mlLastHeight
      };
  } else if(numViable) {
      noReefFound = false;
  }
  
  // Find reef closest to target x coordinate
  double closest = 10000.0;
  double centerTarget = camFrameWidth / 2.0;
  JetsonSubsystem::MLDetectionFrame *target = sorted[0];
  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto reef = *sorted[i];
    double centerX = reef.x + (reef.w / 2.0);

    mlDCenter = fabs(centerX - centerTarget);
    if(mlDCenter > closest) continue;
    closest = mlDCenter;
    target = &reef;
  }

  // Store persistence data from selection
  /*persistenceDataSet = true;*/
  /*mlRioLastCaptureTime = frc::Timer::GetFPGATimestamp();*/
  /*mlLastX = target->x;*/
  /*mlLastY = target->y;*/
  /*mlLastWidth = target->w;*/
  /*mlLastHeight = target->h;*/
  /*mlLastHeightRatio = target->h / target->w;*/

  return *target;
}

frc2::CommandPtr RobotContainer::GetMLFollowCommand() {
  return frc2::cmd::Run(
    [this] {
      auto dets = jetson.GetMLDetections();
      int detSize = dets.size();
      SmartDashboard::PutNumber("numDets", detSize);
      if(mlTrackingTarget == MLLabels::Algae) {

      } else if(mlTrackingTarget == MLLabels::Coral) {
        /*return;*/
        std::vector<JetsonSubsystem::MLDetectionFrame> coralDets;
        for(int i = 0; i < dets.size(); i++) {
          if(dets[i].label == MLLabels::Coral) {
            coralDets.push_back(dets[i]);
          }
        }

        SmartDashboard::PutNumber("numCoral", coralDets.size());
        auto target = GetCoralTrackingTarget(coralDets);
        if(noCoralFound) {
          drive.SetTransXAdjustSpeeds(0.0_mps);
          return;
        }
        mlDCenter = target.x + (target.w / 2.0);
        mlDCenter = mlDCenter - (camFrameWidth / 2.0) - camFrameCenterOffset;
        auto xAdjust = units::meters_per_second_t{coralAdjust.Calculate(mlDCenter)};

        drive.SetTransXAdjustSpeeds({xAdjust});
        SmartDashboard::PutNumber("yVelAdjust", xAdjust.value());
        SmartDashboard::PutNumber("coralTx", target.x);
        SmartDashboard::PutNumber("coralTy", target.y);
        SmartDashboard::PutNumber("coralTw", target.w);
        SmartDashboard::PutNumber("coralTh", target.h);
        SmartDashboard::PutNumber("coralArea", target.w * target.h);
        SmartDashboard::PutNumber("coralDelta", mlDCenter);

      } else if(mlTrackingTarget == MLLabels::Reef) {
        if(ReefHeightLevel != 4) return;
        std::vector<JetsonSubsystem::MLDetectionFrame> reefDets;
        for(int i = 0; i < dets.size(); i++) {
          if(dets[i].label == MLLabels::Reef) {
            reefDets.push_back(dets[i]);
          }
        }

        SmartDashboard::PutNumber("numReef", reefDets.size());
        auto target = GetReefTrackingTarget(reefDets);
        SmartDashboard::PutNumber("reefTx", target.x);
        SmartDashboard::PutNumber("reefTy", target.y);
        SmartDashboard::PutNumber("reefTw", target.w);
        SmartDashboard::PutNumber("reefTh", target.h);
        SmartDashboard::PutNumber("reefArea", target.w * target.h);

        if(noReefFound) {
          drive.SetTransXAdjustSpeeds(0.0_mps);
          return;
        }
        mlDCenter = target.x + (target.w / 2.0);
        mlDCenter = mlDCenter - (camFrameWidth / 2.0) - camFrameCenterOffset;
        auto xAdjust = units::meters_per_second_t{reefAdjust.Calculate(mlDCenter)};

        drive.SetTransXAdjustSpeeds({xAdjust});
        SmartDashboard::PutNumber("reefDelta", mlDCenter);
        SmartDashboard::PutNumber("yVelAdjust", xAdjust.value());


      }
    },
  {&jetson});
}

RobotContainer::RobotContainer() {
  // Autonomous selector configuration
  autonChooser.SetDefaultOption("None", EmptyAuto.get());
  autonChooser.AddOption("1PieceMiddleBlue", OnePieceMiddleBlue.get());
  autonChooser.AddOption("1PieceMiddleRed", OnePieceMiddleRed.get());
  autonChooser.AddOption("2PieceLeftBlue", TwoPieceLeftBlue.get());
  autonChooser.AddOption("2PieceLeftRed", TwoPieceLeftRed.get());
  autonChooser.AddOption("2PieceRightBlue", TwoPieceRightBlue.get());
  autonChooser.AddOption("2PieceRightRed", TwoPieceRightRed.get());

  SmartDashboard::PutNumber("ML Camera ID", mlReefCamId);

  cascade.SetTargetPosition(startingPose.cascadePose);
  pivot.SetTargetAngle(startingPose.pivotAngle);

  SmartDashboard::PutData(std::move(&autonChooser));  // send auton selector to Shuffleboard

  odomTrigger.WhileTrue(std::move(repeatOdom)); // trigger to handle odom updates from AprilTags
 
  /*******Controller Bindings*******/
  
  /*controller.POVLeft().OnTrue(std::move(targetArbitrary));*/

  //Turn lock toggles
  /*controller.LeftStick().OnTrue(std::move(toggleOmegaOverride));*/

  // Uncomment for actual use to prevent dumbass
  // controller.A().OnTrue(std::move(drive.FollowPathCommand("Example Path")));

  // Controller rumble commands 
  // controller.Start().OnTrue(std::move(rumblePrimaryOn));
  // controller.Start().OnFalse(std::move(rumblePrimaryOff));

  controller.Start().OnTrue(frc2::cmd::RunOnce([this]() {
        SetMLTarget(-1);
        ManuallySchedule(std::move(SetAllKinematics(startingPose)));
  }, {}));

  /*controller.Back().OnTrue(frc2::cmd::RunOnce([this]() {*/
  /*      if(TrackingTarget == GlobalConstants::kAlgaeMode) {*/
  /*        ManuallySchedule(std::move(SetAllKinematics({0.0_m, 170_deg})));*/
  /*      } else {*/
  /*        ManuallySchedule(std::move(SetAllKinematics({0.07_m, 0_deg})));*/
  /*      }*/
  /*    }, {}*/
  /*));*/
  
  controller.LeftStick().OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Coral);

            auto angle = IsBlue() ? blueLoading[LoadTarget].Rotation() : redLoading[LoadTarget].Rotation();
            drive.SetThetaToHold(angle);
            ManuallySchedule(std::move(SetAllKinematics(loadPose)));
        } else {
            SetMLTarget(-1);

            auto angle = processorLoading.Rotation();
            if(IsBlue()) angle.RotateBy(180_deg);
            drive.SetThetaToHold(angle);
            ManuallySchedule(std::move(SetAllKinematics(floorIntakePose)));
        }
  }, {}));

  controller2.A().OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
          SetMLTarget(MLLabels::Coral);

          auto angle = IsBlue() ? blueLoading[LoadTarget].Rotation() : redLoading[LoadTarget].Rotation();
          drive.SetThetaToHold(angle);
          ManuallySchedule(std::move(SetAllKinematics(loadPose)));
        } else {
          SetMLTarget(-1);

          auto angle = processorLoading.Rotation();
          if(IsBlue()) angle.RotateBy(180_deg);
          drive.SetThetaToHold(angle);
          ManuallySchedule(std::move(SetAllKinematics(floorIntakePose)));
        }
      }, {}));

  // Level 1
  mainDpadDown.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 1;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l1Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l1Algae)));
        }
      }, {}));

  mainDpadDown2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 1;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l1Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l1Algae)));
        }
      }, {}));

  // Level 2
  mainDpadRight.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 2;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l2Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l2Algae)));
        }
      }, {}));

  mainDpadRight2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 2;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l2Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l2Algae)));
        }
      }, {}));

  // Level 3
  mainDpadLeft.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 3;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l3Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l3Algae)));
        }
      }, {}));

  mainDpadLeft2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 3;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l3Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l3Algae)));
        }
      }, {}));

  // Level 4
  mainDpadUp.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l4Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l4Algae)));
        }
      }, {}));

  mainDpadUp2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(std::move(SetAllKinematics(l4Coral)));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(std::move(SetAllKinematics(l4Algae)));
        }
      }, {}));

  drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());

  controller.RightStick().ToggleOnTrue(frc2::cmd::RunOnce([this]() {
    ManuallySchedule(std::move(frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this](){
        DisableTagTracking();
      }, {}),
      drive.PathGenCommand(IsBlue() ? blueReef[ReefTarget] : redReef[ReefTarget]),
      // drive.FollowPathCommand("1_meter"),
      frc2::cmd::RunOnce([this]() {
        EnableTagTracking();
      }, {}))));
  }, {}));

  reefTargetChanged.WhileTrue(frc2::cmd::RunOnce([this]() {
    rerunThetaSet = false;
    double xDist = controller2.GetLeftX();
    double yDist = -controller2.GetLeftY();
    
    double reefControlAngle = -atan2(yDist, xDist) * (180 / M_PI);

    if(reefControlAngle < 0) reefControlAngle += 360; // Makes this between 0 and 359

    reefControlAngle += 90;
    
    // if(reefControlAngle < 0) reefControlAngle = 270 - reefControlAngle; //If the shift sends the angle below 0, put back into 0-360 scope

    // reefControlAngle = abs(reefControlAngle - 360.0);

    // SmartDashboard::PutNumber("reefTargAngle", reefControlAngle);

    reefControlAngle /= 60.0;

    ReefTarget = round(reefControlAngle);

    if(ReefTarget >= 6) ReefTarget -= 6;

    frc::Rotation2d angle;
    if(TrackingTarget == GlobalConstants::kCoralMode) {
      angle = IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation();
      if(ReefHeightLevel == 1) angle = angle.RotateBy(180_deg);
    } else {
      angle = IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation();
      if(ReefHeightLevel != 4) angle = angle.RotateBy(180_deg);
    }
    drive.SetThetaToHold(angle);
    SmartDashboard::PutNumber("reefTarget", ReefTarget);
  }, {}).Repeatedly());

  controller.A().OnTrue(frc2::cmd::RunOnce([this]() {
    // drive.SetTransAdjust(!drive.GetTransAdjust());
    drive.SetOmegaOverride(!drive.GetOmegaOverride());
  }, {}));
  
  // Change global target to coral
  controller.LeftBumper().OnTrue(std::move(targetCoral)); 
  controller2.LeftBumper().OnTrue(std::move(coDriverTargetCoral));
  controller2.Back().OnTrue(std::move(coDriverIntakeCoral)); 

  //Later change so that clicking cycles through
  
  // Change global target to algae 
  controller.RightBumper().OnTrue(std::move(targetAlgae));
  controller2.RightBumper().OnTrue(std::move(coDriverTargetAlgae));
  controller2.Start().OnTrue(std::move(coDriverIntakeAlgae));

  controller2.X().OnTrue(std::move(kerblooey));

  //Command toggle for field centric
  controller.Y().OnTrue(std::move(toggleFieldCentric));
  SmartDashboard::PutBoolean("preventedExplosion", false);
  
  /*******Subsystem DEFAULT Commands*******/
  drive.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      SmartDashboard::PutNumber("Subsystem Target", TrackingTarget);
      // store control inputs for driving
      double x = -controller.GetLeftY();
      double y = -controller.GetLeftX();
      double turnX = controller.GetRightX();

      tagOverrideDisable = SmartDashboard::GetBoolean("detectorOverride", true);
      
      // zero out axes if they fall within deadzon
      if (x > -DriveConstants::kDriveDeadzone && x < DriveConstants::kDriveDeadzone)
          x = 0.0;
      if (y > -DriveConstants::kDriveDeadzone && y < DriveConstants::kDriveDeadzone)
          y = 0.0;

      // put speeds through a polynomial to smooth out joystick input
      // check the curve out here: https://www.desmos.com/calculator/65tpwhxyai the range between 0.0 to 1.0 is used for the motors
      // change driveCurveExtent to modify curve strength
      float xSpeed = DriveConstants::kDriveCurveExtent * pow(x, 3) + (1 - DriveConstants::kDriveCurveExtent) * x;
      float ySpeed = DriveConstants::kDriveCurveExtent * pow(y, 3) + (1 - DriveConstants::kDriveCurveExtent) * y;
      float turn = 0.95 * pow(turnX, 3) + (1 - 0.95) * turnX;
      // pass filtered inputs to Drive function
      // inputs will be between -1.0 to 1.0, multiply by intended speed range in mps/deg_per_s when passing
      
      if(!autonCoralLoad && !autonReefLineup) {
      /*if(!autonCoralLoad) {*/
        int paddleAdjust = controller.GetHID().GetXButton() - controller.GetHID().GetBButton();
        drive.SetTransXAdjustSpeeds(paddleAdjust * 0.3_mps);

        if(paddleAdjust != 0) drive.SetTransAdjust(true);
        else drive.SetTransAdjust(false);
      }
      
      double cascadeAdjust = 1.0 - ((cascade.GetPosition() - 0.2_m).value() / 2.0);
      if(cascadeAdjust < 0.0) cascadeAdjust = 0.0;
      if(cascadeAdjust > 1.0) cascadeAdjust = 1.0;
      SmartDashboard::PutNumber("cascadeAdjust", cascadeAdjust);
      drive.Drive({
          units::velocity::meters_per_second_t {xSpeed * cascadeAdjust * DriveConstants::kDriveTranslationLimit.value()}, 
          units::velocity::meters_per_second_t {ySpeed * cascadeAdjust * DriveConstants::kDriveTranslationLimit.value()}, 
          turn * -270.0_deg_per_s}, 
          true, fieldCentric);
    }, {&drive}));

  coral.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      double power = 0.0;
      if(TrackingTarget == GlobalConstants::kCoralMode){
        power = controller.GetLeftTriggerAxis() - controller.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      if(CodriverIntakeTarget == GlobalConstants::kCoralMode) {
        if(power == 0.0) power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      if(power > 0.0) power *= 0.1;
      coral.SetPower(power);
    },
  {&coral}));

  algae.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      double power = 0.0;
      if(TrackingTarget == GlobalConstants::kAlgaeMode) {
        power = controller.GetLeftTriggerAxis() - controller.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      if(CodriverIntakeTarget == GlobalConstants::kAlgaeMode) {
        if(power == 0.0) power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      algae.SetIntakePower(power);
    }, 
  {&algae}));

  // climb.SetDefaultCommand(frc2::cmd::Run(
  //   [this] {
  //     double power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
  //     if(fabs(power) < 0.15) power = 0.0;
  //     climb.SetPower(power);
  //   }, 
  // {&climb}));

  xTransAdjust.SetSetpoint(0.0);
  yTransAdjust.SetSetpoint(0.0);

  jetson.SetDefaultCommand(GetMLFollowCommand());

  led.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      switch(TrackingTarget) {
        case GlobalConstants::kCoralMode:
          led.SetPower(LEDConstants::kCoralPreset);
          break;
        case GlobalConstants::kAlgaeMode:
          led.SetPower(LEDConstants::kAlgaePreset);
          break;
        default:
          led.SetPower(LEDConstants::kIdlePreset);
          break;
      }
    },
  {&led}));

}

void RobotContainer::SetDriveBrakes(bool state) {
  drive.SetBrakeMode(state);
}

void RobotContainer::ZeroDriveAdjustments() {
  drive.SetTransXAdjustSpeeds(0.0_mps);
  drive.SetTransYAdjustSpeeds(0.0_mps);
}

void RobotContainer::DisableTagTracking() {
  tagOverrideDisable = true;
  frc::SmartDashboard::PutBoolean("detectorOverride", true);
}

void RobotContainer::EnableTagTracking() {
  tagOverrideDisable = false;
  frc::SmartDashboard::PutBoolean("detectorOverride", false);
}

void RobotContainer::SetSlew(bool state) {
  drive.SetLimiting(state);
}

void RobotContainer::SetRecording(bool state) {
  jetson.SetRecording(state);
}

void RobotContainer::ChangeCoralCamID(int newId) {
  if(newId != -1) return;
  mlReefCamId = newId;
}

void RobotContainer::ChangeAprilTagCamID(int newId) {
  if(newId != -1) return;
  aprilTagCamId  = newId;
  jetson.ChangeTempCamId(aprilTagCamId);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autonChooser.GetSelected();
}
