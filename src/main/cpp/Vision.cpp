#include "RobotContainer.h"

void RobotContainer::SetMLTarget(int label) {
  mlTrackingTarget = label;
  switch(label) {
    case MLLabels::Coral:
      autonReefLineup = false;
      autonCoralLoad = true;
      useLoadTarget = true;
      drive.SetTransAdjust(true);
      jetson.EnableML(aprilTagCamId);
      jetson.DisableML(stationaryMLCamId);
      jetson.EnableML(coralCamId);
      jetson.DisableML(algaeCamId);
      break;
    case MLLabels::Reef:
      autonReefLineup = true;
      autonCoralLoad = false;
      useLoadTarget = false;
      drive.SetTransAdjust(true);
      jetson.DisableML(aprilTagCamId);
      jetson.EnableML(stationaryMLCamId);
      jetson.EnableML(coralCamId);
      jetson.DisableML(algaeCamId);
      break;
    case MLLabels::Algae:
      autonReefLineup = false;
      autonCoralLoad = false;
      useLoadTarget = false;
      drive.SetTransAdjust(false);
      jetson.EnableML(aprilTagCamId);
      jetson.DisableML(stationaryMLCamId);
      jetson.DisableML(coralCamId);
      jetson.EnableML(algaeCamId);
      break;
    default:
      autonReefLineup = false;
      autonCoralLoad = false;
      useLoadTarget = false;
      drive.SetTransAdjust(false);
      jetson.EnableML(aprilTagCamId);
      jetson.EnableML(stationaryMLCamId);
      jetson.DisableML(coralCamId);
      jetson.DisableML(algaeCamId);
      break;
  }
}

// Helper to disqualify nonviable reef detections
bool RobotContainer::IsReefDisqualified(JetsonSubsystem::MLDetectionFrame &reef) {
  // Calculate relevant data
  /*double heightRatio = reef.h / reef.w;*/
  if(reef.label != MLLabels::Reef) return true;
  double centerY = reef.y + (reef.h / 2.0);
  double centerX = reef.x + (reef.w / 2.0);
  double area = reef.h * reef.w;
  
  // Disqualifying conditions for a reef detection
  bool wrongCamera = reef.camId != coralCamId;
  bool withinXRange = true;
  if(ReefHeightLevel != 4) {
    withinXRange = centerX > 0 + reefXNarrow && centerX < camFrameWidth - reefXNarrow;
  }
  bool bigEnough = ReefHeightLevel == 4 ? true : area > reefAreaMin;
  bool lower = centerY > reefYPosMax;
  return wrongCamera || !withinXRange || !bigEnough || ReefHeightLevel == 4 ? false : lower;
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
  bool wrongCamera = coral.camId != coralCamId;
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
        coralCamId,
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
  double centerX = reef.x + (reef.w / 2.0);
  allowableXDrift *= (speeds.vy.value() * timePassed) * reefXSpeedMultiplier;
  double bottom = allowableXDrift < 0.0 ? mlLastX + allowableXDrift : mlLastX;
  double top = bottom == mlLastX ? mlLastX + allowableXDrift : mlLastX;
  if(centerX > top || centerX < bottom) return false;
  /*if(fabs(reef.x - mlLastX) > allowableXDrift) return false;*/

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
  /*      coralCamId,*/
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
        coralCamId,
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

  std::vector<double> bigGuys(detSize, 0.0);

  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto reef = *sorted[i];
    double area = reef.w * reef.h;
    bigGuys.push_back(area);   
  }

  int areas = bigGuys.size();
  bool swapped;

  for (int i = 0; i < areas - 1; i++) {
    swapped = false;
    for (int j = 0; j < areas - i - 1; j++) {
      if (bigGuys[j] < bigGuys[j + 1]) {
        std::swap(bigGuys[j], bigGuys[j + 1]);
        swapped = true;
      }
    }
  
    // If no two elements were swapped, then break
    if (!swapped)
      break;
  }

  double areaThres = 0.0;
  double averaged = (double)numViable / 2.0;
  averaged = ceil(averaged);
  int avg = (int)averaged;
  for(int i = 0; i < avg; i++) {
    areaThres += bigGuys[i];
  }
  areaThres /= avg;
  SmartDashboard::PutNumber("avg", avg);
  SmartDashboard::PutNumber("areaThesssss", areaThres);

  double dCenter = 999999;
  for(int i = 0; i < numViable; i++) {
    if(sorted[i] == nullptr) continue;
    auto reef = *sorted[i];
    double area = reef.w * reef.h;
    SmartDashboard::PutNumber("preFilterArea", area);
    if(area < areaThres * 0.5) continue;
    double centerX = reef.x + (reef.w / 2.0);
    dCenter = fabs(centerX - centerTarget);
    if(dCenter > closest) continue;
    closest = dCenter;
    target = &reef;
  }

  bool areaCheck = target->w * target->h < areaThres * 0.5;
  bool signageCheck = (closest > 0 && mlDCenter < 0) || (closest < 0 && mlDCenter > 0);
  signageCheck = signageCheck && fabs(closest - mlDCenter) > reefMaxXDrift;
  mlDCenter = closest;
  if(areaCheck || signageCheck) {
    noReefFound = true;
    return {
        MLLabels::Reef,
        coralCamId,
        mlLastCaptureTime,
        mlLastX,
        mlLastY,
        mlLastWidth,
        mlLastHeight
      };
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

frc2::CommandPtr RobotContainer::GetReefLineupCommand() {
  return frc2::cmd::Sequence(
    // Set relevant flags pre-lineup
    frc2::cmd::RunOnce([this]() {
        SetMLTarget(MLLabels::Reef);
        ReefHeightLevel = 4;
        drive.SetTransAdjust(true);
        noReefFound = true;
        mlDCenter = 696969;
    }),
    frc2::cmd::WaitUntil([this]() {
      double dist = drive.GetWallDistance();
      double off =  dist - 0.29;
      if(off < 0.0) off = 0.0;
      if(off > 1.0) off = 1.0;
      drive.SetTransYAdjustSpeeds(-5.0_mps * off);
      drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);

      return dist < 0.31;
    }),
    // Modify translational adjustment until robot is lined up
    frc2::cmd::Race(
      GetMLFollowCommand(), // Run until you lose the race
      frc2::cmd::WaitUntil([this]() {
        drive.SetTransYAdjustSpeeds(0.0_mps);
        drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);

        bool centered = fabs(mlDCenter) < mlAutoScoreThreshold && !noReefFound;
        /*double distToCenter = fabs(mlDCenter);*/
        /*if(distToCenter > mlAutoScoreThreshold * 1.2) {*/
        /*  double farAdjust = 1 - ((distToCenter - mlAutoScoreThreshold) / mlAutoScoreThreshold);*/
        /*  drive.SetTransYAdjustSpeeds(2.0_mps * farAdjust);*/
        /*} else {*/
        /*  // Fill if needed*/
        /*}*/
        if(centered) {
          drive.SetTransXAdjustSpeeds(0.0_mps);
          drive.SetTransYAdjustSpeeds(0.0_mps);
          drive.SetTransAdjust(false);
          drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);
        }
        return centered;
      })
    ),
    frc2::cmd::Deadline(
      frc2::cmd::Wait(0.2_s),
      GetMLFollowCommand()
    ),
    frc2::cmd::RunOnce([this]() {
      drive.SetTransXAdjustSpeeds(0.0_mps);
      drive.SetTransYAdjustSpeeds(0.0_mps);
      drive.SetTransAdjust(false);
      drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);
    }, {})
  );
}

frc2::CommandPtr RobotContainer::GetCoralLineupCommand() {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this]() {
      /*drive.SetOmegaOverride(true);*/
      drive.SetThetaToHold(IsBlue() ? autonLoading[0].Rotation() : SwapToRed(autonLoading[0]).Rotation());
      SetMLTarget(MLLabels::Coral);
      drive.SetTransAdjust(true);
      coral.SetPower(-1.0);
    }),
    frc2::cmd::Race(
      frc2::cmd::WaitUntil([this]() {
        drive.SetTransYAdjustSpeeds(0.0_mps);
        drive.Drive({0.0_mps, 0.22_mps, 0.0_deg_per_s}, true, false);

        return coral.IsCoralIndexed();
      }),
      GetMLFollowCommand()
    ),
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(0.0);
    }, {&coral}),
    frc2::cmd::Deadline(
      frc2::cmd::Wait(0.3_s),
      GetMLFollowCommand()
    ),
    frc2::cmd::RunOnce([this]() {
      drive.SetTransXAdjustSpeeds(0.0_mps);
      drive.SetTransAdjust(false);
      drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);
    }, {})
  );
}

frc2::CommandPtr RobotContainer::PathGenKinematics(frc::Pose2d pose, KinematicsPose stance, units::time::second_t delay) {
  return frc2::cmd::Sequence(
    frc2::cmd::Parallel(
      frc2::cmd::RunOnce([this, pose]() {
        /*tagOverrideDisable = true;      */
        SetMLTarget(-1);
        drive.SetTransAdjust(false);
        drive.SetThetaToHold(pose.Rotation());
        drive.SetOmegaOverride(false);
      }, {}),
      drive.PathGenCommand(pose), // Drive to pose
      frc2::cmd::Sequence(  // Wait for delay and then set kinematics stance
        frc2::cmd::Wait(delay),
        SetAllKinematics(stance),
        // Do not release until at target
        frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); }))
    ),
    frc2::cmd::RunOnce([this]() {
      /*drive.SetOmegaOverride(true);*/
      tagOverrideDisable = false;      
    }, {})
  );
}

frc2::CommandPtr RobotContainer::GetMLFollowCommand() {
  return frc2::cmd::Run(
    [this] {
      auto dets = jetson.GetMLDetections();
      int detSize = dets.size();
      SmartDashboard::PutNumber("numDets", detSize);
      coralCamFrameCenter = SmartDashboard::GetNumber("reefCamCenter", coralCamFrameCenter);
      reefXNarrow = SmartDashboard::GetNumber("reefXNarrow", reefXNarrow);
      reefXSpeedMultiplier = SmartDashboard::GetNumber("reefXDSpeed", reefXSpeedMultiplier);
      reefYPosMax = SmartDashboard::GetNumber("reefYHeightThres", reefYPosMax);
      if(mlTrackingTarget == MLLabels::Algae) {

      } else if(mlTrackingTarget == MLLabels::Coral) {
        /*return;*/
        std::vector<JetsonSubsystem::MLDetectionFrame> coralDets;
        for(unsigned int i = 0; i < dets.size(); i++) {
          if(dets[i].label == MLLabels::Coral) {
            coralDets.push_back(dets[i]);
          }
        }

        SmartDashboard::PutNumber("numCoral", coralDets.size());
        auto target = GetCoralTrackingTarget(coralDets);
        if(noCoralFound || tempDisableTracking) {
          drive.SetTransXAdjustSpeeds(0.0_mps);
          return;
        }
        mlDCenter = target.x + (target.w / 2.0);
        mlDCenter = mlDCenter - (camFrameWidth / 2.0) - coralCamFrameCenter;
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
        if(ReefHeightLevel == 1) return;
        std::vector<JetsonSubsystem::MLDetectionFrame> reefDets;
        for(unsigned int i = 0; i < dets.size(); i++) {
          if(dets[i].label == MLLabels::Reef) {
            reefDets.push_back(dets[i]);
          }
        }

        reefHeightRatioThreshold = SmartDashboard::GetNumber("reefHeightRatio", reefHeightRatioThreshold);
        reefYPosMax = SmartDashboard::GetNumber("reefYPosMax", reefYPosMax);
        reefAreaMin = SmartDashboard::GetNumber("reefAreaMin", reefAreaMin);
        persistenceRetries = SmartDashboard::GetNumber("persist", persistenceRetries);
        reefMaxWidthDrift = SmartDashboard::GetNumber("persistWD", reefMaxWidthDrift);
        reefMaxHeightDrift = SmartDashboard::GetNumber("persistHD", reefMaxHeightDrift);
        reefMaxXDrift = SmartDashboard::GetNumber("persistXD", reefMaxXDrift);
        reefMaxYDrift = SmartDashboard::GetNumber("persistYD", reefMaxYDrift);
        reefAdjust.SetP(SmartDashboard::GetNumber("reefkP", reefAdjust.GetP()));
        reefAdjust.SetI(SmartDashboard::GetNumber("reefkI", reefAdjust.GetI()));
        reefAdjust.SetD(SmartDashboard::GetNumber("reefkD", reefAdjust.GetD()));

        SmartDashboard::PutNumber("numReef", reefDets.size());
        auto target = GetReefTrackingTarget(reefDets);
        SmartDashboard::PutNumber("reefTx", target.x);
        SmartDashboard::PutNumber("reefTy", target.y);
        SmartDashboard::PutNumber("reefTw", target.w);
        SmartDashboard::PutNumber("reefTh", target.h);
        SmartDashboard::PutNumber("reefArea", target.w * target.h);

        
        mlDCenter = target.x + (target.w / 2.0);
        mlDCenter = mlDCenter - (camFrameWidth / 2.0) - coralCamFrameCenter;
        SmartDashboard::PutNumber("mlDCenter", mlDCenter);

        disableWallSensor = SmartDashboard::GetBoolean("disableWallSensor", disableWallSensor);
        bool inReefProximity = disableWallSensor ? true : (drive.GetWallDistance() < reefAutoAlignThreshold);
        if(noReefFound || tempDisableTracking || !inReefProximity) {
          drive.SetTransXAdjustSpeeds(0.0_mps);
          return;
        }        

        auto xAdjust = units::meters_per_second_t{reefAdjust.Calculate(mlDCenter)};

        drive.SetTransXAdjustSpeeds({xAdjust});
        SmartDashboard::PutNumber("reefDelta", mlDCenter);
        SmartDashboard::PutNumber("yVelAdjust", xAdjust.value());


      }
    },
  {&jetson});
}
