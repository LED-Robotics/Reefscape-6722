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

frc2::CommandPtr RobotContainer::HandleModeScheduling(frc2::CommandPtr coral, frc2::CommandPtr algae) {
  return frc2::cmd::RunOnce([&]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          frc2::CommandScheduler::GetInstance().Schedule(std::move(algae));
          /*ManuallySchedule(algae);*/
        } else {
          frc2::CommandScheduler::GetInstance().Schedule(std::move(coral));
          /*ManuallySchedule(coral);*/
        }
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

void RobotContainer::ManuallySchedule(frc2::CommandPtr&& cmd) {
  frc2::CommandScheduler::GetInstance().Schedule(std::move(cmd));
}

// Helper to disqualify nonviable reef detections
bool RobotContainer::IsReefDisqualified(JetsonSubsystem::MLDetectionFrame &reef) {
  // Calculate relevant data
  double heightRatio = reef.h / reef.w;
  double centerY = reef.y + (reef.h / 2.0);
  double area = reef.h * reef.w;
  
  // Disqualifying conditions for a reef detection
  bool wrongCamera = reef.camId != mlReefCamId;
  bool taller = heightRatio > reefHeightRatioThreshold;
  bool lower = centerY > reefYPosMax;
  bool bigEnough = area > reefAreaMin;
  if(wrongCamera || !taller || !lower || !bigEnough) return true;
  else return false;
}

// Check if detection is within persistence deadzones
bool RobotContainer::IsViablePersistenceTarget(JetsonSubsystem::MLDetectionFrame &reef) {
  if(!persistenceDataSet) return true;
  
  // Width/height persistence threshold check
  if(fabs(reef.w - mlLastWidth) > maxWidthDrift) return false;
  if(fabs(reef.h - mlLastHeight) > maxHeightDrift) return false;
  
  // Current drivetrain speed
  auto speeds = drive.GetChassisSpeeds();
  double timePassed =  (reef.timeCaptured - mlLastCaptureTime) / 1000.0;

  // X coordinate persistence check
  double allowableXDrift = maxXDrift;
  // Scale drift by chassis speed and camera latency
  allowableXDrift += (speeds.vy.value() * timePassed) * xSpeedMultiplier;
  if(fabs(reef.x - mlLastX) > allowableXDrift) return false;

  // Y coordinate persistence check
  double allowableYDrift = maxYDrift;
  // Scale drift by chassis speed and camera latency
  allowableYDrift += (speeds.vx.value() * timePassed) * ySpeedMultiplier;
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
    if(!IsReefDisqualified(reef)) {
      pop(i--); // Next index is now current index
      continue;
    }
    if(persistenceDataSet && !IsViablePersistenceTarget(reef)) {
      pop(i--); // Next index is now current index
    }
  }

  if(!numViable && persistenceDataSet) {
    if(++currentRetries > persistenceRetries) {
      persistenceDataSet = false;
      currentRetries = 0;
      noReefFound = true;
      return {};
    }
  } else if(!numViable){
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

    double dCenter = fabs(centerX - centerTarget);
    if(dCenter > closest) continue;
    closest = dCenter;
    target = &reef;
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

RobotContainer::RobotContainer() {
  // Autonomous selector configuration
  autonChooser.SetDefaultOption("None", "None");

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

  controller.Start().OnTrue(SetAllKinematics(startingPose));

  /*controller.Back().OnTrue(frc2::cmd::RunOnce([this]() {*/
  /*      if(TrackingTarget == GlobalConstants::kAlgaeMode) {*/
  /*        ManuallySchedule(std::move(SetAllKinematics({0.0_m, 170_deg})));*/
  /*      } else {*/
  /*        ManuallySchedule(std::move(SetAllKinematics({0.07_m, 0_deg})));*/
  /*      }*/
  /*    }, {}*/
  /*));*/
  
  controller.LeftStick().OnTrue(frc2::cmd::RunOnce([this]() {
          if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(loadPose)));
          } else {
            ManuallySchedule(std::move(SetAllKinematics(floorIntakePose)));
          }
        }, {}));

  controller2.A().OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(loadPose)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(floorIntakePose)));
        }
      }, {}));

  // Level 1
  mainDpadDown.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l1Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l1Algae)));
        }
      }, {}));

  mainDpadDown2.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l1Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l1Algae)));
        }
      }, {}));

  // Level 2
  mainDpadRight.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l2Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l2Algae)));
        }
      }, {}));

  mainDpadRight2.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l2Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l2Algae)));
        }
      }, {}));

  // Level 3
  mainDpadLeft.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l3Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l3Algae)));
        }
      }, {}));

  mainDpadLeft2.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l3Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l3Algae)));
        }
      }, {}));

  // Level 4
  mainDpadUp.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l4Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l4Algae)));
        }
      }, {}));

  mainDpadUp2.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            ManuallySchedule(std::move(SetAllKinematics(l4Coral)));
          }   else {
            ManuallySchedule(std::move(SetAllKinematics(l4Algae)));
        }
      }, {}));

  drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());

  dpad2InteractedWith.OnTrue(frc2::cmd::RunOnce([this]() {
    ReefTarget = (int)(controller2.GetHID().GetPOV() / 45.0);
    drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());
    SmartDashboard::PutNumber("reefTarget", ReefTarget);
  }, {}));

  /*controller.B().OnTrue(frc2::cmd::RunOnce([this]() {*/
  /*  drive.SetTransAdjust(!drive.GetTransAdjust());*/
  /*  drive.SetOmegaOverride(drive.GetTransAdjust());*/
  /*}, {}));*/

  // controller.Y().OnTrue(frc2::cmd::RunOnce([this]() {
  //   if(--ReefTarget < 0) ReefTarget = 5; 
  //   drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());
  //   SmartDashboard::PutNumber("reefTarget", ReefTarget);
  // }, {}));

  // controller.A().OnTrue(frc2::cmd::RunOnce([this]() {
  //   if(++ReefTarget > 5) ReefTarget = 0; 
  //   drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());
  //   SmartDashboard::PutNumber("reefTarget", ReefTarget);
  // }, {}));
  
  // Change global target to coral
  controller.LeftBumper().OnTrue(std::move(targetCoral)); 
  controller2.LeftBumper().OnTrue(std::move(coDriverTargetCoral));
  controller2.Back().OnTrue(std::move(coDriverIntakeCoral)); 

  //Later change so that clicking cycles through
  
  // Change global target to algae 
  controller.RightBumper().OnTrue(std::move(targetAlgae));
  controller2.RightBumper().OnTrue(std::move(coDriverTargetAlgae));
  controller2.Start().OnTrue(std::move(coDriverIntakeAlgae));

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
      int paddleAdjust = controller.GetHID().GetXButton() - controller.GetHID().GetBButton();
      drive.SetTransAdjustSpeeds(paddleAdjust * 0.3_mps, 0.0_mps);
      
      if(paddleAdjust != 0) drive.SetTransAdjust(true);
      else drive.SetTransAdjust(false);

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

  // funnel.SetDefaultCommand(frc2::cmd::Run(
  //  [this] {

  //   },  
  // {&funnel}));

  xTransAdjust.SetSetpoint(0.0);
  yTransAdjust.SetSetpoint(0.0);

  jetson.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      auto dets = jetson.GetMLDetections();
      int detSize = dets.size();
      SmartDashboard::PutNumber("numDets", detSize);
      if(mlTrackingTarget == MLLabels::Algae) {

      } else if(mlTrackingTarget == MLLabels::Coral) {

      } else if(mlTrackingTarget == MLLabels::Reef) {
        std::vector<JetsonSubsystem::MLDetectionFrame> reefDets;
        for(int i = 0; i < dets.size(); i++) {
          if(dets[i].label == MLLabels::Reef) {
            reefDets.push_back(dets[i]);
          }
        }

        auto target = GetReefTrackingTarget(reefDets);
        if(noReefFound) return;
        double dCenter = target.x + (target.w / 2.0);
        dCenter = dCenter - (camFrameWidth / 2.0);
        auto yAdjust = units::meters_per_second_t{yTransAdjust.Calculate(dCenter)};

        drive.SetTransAdjustSpeeds(units::meters_per_second_t{0.0}, {yAdjust});
        SmartDashboard::PutNumber("yVelAdjust", yAdjust.value());

        SmartDashboard::PutNumber("reefTx", target.x);
        SmartDashboard::PutNumber("reefTy", target.y);
        SmartDashboard::PutNumber("reefTw", target.w);
        SmartDashboard::PutNumber("reefTh", target.h);
        SmartDashboard::PutNumber("reefArea", target.w * target.h);
        SmartDashboard::PutNumber("reefDelta", dCenter);

      }
    },
  {&jetson}));

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
