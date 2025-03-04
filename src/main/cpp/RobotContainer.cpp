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

frc2::Command* RobotContainer::HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner) {
  return new frc2::InstantCommand(
    [this, solo, partner]() { 
        if(controller2.IsConnected()) partner->Schedule();
        else solo->Schedule();
      }, {});
}

frc2::Command* RobotContainer::GetEmptyCommand() {
  return new frc2::InstantCommand(
          [this]() { 
           }, {});
}

frc2::CommandPtr RobotContainer::SetMultijoint(units::length::meter_t cascadeHeight, units::angle::degree_t algaeAngle) {
  return frc2::cmd::Parallel(
    cascade.GetMoveCommand(cascadeHeight),
    algae.GetMoveCommand(algaeAngle)
  );
}

void RobotContainer::ManuallySchedule(frc2::CommandPtr&& cmd) {
  frc2::CommandScheduler::GetInstance().Schedule(cmd);
}

// Helper to disqualify nonviable reef detections
bool RobotContainer::IsReefDisqualified(JetsonSubsystem::MLDetectionFrame reef) {
  // Calculate relevant data
  double heightRatio = reef.h / reef.w;
  double centerY = reef.y + (reef.h / 2.0);
  double area = reef.h * reef.w;
  
  // Disqualifying conditions for a reef detection
  bool taller = heightRatio > reefHeightRatioThreshold;
  bool lower = centerY > reefYPosMax;
  bool bigEnough = area > reefAreaMin;
  if(!taller || !lower || !bigEnough) return true;
  else return false;
}

// Check if detection is within persistence deadzones
bool RobotContainer::IsViablePersistenceTarget(JetsonSubsystem::MLDetectionFrame reef) {
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
JetsonSubsystem::MLDetectionFrame RobotContainer::GetReefTrackingTarget(std::vector<JetsonSubsystem::MLDetectionFrame> dets) {
  int detSize = dets.size();
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
  
  // Find reef closest to target x coordinate
  double closest = 10000.0;
  double centerTarget = camFrameWidth / 2.0;
  JetsonSubsystem::MLDetectionFrame *target = sorted[0];
  for(int i = 0; i < numViable; i++) {
    auto reef = *sorted[i];
    double centerX = reef.x + (reef.w / 2.0);

    double dCenter = fabs(centerX - centerTarget);
    if(dCenter > closest) continue;
    closest = dCenter;
    target = &reef;
  }

  // Store persistence data from selection
  persistenceDataSet = true;
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

  controller.Start().OnTrue(SetMultijoint(0.0_m, 80_deg));

  controller.Back().OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          ManuallySchedule(std::move(SetMultijoint(0.0_m, -60_deg)));
        } else {
          ManuallySchedule(std::move(SetMultijoint(0.07_m, 80_deg)));
        }
      }, {}
  ));

  // Level 1
  mainDpadDown.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          ManuallySchedule(std::move(SetMultijoint(0.0_m, -15.0_deg)));
        } else {
          ManuallySchedule(std::move(SetMultijoint(0.15_m, 80_deg)));
        }
      }, {}
  ));

  // Level 2
  mainDpadRight.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          ManuallySchedule(std::move(SetMultijoint(0.13_m, -5_deg)));
        } else {
          ManuallySchedule(std::move(SetMultijoint(0.33_m, 80_deg)));
        }
      }, {}
  ));

  // Level 3
  mainDpadLeft.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          ManuallySchedule(std::move(SetMultijoint(0.51_m, -5_deg)));
        } else {
          ManuallySchedule(std::move(SetMultijoint(0.71_m, 80_deg)));
        }
      }, {}
  ));

  // Level 4
  mainDpadUp.OnTrue(frc2::cmd::RunOnce([this]() {
        if(TrackingTarget == GlobalConstants::kAlgaeMode) {
          ManuallySchedule(std::move(SetMultijoint(1.3_m, 60.0_deg)));
        } else {
          ManuallySchedule(std::move(SetMultijoint(1.37_m, 80.0_deg)));
        }
      }, {}
  ));

  drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());

  controller.B().OnTrue(frc2::cmd::RunOnce([this]() {
    drive.SetTransAdjust(!drive.GetTransAdjust());
    drive.SetOmegaOverride(!drive.GetTransAdjust());
  }, {}));

  controller.Y().OnTrue(frc2::cmd::RunOnce([this]() {
    if(--ReefTarget < 0) ReefTarget = 5; 
    drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());
    SmartDashboard::PutNumber("reefTarget", ReefTarget);
  }, {}));

  controller.A().OnTrue(frc2::cmd::RunOnce([this]() {
    if(++ReefTarget > 5) ReefTarget = 0; 
    drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());
    SmartDashboard::PutNumber("reefTarget", ReefTarget);
  }, {}));
  
  // Change global target to coral
  controller.LeftBumper().OnTrue(std::move(targetCoral)); 

  //Later change so that clicking cycles through
  
  // Change global target to algae 
  controller.RightBumper().OnTrue(std::move(targetAlgae)); 

  //Command toggle for field centric
  controller.Y().OnTrue(std::move(toggleFieldCentric));
  
  /*******Subsystem DEFAULT Commands*******/
  drive.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      SmartDashboard::PutNumber("Subsystem Target", TrackingTarget);
      // store control inputs for driving
      double x = -controller.GetLeftY();
      double y = -controller.GetLeftX();
      double turnX = controller.GetRightX();

      tagOverrideDisable = SmartDashboard::GetBoolean("detectorOverride", true);
      
      // zero out axes if they fall within deadzone
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
      drive.Drive({xSpeed * DriveConstants::kDriveTranslationLimit, ySpeed * DriveConstants::kDriveTranslationLimit, 
      turn * -270.0_deg_per_s}, true, fieldCentric);
    }, {&drive}));

  intake.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if(TrackingTarget == GlobalConstants::kCoralMode || TrackingTarget == GlobalConstants::kArbitrary){
        double power = controller.GetLeftTriggerAxis() - controller.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
        intake.SetPower(power);
      }
    },
  {&intake}));

  algae.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if(TrackingTarget == GlobalConstants::kAlgaeMode || TrackingTarget == GlobalConstants::kArbitrary) {
        double power = controller.GetLeftTriggerAxis() - controller.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
        algae.SetIntakePower(power);
      }
    }, 
  {&algae}));

  climb.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      double power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
      if(fabs(power) < 0.15) power = 0.0;
      climb.SetPower(power);
    }, 
  {&climb}));

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
        for(int i = 0; i < detSize; i++) {
          if(dets[i].label == MLLabels::Reef) {
            reefDets.push_back(dets[i]);
          }
        }

        auto target = GetReefTrackingTarget(reefDets);
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
    
    },
  {&led}));

}

frc2::CommandPtr RobotContainer::SetAllKinematics(RobotContainer::KinematicsPoses kinInfoRef) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce(
      [&]() {
        cascade.SetTargetPosition(kinInfoRef.cascadePose);
        // Add other subsystems
      }, {&cascade})
  );
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
