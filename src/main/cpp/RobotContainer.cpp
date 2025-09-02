// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "GlobalConstants.h"
#include "frc2/command/Commands.h"
#include "subsystems/DriveSubsystem/Constants.h"
#include "units/time.h"

// return current Alliance from either FMS or Driver Station
bool RobotContainer::IsBlue() {
  return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
}

// INPUT A BLUE POSE
frc::Pose2d RobotContainer::SwapToRed(frc::Pose2d pose) {
  return pose.RotateAround(fieldMiddle, 180_deg);
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

  auto currentAngle = pivot.GetAngleDegrees();
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
    cascade.SetTargetMeters(cascadeTarget);
    pivot.SetTargetDegrees(pose.pivotAngle);
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

frc2::CommandPtr RobotContainer::PathGenThenKinematics(frc::Pose2d pose, KinematicsPose stance, units::time::second_t delay) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this, pose]() {
      /*tagOverrideDisable = true;      */
      SetMLTarget(-1);
      drive.SetTransAdjust(false);
      drive.SetThetaToHold(pose.Rotation());
      drive.SetOmegaOverride(false);
    }, {}),
    drive.PathGenCommand(pose), // Drive to pose
    frc2::cmd::Wait(delay),
    SetAllKinematics(stance),
    // Do not release until at target
    frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); }),
    frc2::cmd::RunOnce([this]() {
      /*drive.SetOmegaOverride(true);*/
      tagOverrideDisable = false;      
    }, {})
  );
}

frc2::CommandPtr RobotContainer::ShootCoralCommand() {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(0.4);
    }, {&coral}),
    frc2::cmd::Wait(0.7_s),
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(0.0);
    }, {&coral})
  );
}

frc2::CommandPtr RobotContainer::GetCoralFixCommand() {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(0.25);
    }, {&coral}),
    frc2::cmd::Wait(0.09_s),
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(-1.0);
    }, {&coral}),
    frc2::cmd::Wait(0.3_s),
    frc2::cmd::RunOnce([this]() {
      coral.SetPower(0.0);
    }, {&coral})
  );
}

RobotContainer::RobotContainer() {
  // Autonomous selector configuration
  autonChooser.SetDefaultOption("None", EmptyAuto.get());
  autonChooser.AddOption("1PieceMiddle", OnePieceMiddle.get());
  /*autonChooser.AddOption("2PieceLeft", TwoPieceLeft.get());*/
  autonChooser.AddOption("2PieceRight", TwoPieceRight.get());

  SmartDashboard::PutNumber("Stationary Camera ID", stationaryMLCamId);
  SmartDashboard::PutNumber("AprilTag Camera ID", aprilTagCamId);
  SmartDashboard::PutNumber("Coral Camera ID", coralCamId);
  SmartDashboard::PutNumber("Algae Camera ID", algaeCamId);

  SmartDashboard::PutNumber("reefHeightRatio", reefHeightRatioThreshold);
  SmartDashboard::PutNumber("reefYPosMax", reefYPosMax);
  SmartDashboard::PutNumber("reefAreaMin", reefAreaMin);

  SmartDashboard::PutNumber("persistWD", reefMaxWidthDrift);
  SmartDashboard::PutNumber("persistHD", reefMaxHeightDrift);
  SmartDashboard::PutNumber("persistXD", reefMaxXDrift);
  SmartDashboard::PutNumber("persistYD", reefMaxYDrift);

  SmartDashboard::PutNumber("reefCamCenter", coralCamFrameCenter);
  SmartDashboard::PutNumber("reefXNarrow", reefXNarrow);
  SmartDashboard::PutNumber("reefXDSpeed", reefXSpeedMultiplier);
  SmartDashboard::PutNumber("reefYHeightThres", reefYPosMax);

  SmartDashboard::PutNumber("reefkP", reefAdjust.GetP());
  SmartDashboard::PutNumber("reefkI", reefAdjust.GetI());
  SmartDashboard::PutNumber("reefkD", reefAdjust.GetD());

  SmartDashboard::PutBoolean("disableWallSensor", disableWallSensor);

  cascade.SetTargetMeters(startingPose.cascadePose);
  pivot.SetTargetDegrees(startingPose.pivotAngle);

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
        ManuallySchedule(SetAllKinematics(startingPose));
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
            ManuallySchedule(SetAllKinematics(loadPose));
        } else {
            SetMLTarget(-1);

            auto angle = processorLoading.Rotation();
            if(IsBlue()) angle.RotateBy(180_deg);
            drive.SetThetaToHold(angle);
            ManuallySchedule(SetAllKinematics(floorIntakePose));
        }
  }, {}));

  controller2.A().OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
          SetMLTarget(MLLabels::Coral);

          auto angle = IsBlue() ? blueLoading[LoadTarget].Rotation() : redLoading[LoadTarget].Rotation();
          drive.SetThetaToHold(angle);
          ManuallySchedule(SetAllKinematics(loadPose));
        } else {
          SetMLTarget(-1);

          auto angle = processorLoading.Rotation();
          if(IsBlue()) angle.RotateBy(180_deg);
          drive.SetThetaToHold(angle);
          ManuallySchedule(SetAllKinematics(floorIntakePose));
        }
      }, {}));

  controller2.B().OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 1;
        ManuallySchedule(SetAllKinematics(startingPose));
        if(TrackingTarget == GlobalConstants::kCoralMode) {
          ManuallySchedule(SetAllKinematics(startingPose));
        } else {
          ManuallySchedule(SetAllKinematics(floorIntakePose));
        }
      }, {}));

  // Level 1
  mainDpadDown.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 1;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l1Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l1Algae));
        }
      }, {}));

  mainDpadDown2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 1;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l1Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l1Algae));
        }
      }, {}));

  // Level 2
  mainDpadRight.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 2;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l2Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l2Algae));
        }
      }, {}));

  mainDpadRight2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 2;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l2Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l2Algae));
        }
      }, {}));

  // Level 3
  mainDpadLeft.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 3;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l3Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l3Algae));
        }
      }, {}));

  mainDpadLeft2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 3;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l3Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l3Algae));
        }
      }, {}));

  // Level 4
  mainDpadUp.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l4Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l4Algae));
        }
      }, {}));

  mainDpadUp2.OnTrue(frc2::cmd::RunOnce([this]() {
        ReefHeightLevel = 4;
        if(TrackingTarget == GlobalConstants::kCoralMode) {
            SetMLTarget(MLLabels::Reef);
            ManuallySchedule(SetAllKinematics(l4Coral));
          }   else {
            SetMLTarget(-1);
            ManuallySchedule(SetAllKinematics(l4Algae));
        }
      }, {}));

  drive.SetThetaToHold(IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation());

  controller.RightStick().ToggleOnTrue(frc2::cmd::RunOnce([this]() {
    ManuallySchedule(frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this](){
        DisableTagTracking();
      }, {}),
      drive.PathGenCommand(IsBlue() ? blueReef[ReefTarget] : redReef[ReefTarget]),
      // drive.FollowPathCommand("1_meter"),
      frc2::cmd::RunOnce([this]() {
        EnableTagTracking();
      }, {})));
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
    if(useLoadTarget) {
     if(ReefTarget >= 0 && ReefTarget < 3) {
        LoadTarget = 0;
      } else {
        LoadTarget = 1;
      }   
      angle = IsBlue() ? blueLoading[LoadTarget].Rotation() : redLoading[LoadTarget].Rotation();
      loadThetaAngle = angle;
    } else {
      if(TrackingTarget == GlobalConstants::kCoralMode) {
        angle = IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation();
        if(ReefHeightLevel == 1) angle = angle.RotateBy(180_deg);
      } else {
        angle = IsBlue() ? blueReef[ReefTarget].Rotation() : redReef[ReefTarget].Rotation();
        if(ReefHeightLevel != 4) angle = angle.RotateBy(180_deg);
      }
      scoringThetaAngle = angle;
    }
    /*drive.SetThetaToHold(angle);*/
    SmartDashboard::PutNumber("reefTarget", ReefTarget);
  }, {}).Repeatedly());

  controller.A().OnTrue(frc2::cmd::RunOnce([this]() {
    // drive.SetTransAdjust(!drive.GetTransAdjust());
    drive.SetOmegaOverride(!drive.GetOmegaOverride());
  }, {}));
  
  controller.X().OnTrue(frc2::cmd::RunOnce([this]() {
    tempDisableTracking = true;
  }, {}));

  controller.X().OnFalse(frc2::cmd::RunOnce([this]() {
    tempDisableTracking = false;
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

  controller2.X().OnTrue(GetCoralFixCommand());

  //Command toggle for field centric
  controller.Y().OnTrue(std::move(toggleFieldCentric));
  SmartDashboard::PutBoolean("preventedExplosion", false);
  
  /*******Subsystem DEFAULT Commands*******/
  drive.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      
      if(useLoadTarget) {
        drive.SetThetaToHold(loadThetaAngle);
      } else {
        drive.SetThetaToHold(scoringThetaAngle);
      }

      SmartDashboard::PutNumber("Subsystem Target", TrackingTarget);
      SmartDashboard::PutBoolean("ManipAtTarget", ManipulatorIsAtTarget());
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

      if (turnX > -DriveConstants::kTurnDeadzone && turnX < DriveConstants::kTurnDeadzone)
          turnX = 0.0;

      // put speeds through a polynomial to smooth out joystick input
      // check the curve out here: https://www.desmos.com/calculator/65tpwhxyai the range between 0.0 to 1.0 is used for the motors
      // change driveCurveExtent to modify curve strength
      float xSpeed = DriveConstants::kDriveCurveExtent * pow(x, 3) + (1 - DriveConstants::kDriveCurveExtent) * x;
      float ySpeed = DriveConstants::kDriveCurveExtent * pow(y, 3) + (1 - DriveConstants::kDriveCurveExtent) * y;
      float turn = 0.3 * pow(turnX, 3) + (1 - 0.3) * turnX;
      // pass filtered inputs to Drive function
      // inputs will be between -1.0 to 1.0, multiply by intended speed range in mps/deg_per_s when passing
      
      if(!autonCoralLoad && !autonReefLineup) {
      /*if(!autonCoralLoad) {*/
        int paddleAdjust = controller.GetHID().GetXButton() - controller.GetHID().GetBButton();
        drive.SetTransXAdjustSpeeds(paddleAdjust * 0.3_mps);

        if(paddleAdjust != 0) drive.SetTransAdjust(true);
        else drive.SetTransAdjust(false);
      }
      
      double cascadeAdjust = 1.0 - ((cascade.GetPositionMeters() - 0.2_m).value() / 2.2);
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
      if(!controller2.GetHID().GetYButton() && CodriverIntakeTarget == GlobalConstants::kCoralMode) {
        if(power == 0.0) power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      if(power > 0.0) power *= 0.13;
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
      if(!controller2.GetHID().GetYButton() && CodriverIntakeTarget == GlobalConstants::kAlgaeMode) {
        if(power == 0.0) power = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
        if(fabs(power) < 0.1) power = 0.0;
      }
      algae.SetIntakePower(power);
    }, 
  {&algae}));

  climb.SetDefaultCommand(frc2::cmd::Run(
    [this] {
      if(controller2.GetHID().GetYButton()) {
        double power = controller2.GetLeftTriggerAxis();
        if(fabs(power) < 0.15) power = 0.0;
        climb.SetPower(power);
      }
    }, 
  {&climb}));

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

void RobotContainer::ChangeStationaryMLCamID(int newId) {
  if(newId != -1) return;
  stationaryMLCamId = newId;
  SetCameraStates();
}

void RobotContainer::ChangeAprilTagCamID(int newId) {
  if(newId != -1) return;
  aprilTagCamId = newId;
  jetson.ChangeTempCamId(aprilTagCamId);
  SetCameraStates();
}
void RobotContainer::ChangeCoralCamID(int newId) {
  if(newId != -1) return;
  coralCamId = newId;
  SetCameraStates();
}

void RobotContainer::ChangeAlgaeCamID(int newId) {
  if(newId != -1) return;
  algaeCamId = newId;
  SetCameraStates();
}

void RobotContainer::SetCameraStates() {
  SetMLTarget(mlTrackingTarget);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autonChooser.GetSelected();
}
