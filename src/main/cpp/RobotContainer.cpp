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

RobotContainer::RobotContainer() {
  // Autonomous selector configuration
  autonChooser.SetDefaultOption("None", "None");

  SmartDashboard::PutData(std::move(&autonChooser));  // send auton selector to Shuffleboard

  odomTrigger.WhileTrue(std::move(repeatOdom)); // trigger to handle odom updates from AprilTags
 
  /*******Controller Bindings*******/
  
  /*controller.POVLeft().OnTrue(std::move(targetArbitrary));*/

  //Turn lock toggles
  controller.LeftStick().OnTrue(std::move(toggleOmegaOverride));

  // Uncomment for actual use to prevent dumbass
  // controller.A().OnTrue(std::move(m_drive.FollowPathCommand("Example Path")));

  // Controller rumble commands 
  // controller.Start().OnTrue(std::move(rumblePrimaryOn));
  // controller.Start().OnFalse(std::move(rumblePrimaryOff));

  controller.Start().OnTrue(SetMultijoint(0.0_m, 80_deg));

  /*controller.Back().OnTrue(cascade.GetMoveCommand(0.2475_m));*/

  mainBack.OnTrue(frc2::cmd::Either(
        SetMultijoint(0.0_m, -45_deg),
        SetMultijoint(0.07_m, 80_deg),
        [&]() { return TrackingTarget == GlobalConstants::kAlgaeMode; }
  ));

  // Level 1
  /*mainDpadDown.OnTrue(cascade.GetMoveCommand(0.435_m)); */
  mainDpadDown.OnTrue(frc2::cmd::Either(
        SetMultijoint(0.0_m, -15.0_deg),
        SetMultijoint(0.15_m, 80_deg),
        [&]() { return TrackingTarget == GlobalConstants::kAlgaeMode; }
  ));

  // Level 2
  /*mainDpadRight.OnTrue(cascade.GetMoveCommand(0.623_m)); */
  mainDpadRight.OnTrue(frc2::cmd::Either(
        SetMultijoint(0.38_m, 0_deg),
        SetMultijoint(0.38_m, 80_deg),
        [&]() { return TrackingTarget == GlobalConstants::kAlgaeMode; }
  ));
  // Level 3
  /*mainDpadLeft.OnTrue(cascade.GetMoveCommand(0.998_m)); */
  mainDpadLeft.OnTrue(frc2::cmd::Either(
        SetMultijoint(0.76_m, 0_deg),
        SetMultijoint(0.76_m, 80_deg),
        [&]() { return TrackingTarget == GlobalConstants::kAlgaeMode; }
  ));
  // Level 4
  /*mainDpadUp.OnTrue(cascade.GetMoveCommand(1.47_m));*/
  mainDpadUp.OnTrue(frc2::cmd::Either(
        SetMultijoint(1.3_m, 60.0_deg),
        SetMultijoint(1.3_m, 80.0_deg),
        [&]() { return TrackingTarget == GlobalConstants::kAlgaeMode; }
  ));
  
  // Change global target to coral
  controller.LeftBumper().OnTrue(std::move(targetCoral)); 

  //Later change so that clicking cycles through
  
  // Change global target to algae 
  controller.RightBumper().OnTrue(std::move(targetAlgae)); 

  //Command toggle for field centric
  controller.Y().OnTrue(std::move(toggleFieldCentric));
  
  /*******Subsystem DEFAULT Commands*******/
  m_drive.SetDefaultCommand(frc2::cmd::Run(
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
      m_drive.Drive({xSpeed * DriveConstants::kDriveTranslationLimit, ySpeed * DriveConstants::kDriveTranslationLimit, 
      turn * -270.0_deg_per_s}, true, fieldCentric);
    }, {&m_drive}));

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

  // climb.SetDefaultCommand(frc2::cmd::Run(
  //   [this] {
          
  //   }, 
  // {&climb}));

  // funnel.SetDefaultCommand(frc2::cmd::Run(
  //   [this] {

  //   },  
  // {&funnel}));

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
  m_drive.SetBrakeMode(state);
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
  m_drive.SetLimiting(state);
}
