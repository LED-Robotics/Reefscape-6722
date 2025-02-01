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
// #include "pathplanner/lib/auto/NamedCommands.h"

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

RobotContainer::RobotContainer() {
  // Autonomous selector configuration
  autonChooser.SetDefaultOption("None", "None");

  SmartDashboard::PutData(std::move(&autonChooser));  // send auton selector to Shuffleboard

  odomTrigger.WhileTrue(std::move(repeatOdom)); // trigger to handle odom updates from AprilTags

  controller.POVLeft().OnTrue(std::move(targetArbitrary));

  // controller2.POVLeft().OnTrue(std::move(targetArbitrary));
  //Turn lock toggles
  controller.LeftStick().OnTrue(std::move(toggleOmegaOverride));
  // controller.RightStick().OnTrue(std::move(rotateTo180));
  // controller2.LeftBumper().OnTrue(std::move(toggleOmegaOverride));

  driverTurning.OnTrue(std::move(tempDisableOmega));
  driverTurning.OnFalse(std::move(restoreOmega));

  // Uncomment for actual use to prevent dumbass
  // controller.A().OnTrue(std::move(m_drive.FollowPathCommand("Example Path")));

  // funny rumble command bindings. These might not work. 
  controller.Start().WhileTrue(std::move(rumblePrimaryOn));
  controller.Start().OnFalse(std::move(rumblePrimaryOff));

  //Command toggle for field centric
  controller.Y().OnTrue(std::move(toggleFieldCentric));
  // Set up default drive command
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

  led.SetDefaultCommand(frc2::cmd::Run(
    [this] {
    
    },
  {&led}));

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
