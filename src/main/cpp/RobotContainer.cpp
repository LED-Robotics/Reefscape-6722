// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <iostream>
#include <frc/controller/PIDController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "pathplanner/lib/auto/NamedCommands.h"
#include <pathplanner/lib/path/PathPlannerPath.h>

using namespace pathplanner;

// Load a Choreo trajectory as a PathPlannerPath
// PathPlannerPath exampleChoreoTraj = PathPlannerPath::fromChoreoTrajectory("Example Choreo Traj");

#include "Constants.h"

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
  autonChooser.AddOption("Test1", testAuto.get());
  
// Load a Choreo trajectory as a PathPlannerPath
  // auto exampleChoreoTraj = PathPlannerPath::fromChoreoTrajectory("Slur");

  frc::SmartDashboard::PutBoolean("detectorOverride", tagOverrideDisable); // flashes green to tell drivers odom has been updated

  // Autonomous selector configuration
  // SmartDashboard::PutData(&musicalSelector); // musical selector to shuffleboard

  //Pahtplanner Stuff
  pathplanner::NamedCommands::registerCommand("TestInstantCommand", std::move(testInstantCommand));
  pathplanner::NamedCommands::registerCommand("Odom Reset", std::move(autonOdomSet));
  
  // chooser.SetDefaultOption("Dummy", highDock.get());
  autonChooser.AddOption("None", GetEmptyCommand());

  SmartDashboard::PutData(&autonChooser);  // send auton selector to Shuffleboard

  zeroTrigger.WhileTrue(&reZeroSwerve); // trigger to handle odom updates from AprilTags

  // DISABLE WHEN NOT IN USE SO DUMMIES DON'T COMMIT MURDER
  // controller.B().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("testPath2Rawr")));
  // controller.A().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("testPath2Rawr")));
  // controller.X().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("turn")));
  controller.X().ToggleOnTrue(m_drive.FollowPathCommand(PathPlannerPath::fromChoreoTrajectory("Slur")));


  // controller.Start().OnTrue(std::move(indexPrimed));

  mainDpadLeft.OnTrue(&targetArbitrary);
  // mainDpadUp.OnTrue(&targetSpeaker);
  // mainDpadRight.OnTrue(&targetAmp);
  // mainDpadDown.OnTrue(&targetSource);
  // controller.B().OnTrue(&reZeroSwerve); // trigger to handle odom updates from AprilTags


  // partner LED control bindings
  // controller2.A().OnTrue(&SetBlinkinAButton);
  // controller2.LeftBumper().OnTrue(&SetBlinkinLeftBumper);
  // controller2.RightBumper().OnTrue(&SetBlinkinRightBumper);

  controller.Back().OnTrue(&toggleOmegaOverride);



  // funny rumble command bindings. These might not work. 
  // controller.Start().WhileTrue(&rumbleSecondaryOn);
  controller2.Start().WhileTrue(&rumblePrimaryOn);
  // controller.Start().OnFalse(&rumbleSecondaryOff);
  controller2.Start().OnFalse(&rumblePrimaryOff);
  // controller.RightBumper().OnTrue(std::move(targetSpeaker));

  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      SmartDashboard::PutNumber("Subsystem Target", TrackingTarget);

      // lazy non-command implementation of a field-centric drive toggle
      if(controller.GetYButtonPressed()) fieldCentric = !fieldCentric;

      // store control inputs for driving
      double x = -controller.GetLeftY();
      double y = -controller.GetLeftX();
      double turnX = controller.GetRightX();

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
      turn * -226.0_deg_per_s}, true, fieldCentric);
    }, {&m_drive}));
}

void RobotContainer::SetDriveBrakes(bool state) {
        resetting = state;
        m_drive.SetBrakeMode(state);
        // elevator.SetBrakeMode(state);
        // arm.SetBrakeMode(state);
        // intake.SetBrakeMode(state);
        //Ahlgrims mom is attractive
}

void RobotContainer::ZeroSwerve() {
        m_drive.ZeroSwervePosition();
}

void RobotContainer::SetSlew(bool state) {
  m_drive.SetLimiting(state);
}

// return selected auton routine from Shuffleboard



frc2::Command* RobotContainer::GetAutonomousCommand() {
    auto selected = autonChooser.GetSelected();
    return selected;
}