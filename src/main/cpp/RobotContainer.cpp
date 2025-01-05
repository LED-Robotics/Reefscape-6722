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
<<<<<<< Updated upstream
#include "pathplanner/lib/auto/NamedCommands.h"
=======
// #include "pathplanner/lib/auto/NamedCommands.h"
>>>>>>> Stashed changes

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
  musicalSelector.SetDefaultOption("|01) Cantina",               "Maestro_Musical/cantina.chrp");
  musicalSelector.AddOption("|01) Cantina",                      "Maestro_Musical/cantina.chrp");
  musicalSelector.AddOption("|02) Cynthia's Theme",              "Maestro_Musical/CynthiaTheme3492922.chrp");
  musicalSelector.AddOption("|03) Echo",                         "Maestro_Musical/Echo1547606.chrp");
  musicalSelector.AddOption("|04) FNAF6 Broken Windows",         "Maestro_Musical/FNAF6_3735567.chrp");
  musicalSelector.AddOption("|05) Gusty Garden",                 "Maestro_Musical/GustyGardenVGMUSIC.chrp");
  musicalSelector.AddOption("|06) Imperial March",               "Maestro_Musical/ImperialMarch2178760.chrp");
  musicalSelector.AddOption("|07) The World Revolving",          "Maestro_Musical/Jevil3661983.chrp");
  musicalSelector.AddOption("|08) Marble Hill Zone",             "Maestro_Musical/MarbleHillVGMusic.chrp");
  musicalSelector.AddOption("|09) World's End Valentine",        "Maestro_Musical/OMORI86_3174948.chrp");
  musicalSelector.AddOption("|10) Reconstructing Science",       "Maestro_Musical/ReconstructingScience2809779.chrp");
  musicalSelector.AddOption("|11) The Amazing Digital Circus",   "Maestro_Musical/TADC3699908.chrp");
  musicalSelector.AddOption("|12) Viva La Vida",                 "Maestro_Musical/Viva1717136.chrp");
  musicalSelector.AddOption("|13) BohemianRhapsodyFREEMIDI",     "Maestro_Musical/BohemianRhapsodyFREEMIDI.chrp");
  musicalSelector.AddOption("|14) Bad Apple",                    "Maestro_Musical/BadApple3820096.chrp");
  musicalSelector.AddOption("|15) Fly Me to the Moon",           "Maestro_Musical/FlyMoon3600404.chrp");
  musicalSelector.AddOption("|16) DisassemblyRequired3252559",   "Maestro_Musical/DisassemblyRequired3252559.chrp");
  musicalSelector.AddOption("|17) Green Hill Zone",              "Maestro_Musical/GreenHill3698170.chrp");
  musicalSelector.AddOption("|18) Attack of the Killer Queen",   "Maestro_Musical/KillerQueen3777783.chrp");
  musicalSelector.AddOption("|19) Macarena",                     "Maestro_Musical/Macarena2880456.chrp");
  musicalSelector.AddOption("|20) Master Kohga",                 "Maestro_Musical/MasterKohga3463326.chrp");
  musicalSelector.AddOption("|21) Megalotrousle",                "Maestro_Musical/Megalotrousle2155665.chrp");
  musicalSelector.AddOption("|22) Slipping Through My Fingers",  "Maestro_Musical/SlippingThroughMyFingersFREEMIDI.chrp");
  musicalSelector.AddOption("|23) Airship Battle Theme",         "Maestro_Musical/SMB3AirshipVGMusic.chrp");
  musicalSelector.AddOption("|24) Song of Storms",               "Maestro_Musical/SongOfStorms3780744.chrp");
  musicalSelector.AddOption("|25) I'm Still Standing",           "Maestro_Musical/StillStanding3696063.chrp");
  musicalSelector.AddOption("|26) Wait of the World",            "Maestro_Musical/WaitOfWorld3312229.chrp");
  musicalSelector.AddOption("|27) Asgore",                       "Maestro_Musical/Asgore1633865.chrp");
  musicalSelector.AddOption("|28) Bready Steady Go",             "Maestro_Musical/OMORI125_3448462.chrp");
  musicalSelector.AddOption("|29) Beat It",                      "Maestro_Musical/BeatIt3563264.chrp");
  musicalSelector.AddOption("|30) Buddy Holly",                  "Maestro_Musical/BuddyHolly2294759.chrp");
  musicalSelector.AddOption("|31) Fallen Down",                  "Maestro_Musical/UNDERTALE85_3359661.chrp");
  musicalSelector.AddOption("|32) Livin' On A Prayer",           "Maestro_Musical/LivinOnAPrayer2528149.chrp");
  musicalSelector.AddOption("|33) Lost Woods",                   "Maestro_Musical/LostWoods782496.chrp");
  musicalSelector.AddOption("|34) Pirates of the Carribean",     "Maestro_Musical/Caribbean3586082.chrp");
  musicalSelector.AddOption("|35) Pokemon Trainer Battle",       "Maestro_Musical/PokemonTrainerBattle3557578.chrp");
  musicalSelector.AddOption("|36) Scary Option",                 "Maestro_Musical/Scary2774320.chrp");
  musicalSelector.AddOption("|37) Super Mario 64",               "Maestro_Musical/SM64_3585783.chrp");
  musicalSelector.AddOption("|38) Roll",                         "Maestro_Musical/NeverGonna59740.chrp");
  musicalSelector.AddOption("|39) Rush E",                       "Maestro_Musical/RushEFanchen.chrp");
  musicalSelector.AddOption("|40) Bugger",                       "Maestro_Musical/StickBug.chrp");
  frc::SmartDashboard::PutData("Chosen Song", &musicalSelector);

  frc::SmartDashboard::PutBoolean("detectorOverride", tagOverrideDisable); // flashes green to tell drivers odom has been updated
  frc::SmartDashboard::PutBoolean("autoHunt", autoHuntEnabled); // auto note hunting flag
  frc::SmartDashboard::PutBoolean("autoIntake", autoIntakeEnabled); // auto intake flag

<<<<<<< Updated upstream
    //Pahtplanner Stuff
  pathplanner::NamedCommands::registerCommand("OdomReset", std::move(autonOdomSet));
  pathplanner::NamedCommands::registerCommand("HuntNote", std::move(autoHunt));
  pathplanner::NamedCommands::registerCommand("TargetSpeaker", std::move(autonSpeakerTarget));
  pathplanner::NamedCommands::registerCommand("GoTo180", std::move(rotateTo180));
  pathplanner::NamedCommands::registerCommand("GoTo90", std::move(rotateTo90));
  pathplanner::NamedCommands::registerCommand("GoToNeg90", std::move(rotateToNeg90));
  pathplanner::NamedCommands::registerCommand("GoToNeg70", std::move(rotateToNeg70));
  pathplanner::NamedCommands::registerCommand("GoTo82", std::move(rotateTo82));
  pathplanner::NamedCommands::registerCommand("GoToNeg130", std::move(rotateToNeg130));
  pathplanner::NamedCommands::registerCommand("DriveOff", std::move(driveOff));
  pathplanner::NamedCommands::registerCommand("WaitAligned", std::move(lineupSpeaker));
  pathplanner::NamedCommands::registerCommand("WaitIndexed", std::move(ensureIndexed));
  pathplanner::NamedCommands::registerCommand("OmegaDisable", std::move(autonTrackingDisable));
  pathplanner::NamedCommands::registerCommand("Shoot", std::move(autonShoot));
=======
  //Pahtplanner Stuff
  // pathplanner::NamedCommands::registerCommand("OdomReset", std::move(autonOdomSet));
  // pathplanner::NamedCommands::registerCommand("HuntNote", std::move(autoHunt));
  // pathplanner::NamedCommands::registerCommand("TargetSpeaker", std::move(autonSpeakerTarget));
  // pathplanner::NamedCommands::registerCommand("GoTo180", std::move(rotateTo180));
  // pathplanner::NamedCommands::registerCommand("GoTo90", std::move(rotateTo90));
  // pathplanner::NamedCommands::registerCommand("GoToNeg90", std::move(rotateToNeg90));
  // pathplanner::NamedCommands::registerCommand("GoToNeg70", std::move(rotateToNeg70));
  // pathplanner::NamedCommands::registerCommand("GoTo82", std::move(rotateTo82));
  // pathplanner::NamedCommands::registerCommand("GoToNeg130", std::move(rotateToNeg130));
  // pathplanner::NamedCommands::registerCommand("DriveOff", std::move(driveOff));
  // pathplanner::NamedCommands::registerCommand("WaitAligned", std::move(lineupSpeaker));
  // pathplanner::NamedCommands::registerCommand("WaitIndexed", std::move(ensureIndexed));
  // pathplanner::NamedCommands::registerCommand("OmegaDisable", std::move(autonTrackingDisable));
  // pathplanner::NamedCommands::registerCommand("Shoot", std::move(autonShoot));
>>>>>>> Stashed changes

  // SmartDashboard::PutNumber("noteXP", 0.0);
  // SmartDashboard::PutNumber("noteYP", 0.0);
  // SmartDashboard::PutNumber("noteThetaP", 0.0);
  // SmartDashboard::PutNumber("noteDistanceScale", 0.03);

  // Autonomous selector configuration
  
  autonChooser.SetDefaultOption("None", "None");
  // autonChooser.AddOption("None", GetEmptyCommand());
  autonChooser.AddOption("Four Note", "FourNote");
  autonChooser.AddOption("Far Auton", "FarAuton");
  autonChooser.AddOption("Amp Far Auton", "AmpFarAuton");
  autonChooser.AddOption("Amp Shoot", "AmpShoot");
  autonChooser.AddOption("CenterFarAmpAuton", "CenterFarAmpAuton");
  // autonChooser.AddOption("Test Auto", testAuto.get());
  // autonChooser.AddOption("Three Note Auto", threeNoteAuto.get());

  SmartDashboard::PutData(&autonChooser);  // send auton selector to Shuffleboard

  odomTrigger.WhileTrue(&repeatOdom); // trigger to handle odom updates from AprilTags

  // DISABLE WHEN NOT IN USE SO DUMMIES DON'T COMMIT MURDER'
  // controller2.B().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("testPath")));
  // controller.B().ToggleOnTrue(&goToSource);
  // controller2.A().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("toSource")));
  // controller2.X().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("turn")));
<<<<<<< Updated upstream
  controller2.RightStick().OnTrue(&orcaToggle);
=======
  controller.RightStick().OnTrue(&orcaToggle);
>>>>>>> Stashed changes
  controller2.LeftStick().OnTrue(&orcaSelectNow);
  // controller.X().ToggleOnTrue(m_drive.FollowPathCommand(pathplanner::PathPlannerPath::fromPathFile("coolPath")));

  controller.RightBumper().OnTrue(&shootNote);
  // controller2.RightBumper().OnTrue(&shootNote);
<<<<<<< Updated upstream
  controller2.RightBumper().OnTrue(std::move(indexPrimed));
=======
  // controller2.RightBumper().OnTrue(std::move(indexPrimed));
>>>>>>> Stashed changes

  // controller2.LeftBumper().ToggleOnTrue(&noteFindRight);

  // controller2.LeftStick().OnTrue(&odomReset);

  controller2.X().OnTrue(&climbPrepare);
  controller2.Y().ToggleOnTrue(&climb);
<<<<<<< Updated upstream
  controller.A().OnTrue(std::move(indexResting));
  controller.Start().ToggleOnTrue(std::move(hunt));
  controller. RightStick().ToggleOnTrue(std::move(driveRotateToNeg70));
=======
  controller.A().OnTrue(std::move(indexPrimed));
  controller.Start().ToggleOnTrue(std::move(hunt));
  // controller.RightStick().ToggleOnTrue(std::move(driveRotateToNeg70));
>>>>>>> Stashed changes

  // controller.X().OnTrue(&targetStage);
  // controller.B().OnTrue(&enableClimbOverride);
  // controller.B().OnFalse(&disableClimbOverride);
  // controller2.X().OnTrue(&targetStage);
  // huntTrigger.WhileTrue(std::move(hunt));

  // noteDetected.OnTrue(&rumbleSecondaryOn);
  // noteDetected.OnFalse(&rumbleSecondaryOff);

  // controller.Start().OnTrue(std::move(indexPrimed));

<<<<<<< Updated upstream
  mainDpadLeft.OnTrue(&targetArbitrary);
  mainDpadUp.OnTrue(&targetSpeaker);
  mainDpadRight.OnTrue(&targetAmp);
  mainDpadDown.OnTrue(&targetNote);
  // controller.B().OnTrue(&reZeroSwerve); // trigger to handle odom updates from AprilTags
  partnerDpadLeft.OnTrue(&targetArbitrary);
  partnerDpadUp.OnTrue(&targetSpeaker);
  partnerDpadRight.OnTrue(&targetAmp);
  partnerDpadDown.OnTrue(&targetNote);
  
  //Turn lock toggles
  controller.LeftStick().OnTrue(&toggleOmegaOverride);
  // controller.RightStick().OnTrue(std::move(rotateTo180));
  controller2.LeftBumper().OnTrue(&toggleOmegaOverride);
  // controller2.B().OnTrue(&tempDisableOmega);
  // controller2.B().OnFalse(&restoreOmega);
  // controller2.A().OnTrue(std::move(trapScore));
  controller2.Back().OnTrue(&targetSource);

=======
  controller.POVLeft().OnTrue(&targetArbitrary);
  controller.POVUp().OnTrue(&targetSpeaker);
  controller.POVRight().OnTrue(&targetAmp);
  controller.POVDown().OnTrue(&targetNote);

  controller2.POVLeft().OnTrue(&targetArbitrary);
  controller2.POVUp().OnTrue(&targetSpeaker);
  controller2.POVRight().OnTrue(&targetAmp);
  controller2.POVDown().OnTrue(&targetNote);
  
  //Turn lock toggles
  controller.LeftStick().OnTrue(&toggleOmegaOverride);
  // controller.RightStick().OnTrue(std::move(rotateTo180));
  controller2.LeftBumper().OnTrue(&toggleOmegaOverride);
  // controller2.B().OnTrue(&tempDisableOmega);
  // controller2.B().OnFalse(&restoreOmega);
  // controller2.A().OnTrue(std::move(trapScore));
  controller2.Back().OnTrue(&targetSource);

>>>>>>> Stashed changes
  driverTurning.OnTrue(&tempDisableOmega);
  driverTurning.OnFalse(&restoreOmega);


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
      autoHuntEnabled = SmartDashboard::GetBoolean("autoHunt", false);  // should be put on co-driver

      // lazy non-command implementation of a field-centric drive toggle
      // if(controller.GetYButtonPressed()) fieldCentric = !fieldCentric;
<<<<<<< Updated upstream
      fieldCentric = !controller.GetYButton();

=======
      fieldCentric = !controller.Y().Get();
>>>>>>> Stashed changes
      // store control inputs for driving
      double x = -controller.GetLeftY();
      double y = -controller.GetLeftX();
      double turnX = controller.GetRightX();
<<<<<<< Updated upstream
      if(controller.GetYButton()) {
=======
      if(controller.Y().Get()) {
>>>>>>> Stashed changes
        x *= -1.0;
        if(TrackingTarget == GlobalConstants::kNote) {
          y = 0.0;
        }
      }
<<<<<<< Updated upstream

=======
      
>>>>>>> Stashed changes
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
    
    // // function to handle intake logic. Should probably be a command
  intake.SetDefaultCommand(frc2::RunCommand(
    [this] {
        double main = controller.GetLeftTriggerAxis() - controller.GetRightTriggerAxis();
        double partner = controller2.GetLeftTriggerAxis() - controller2.GetRightTriggerAxis();
        double power = fabs(main) < 0.25 ? partner * IntakeConstants::kIntakeSpeed : main * IntakeConstants::kIntakeSpeed;
        if(fabs(power) > 0.1) {
          intake.SetState(IntakeConstants::kPowerMode);
          intake.SetPower(power);
        } else {
          autoIntakeEnabled = SmartDashboard::GetBoolean("autoIntake", false);
          if(autoIntakeEnabled) intake.SetState(IntakeConstants::kAutoMode);
          else intake.Off();
        }
    },
  {&intake}));

  // Shooter enable default command
  shooter.SetDefaultCommand(frc2::RunCommand(
    [this] {
<<<<<<< Updated upstream
      if(controller.GetLeftBumper()) shooter.SetState(ShooterConstants::kRpmMode);
=======
      if(controller.LeftBumper().Get()) shooter.SetState(ShooterConstants::kRpmMode);
>>>>>>> Stashed changes
      else shooter.SetState(ShooterConstants::kOff);
    },
  {&shooter}));

  // Arm enable default command
  arm.SetDefaultCommand(frc2::RunCommand(
    [this] {
      arm.SetState(ArmConstants::kAngleMode);
    },
  {&arm}));

  // Arm enable default command
  climber.SetDefaultCommand(frc2::RunCommand(
    [this] {
      // if(controller2.IsConnected()) {
      //   if(controller2.GetAButton()) {
      //     double angle = controller2.GetLeftY() * ClimbConstants::kArmDegreeMax;
      //     SmartDashboard::PutNumber("Climb Position", angle);
      //     climber.SetTargetAngle(angle);
      //   }
      // }
    },
  {&climber}));

  // command to trigger odom updates from limelight AprilTag data
  limelight.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(!limelight.IsTarget()) {
        validTag = false; // if there is no tag detected or the tag is very small don't update odom!
      } else {
        if(limelight.GetTargetArea() > 0.0) {
          validTag = true;  // positional data most likely good. Update odom! 
        }
      }
        frc::SmartDashboard::PutBoolean("tagDetected", validTag); // flashes green to tell drivers odom has been updated
        tagOverrideDisable = frc::SmartDashboard::GetBoolean("detectorOverride", false);

    },
  {&limelight}));

  led.SetDefaultCommand(frc2::RunCommand(
    [this] {
<<<<<<< Updated upstream
      if(controller2.GetBackButtonPressed()) {
        climbed = !climbed;
      }
=======
>>>>>>> Stashed changes
      if(climbed) {
        led.SetPower(LEDConstants::kClimbedPreset);
      } else if(TrackingTarget == GlobalConstants::kNote){
        if(shooter.IsNoteIndexed()){
          led.SetPower(LEDConstants::kNoteIndexed);
        }
        else{
        led.SetPower(LEDConstants::kNoteAutoAlign);
        }
      } else {
        // Add logic for idle/shooter lighting presets
        bool useShooterLighting = false;
        auto coord = m_drive.GetPose().Translation();
        if(coord.X() > GlobalConstants::kShootingBox[0].X() && coord.X() < GlobalConstants::kShootingBox[1].X()) {
          if(coord.Y() > GlobalConstants::kShootingBox[0].Y() && coord.Y() < GlobalConstants::kShootingBox[1].Y()) {
            useShooterLighting = true;
          }
        }
        if(TrackingTarget == GlobalConstants::kSpeaker && useShooterLighting) {
       
          // Add shooter preset(s)
          bool linedUp = m_drive.IsAtTarget() && arm.IsAtTarget() && shooter.IsAtTarget() && limelight.IsTarget();

          if(linedUp) led.SetPower(LEDConstants::kShootReadyPreset);
          else led.SetPower(LEDConstants::kShootNotReadyPreset);          
        } else {
          // Add idle preset
          led.SetPower(LEDConstants::kIdlePreset);
        }
      }
    },
  {&led}));

}

void RobotContainer::SetDriveBrakes(bool state) {
  m_drive.SetBrakeMode(state);
  // elevator.SetBrakeMode(state);
  // arm.SetBrakeMode(state);
  // intake.SetBrakeMode(state);
  //Ahlgrims mom is attractive
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

void RobotContainer::SetRecording(bool state) {
  jetson.SetRecording(state);
}

void RobotContainer::SetAutoIndex(bool state) {
  shooter.SetIndexerAuto(state);
}

// return selected auton routine from Shuffleboard
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto string = autonChooser.GetSelected();
  // m_drive.ResetOdometry(pathplanner::PathPlannerAuto::getStartingPoseFromAutoFile(string));
<<<<<<< Updated upstream
  return pathplanner::PathPlannerAuto(string).ToPtr();
=======
  // return pathplanner::PathPlannerAuto(string).ToPtr();
>>>>>>> Stashed changes
}