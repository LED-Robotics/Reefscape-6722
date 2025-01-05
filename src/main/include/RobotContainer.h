// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/DriverStation.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include "units/angle.h"

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/TrapSubsystem.h"
#include "subsystems/LimelightSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include "commands/IndexerSet.h"
#include "commands/ArmSet.h"
#include "commands/ClimbSet.h"
#include "commands/TrapSet.h"
#include "commands/AlignNote.h"
#include "commands/HuntNote.h"
#include "commands/TargetSpeaker.h"
#include "commands/RotateToAngle.h"
#include "commands/IntakeNote.h"
#include "commands/TurnToNote.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "iostream"
#include "frc/motorcontrol/Spark.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <ctre/Phoenix.h>

#include <pathplanner/lib/commands/FollowPathHolonomic.h>


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  /**
   * Set the brake mode of most robot motors.
   */  
  void SetDriveBrakes(bool state);

  void DisableTagTracking();

  void EnableTagTracking();
  /**
   * Zero swerve drive.
   */  
  void ZeroSwerve();
  /**
   * Function to handle the IntakeSubsystem's control logic.
   */  
  void HandleIntake();
  /**
   * Set the state of the DriveSubsystem's SlewRateLimiters.
   */  
  void SetSlew(bool state);

  void SetRecording(bool state);

  void SetAutoIndex(bool state);


 private:

  // The driver's controller
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // The partner controller
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  int TrackingTarget = GlobalConstants::kArbitrary;

  // The robot's subsystems

  ctre::phoenix6::Orchestra orchestra;

  ctre::phoenix::motorcontrol::can::WPI_VictorSPX trapOpener{21};

  LimelightSubsystem limelight{"limelight"};
  
  JetsonSubsystem jetson{};

  DriveSubsystem m_drive{&limelight, &jetson, &TrackingTarget, &orchestra};

  std::function<units::length::meter_t()> distToTarget{[this]() { 
      return m_drive.GetDistToTarget();
  }};
    
  ShooterSubsystem shooter{distToTarget, &TrackingTarget, &orchestra};

  ArmSubsystem arm{distToTarget, &TrackingTarget, &orchestra};

  ClimbSubsystem climber{&orchestra};

  IntakeSubsystem intake{&shooter, &orchestra};

  TrapSubsystem claw{&arm, &orchestra};

  LEDSubsystem led{};

  // needs a ref to the arm so it can determine its angle relative to the floor
  // IntakeSubsystem intake{&arm};

  // LimelightSubsystem armLimelight{"limelight-arm"};

  // robot offset relative to the field as it started
  // used for AprilTag odom updates
  units::degree_t startOffset{180.0};

  // flag to drive using field-centric positions
  bool fieldCentric = true;

  bool omegaOverride = false;

  bool yOverride = false;

  bool validTag = false;

  bool tagOverrideDisable = false;

  bool autoHuntEnabled = false;

  bool autoIntakeEnabled = false;

  bool climbed = false;

  int currentTarget = 0;

  int omegaTempDisabled = 0;

  // update odom based on Limelight's AprilTag megabotpose estimation
  frc2::SequentialCommandGroup updateOdometry{
    frc2::InstantCommand([this] {
      if(!tagOverrideDisable) {
        m_drive.ResetFromLimelight();
      }
    }, {}),
    frc2::WaitCommand(5.0_s)
  };

  frc2::InstantCommand climbPrepare{[this] {
    TrackingTarget = GlobalConstants::kArbitrary;//April Tag stuff to target the stage
    SmartDashboard::PutNumber("Arm Position", 30.0);
    // SmartDashboard::PutNumber("Claw Position", 30.0);
    SmartDashboard::PutNumber("Climb Position", ClimbConstants::kPreparePos);
    arm.SetTargetAngle(30.0);
    climber.SetTargetPosition(ClimbConstants::kPreparePos);
    // claw.SetTargetAngle(30.0);
    // claw.SetState(TrapConstants::kAngleMode);
  }, {}};

  frc2::InstantCommand climb{[this] {
    TrackingTarget = GlobalConstants::kArbitrary;//April Tag stuff to target the stage
    SmartDashboard::PutNumber("Arm Position", 30.0);
    // SmartDashboard::PutNumber("Claw Position", 30.0);
    SmartDashboard::PutNumber("Climb Position", ClimbConstants::kClimbedPos);
    arm.SetTargetAngle(30.0);
    climber.SetTargetPosition(ClimbConstants::kClimbedPos);
    // claw.SetTargetAngle(30.0);
    // claw.SetState(TrapConstants::kAngleMode);
  }, {}};

  // frc2::InstantCommand climbPrepare{[this] {
  //   TrackingTarget = GlobalConstants::kArbitrary;//April Tag stuff to target the stage
  //   SmartDashboard::PutNumber("Arm Position", 70.0);
  //   SmartDashboard::PutNumber("Climb Position", 27.0);
  //   // SmartDashboard::PutNumber("Claw Position", 30.0);
  //   arm.SetTargetAngle(70.0);
  //   climber.SetTargetPosition(27.0);
  //   // claw.SetTargetAngle(30.0);
  //   // claw.SetState(TrapConstants::kAngleMode);
  // }, {}};

  // Climb Command                                                                                                                                                                PINEAPPLE
  // frc2::CommandPtr climb{frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([this] { 
  //       TrackingTarget = GlobalConstants::kArbitrary;
  //       // claw.SetState(TrapConstants::kAngleMode);
  //     }, {}
  //   ),
  //   frc2::ParallelCommandGroup(
  //     ClimbSet(158.0, &climber),
  //     ArmSet(30.0, &arm)
  //   ),
  //   frc2::WaitCommand(0.5_s),
  //   ArmSet(1.0, &arm),
  //   frc2::WaitCommand(1.0_s),
  //   ClimbSet(230.0, &climber),
  //   frc2::WaitCommand(0.5_s),
  //   ClimbSet(100.0, &climber)
  //   // frc2::WaitCommand(1_s),
  //   // frc2::ParallelCommandGroup(
  //   //   ClimbSet(40.0, &climber),
  //   //   ArmSet(78.0, &arm)
  //   // )
  //   ).ToPtr()//Fuck you tristan
  // };

  frc2::CommandPtr trapScore{frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { 
        trapOpener.Set(1.0);
      }, {}),
      frc2::WaitCommand(2.0_s),
      frc2::InstantCommand([this] { 
          trapOpener.Set(0.0);
        }, {})
    ).ToPtr()
  };

  frc2::CommandPtr autonOdomSet{frc2::InstantCommand ([this]{
      m_drive.ResetOdometry(AutoConstants::kDefaultStartingPose);
    },{&m_drive}
  ).ToPtr()};

  frc2::InstantCommand odomReset{[this]{
      m_drive.ResetOdometry({7.5_m, 4.3_m, 180_deg});
    },{}};



  // AutoNote Command
  frc2::CommandPtr hunt{frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { 
          intake.SetState(IntakeConstants::kAutoMode);
        }, {&intake}),
      AlignNote(&m_drive, &jetson),
      HuntNote(&m_drive, &intake, &jetson),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.7_s),
        IntakeNote(&intake, &shooter)
      )
    ).ToPtr()
  };

  // TurnToNote noteFindLeft{220_deg_per_s, &m_drive, &jetson};
  // TurnToNote noteFindRight{-220_deg_per_s, &m_drive, &jetson};

  // I am a lazy hack
  // AutoNote Command
  frc2::CommandPtr autoHunt{frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { 
          intake.SetState(IntakeConstants::kAutoMode);
        }, {&intake}),
      AlignNote(&m_drive, &jetson),
      HuntNote(&m_drive, &intake, &jetson)
    ).ToPtr()
  };

  frc2::CommandPtr ensureIndexed{IntakeNote(&intake, &shooter)};

  // Command to repetitively call odom update
  frc2::RepeatCommand repeatOdom{std::move(updateOdometry)};
  // frc2::RepeatCommand zeroSwerve{reZeroSwerve.toPtr()};

  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { return validTag && !tagOverrideDisable; }};

  frc2::Trigger huntTrigger{[this]() { 
      double x = abs(controller.GetLeftY());
      double y = abs(controller.GetLeftX());
      double theta = abs(controller.GetRightX());
      bool driveInactive = x < DriveConstants::kDriveDeadzone;
      driveInactive &=  y < DriveConstants::kDriveDeadzone;
      driveInactive &=  theta < DriveConstants::kTurnDeadzone;
      return autoHuntEnabled && jetson.IsTarget() && driveInactive && !shooter.IsNoteIndexed();
      // hunt note is auto-hunting is enabled, we see a note, 
      // the driver is not driving, and a note isn't in the shooter already
    }
  };

  frc2::Trigger noteDetected{[this]() { 
      return !shooter.IsNoteIndexed() && jetson.IsTarget() && jetson.GetTargetSize() > 0.03;
    }
  };

  frc2::InstantCommand toggleOmegaOverride{[this] { 
      omegaOverride = !omegaOverride;
      m_drive.SetOmegaOverride(omegaOverride);
    }, {}
  };

  frc2::InstantCommand enableClimbOverride{[this] { 
      m_drive.SetOmegaOverride(true);
      m_drive.SetYOverride(true);
    }, {}
  };

  frc2::InstantCommand disableClimbOverride{[this] { 
      m_drive.SetOmegaOverride(omegaOverride);
      m_drive.SetYOverride(yOverride);
    }, {}
  };

  frc2::Trigger driverTurning{[this]() {
      return abs(controller.GetRightX()) > DriveConstants::kTurnDeadzone && !controller2.GetAButton();
    }
  };

  frc2::InstantCommand tempDisableOmega{[this] { 
      omegaTempDisabled++;
      m_drive.SetOmegaOverride(false);
    }, {}
  };

  frc2::InstantCommand restoreOmega{[this] { 
      omegaTempDisabled--;
      if(omegaTempDisabled <= 0) {
        omegaTempDisabled = 0;
        m_drive.SetOmegaOverride(omegaOverride);
      }
    }, {}
  };

  frc2::InstantCommand targetArbitrary{[this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
    }, {}
  };

  //Regular pointer shit
  frc2::InstantCommand targetSpeaker{[this] { 
      TrackingTarget = GlobalConstants::kSpeaker;
    }, {}
  };

  frc2::InstantCommand targetAmp{[this] { 
      TrackingTarget = GlobalConstants::kAmp;
    }, {}
  };

  frc2::InstantCommand targetSource{[this] { 
      TrackingTarget = GlobalConstants::kSource;
    }, {}
  };

  frc2::InstantCommand targetNote{[this] { 
      TrackingTarget = GlobalConstants::kNote;
    }, {}
  };

  frc2::InstantCommand targetStage{[this] { 
      TrackingTarget = GlobalConstants::kStage;
    }, {}
  };

  frc2::CommandPtr rotateTo180{RotateToAngle(&TrackingTarget, {180_deg}, &m_drive)};
  frc2::CommandPtr rotateTo90{RotateToAngle(&TrackingTarget, {90_deg}, &m_drive)};
  frc2::CommandPtr rotateToNeg90{RotateToAngle(&TrackingTarget, {-90_deg}, &m_drive)};
  frc2::CommandPtr rotateToNeg70{RotateToAngle(&TrackingTarget, {-70_deg}, &m_drive)};
  frc2::CommandPtr driveRotateToNeg70{RotateToAngle(&TrackingTarget, {-70_deg}, &m_drive)};
  frc2::CommandPtr rotateTo82{RotateToAngle(&TrackingTarget, {82_deg}, &m_drive)};
  frc2::CommandPtr rotateToNeg130{RotateToAngle(&TrackingTarget, {-130_deg}, &m_drive)};

  //Unique pointer shit
  frc2::CommandPtr autonSpeakerTarget{frc2::InstantCommand([this] { 
        TrackingTarget = GlobalConstants::kSpeaker;
        m_drive.SetOmegaOverride(true);
      }, {}
    ).ToPtr()
  };

  frc2::CommandPtr driveOff{frc2::InstantCommand([this] { 
        m_drive.Drive({0_mps, 0_mps, 0_deg_per_s});
      }, {&m_drive}
    ).ToPtr()
  };

  frc2::CommandPtr autonTrackingDisable{frc2::InstantCommand([this] { 
        TrackingTarget = GlobalConstants::kArbitrary;
        m_drive.SetOmegaOverride(false);
      }, {&m_drive}
    ).ToPtr()
  };

  frc2::CommandPtr lineupSpeaker{TargetSpeaker(&TrackingTarget, &m_drive, &arm, &shooter, &limelight).ToPtr()};

  frc::SendableChooser<const char*> musicalSelector;
  
  frc2::InstantCommand orcaSelectNow{[this] {
    orchestra.LoadMusic(musicalSelector.GetSelected());
    }, {}
  };

  bool orcaState = false;
  frc2::InstantCommand orcaToggle{[this] { 
      orcaState = !orcaState;
      if(orcaState) {
        orchestra.Play();
      } else {
        orchestra.Stop();
      }
    }, {}
  };

  // Index Command
  // frc2::CommandPtr shootNote{frc2::SequentialCommandGroup(
  //   IndexerSet(ShooterConstants::kIndexerKicking, &shooter),
  //   frc2::WaitCommand(0.5_s),
  //   frc2::InstantCommand([this] { 
  //     shooter.OffsetIndexer(((int)ShooterConstants::kIndexerKicking));
  //   }, {}),
  //   IndexerSet(ShooterConstants::kIndexerHolding, &shooter)
  //   ).ToPtr()
  // };

  frc2::SequentialCommandGroup shootNote{
    IndexerSet(ShooterConstants::kIndexerKicking, &shooter),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] { 
      shooter.OffsetIndexer(((int)ShooterConstants::kIndexerKicking));
    }, {}),
    IndexerSet(ShooterConstants::kIndexerHolding, &shooter)
  };

  frc2::CommandPtr autonShoot{frc2::SequentialCommandGroup(
    IndexerSet(ShooterConstants::kIndexerKicking, &shooter),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this] { 
      shooter.OffsetIndexer(((int)ShooterConstants::kIndexerKicking));
    }, {}),
    IndexerSet(ShooterConstants::kIndexerHolding, &shooter)
    ).ToPtr()
  };

  frc2::CommandPtr indexResting{IndexerSet(ShooterConstants::kIndexerResting, &shooter).ToPtr()};
  frc2::CommandPtr indexPrimed{IndexerSet(ShooterConstants::kIndexerPrimed, &shooter).ToPtr()};

  // Triggers for main and partner D-PAD positions

  frc2::Trigger mainDpadUp{[this]() { return controller.GetPOV() == 0; }};
  frc2::Trigger mainDpadRight{[this]() { return controller.GetPOV() == 90; }};
  frc2::Trigger mainDpadDown{[this]() { return controller.GetPOV() == 180; }};
  frc2::Trigger mainDpadLeft{[this]() { return controller.GetPOV() == 270; }};

  frc2::Trigger partnerDpadUp{[this]() { return controller2.GetPOV() == 0; }};
  frc2::Trigger partnerDpadRight{[this]() { return controller2.GetPOV() == 90; }};
  frc2::Trigger partnerDpadDown{[this]() { return controller2.GetPOV() == 180; }};
  frc2::Trigger partnerDpadLeft{[this]() { return controller2.GetPOV() == 270; }};

  // funny rumble Commands
  frc2::InstantCommand rumblePrimaryOn{[this] { controller.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOn{[this] { controller2.SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {}};
  
  frc2::InstantCommand rumblePrimaryOff{[this] { controller.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};

  frc2::InstantCommand rumbleSecondaryOff{[this] { controller2.SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {}};
  /**
   * Find whether the robot is on the blue or red alliance as set by the FMS/DriverStation.
   *
   * @return A bool for if the robot is on the blue alliance
   */
  bool IsBlue();
  /**
   * Return one of two Commands based on whether a partner controller is connected.
   *
   * @return The appropriate Command* based on partner controller status
   */
  frc2::Command* HandlePartnerCommands(frc2::Command* solo, frc2::Command* partner);
  /**
   * Return a pointer to an empty Command that will do nothing when run.
   *
   * @return A Command* to an empty Command
   */
  frc2::Command* GetEmptyCommand();

  // frc2::CommandPtr testAuto{pathplanner::PathPlannerAuto("TestAuto").ToPtr()};
  // frc2::CommandPtr threeNoteAuto{pathplanner::PathPlannerAuto("ThreeNoteAuto").ToPtr()};

  // The chooser for the autonomous routines
  frc::SendableChooser<std::string> autonChooser;
  // frc::SendableChooser<frc2::Command*> autonChooser;
  frc2::Command* currentAuton;
};