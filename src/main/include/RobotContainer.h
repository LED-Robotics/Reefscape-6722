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
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include "units/angle.h"

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include "frc/motorcontrol/Spark.h"

#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>


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

  frc2::Command* GetAutonomousCommand();

  /**
   * Set the brake mode of most robot motors.
   */  
  void SetDriveBrakes(bool state);
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

 private:

  // The driver's controller
  frc2::CommandXboxController controller{OIConstants::kDriverControllerPort};
  // The partner controller
  frc2::CommandXboxController controller2{OIConstants::kCoDriverControllerPort};

  //Blinkin LED controller
  Spark blinkinLeftChassis{BlinkinConstants::kBlinkinLeftChassisPort};
  Spark blinkinRightChassis{BlinkinConstants::kBlinkinRightChassisPort};
  Spark blinkinLeftShooter{BlinkinConstants::kBlinkinLeftShooterPort};
  Spark blinkinRightShooter{BlinkinConstants::kBlinkinRightShooterPort};

  int TrackingTarget = GlobalConstants::kArbitrary;

  // The robot's subsystems

  ctre::phoenix6::Orchestra orchestra;

  DriveSubsystem m_drive{&TrackingTarget, &orchestra};

  // DriveSubsystem m_drive{&TrackingTarget};

  // std::function<units::length::meter_t()> distToTarget{[this]() { 
  //     return m_drive.GetDistToTarget();
  // }};
    
  // ShooterSubsystem shooter{distToTarget, &TrackingTarget, &orchestra};

  // ArmSubsystem arm{distToTarget, &TrackingTarget, &orchestra};

  // ClimbSubsystem climber{&arm, distToTarget, &TrackingTarget, &orchestra};

  // IntakeSubsystem intake{&orchestra};

  // needs a ref to the arm so it can determine its angle relative to the floor
  // IntakeSubsystem intake{&arm};

  // robot offset relative to the field as it started
  // used for AprilTag odom updates
  units::degree_t startOffset{180.0};

  // flag to drive using field-centric positions
  bool fieldCentric = true;

  bool omegaOverride = false;

  bool validTag = false;

  bool tagOverrideDisable = false;

  bool resetting = false;

  int currentTarget = 0;

  frc2::SequentialCommandGroup zeroSwerve{frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] {
      m_drive.ZeroSwervePosition();
    }, {}),
    frc2::WaitCommand(0.125_s)
  )};




  // frc2::InstantCommand zeroSwerve{[this] {
  //   m_drive.ZeroSwervePosition();
  // }, {}};

  // frc2::InstantCommand climb{[this] {
  //   TrackingTarget = GlobalConstants::kArbitrary;
  //   SmartDashboard::PutNumber("Arm Position", 5.0);
  //   SmartDashboard::PutNumber("Climb Position", 160.0);
  //   arm.SetTargetAngle(5.0);
  //   climber.SetTargetAngle(160.0);
  // }, {}};

  // Climb Command



  frc2::RepeatCommand reZeroSwerve{std::move(zeroSwerve)};

  // Command to repetitively call odom update
  // frc2::RepeatCommand zeroSwerve{reZeroSwerve.toPtr()};

  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { return validTag; }};
  frc2::Trigger zeroTrigger{[this]() { return resetting; }};

  frc2::InstantCommand toggleOmegaOverride{[this] { 
      omegaOverride = !omegaOverride;
      m_drive.SetOmegaOverride(omegaOverride);
    }, {}
  };

  frc2::CommandPtr autonOdomSet{frc2::InstantCommand ([this]{
      m_drive.ResetOdometry(AutonConstants::kDefaultStartingPose);
    },{}
  ).ToPtr()};

  frc2::CommandPtr testInstantCommand{frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
        controller.SetRumble(GenericHID::RumbleType::kBothRumble, 1.0);
      }, {}), 
      frc2::WaitCommand(0.375_s),
      frc2::InstantCommand([this]{
        controller.SetRumble(GenericHID::RumbleType::kBothRumble, 0.0);
      }, {})
    ).ToPtr()};

  frc2::InstantCommand targetArbitrary{[this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
    }, {}
  };

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

  frc::SendableChooser<int> musicalSelector;
  frc2::InstantCommand orcaSelectNow{[this] {
      int music_selected = musicalSelector.GetSelected();
      switch (music_selected) {
        case 1:
          orchestra.LoadMusic("Maestro_Musical/cantina.chrp");
          break;
        case 2:
          orchestra.LoadMusic("Maestro_Musical/CynthiaTheme3492922.chrp");
          break;
        case 3:
          orchestra.LoadMusic("Maestro_Musical/Echo1547606.chrp");
          break;
        case 4:
          orchestra.LoadMusic("Maestro_Musical/FNAF6_3735567.chrp");
          break;
        case 5:
          orchestra.LoadMusic("Maestro_Musical/GustyGardenVGMUSIC.chrp");
          break;
        case 6:
          orchestra.LoadMusic("Maestro_Musical/ImperialMarch2178760.chrp");
          break;
        case 7:
          orchestra.LoadMusic("Maestro_Musical/Jevil3661983.chrp");
          break;
        case 8:
          orchestra.LoadMusic("Maestro_Musical/MarbleHillVGMusic.chrp");
          break;
        case 9:
          orchestra.LoadMusic("Maestro_Musical/OMORI86_3174948.chrp");
          break;
        case 10:
          orchestra.LoadMusic("Maestro_Musical/ReconstructingScience2809779.chrp");
          break;
        case 11:
          orchestra.LoadMusic("Maestro_Musical/TADC3699908.chrp");
          break;
        case 12:
          orchestra.LoadMusic("Maestro_Musical/Viva1717136.chrp");
          break;
        case 13:
          orchestra.LoadMusic("Maestro_Musical/BohemianRhapsodyFREEMIDI.chrp");
          break;
        case 14:
          orchestra.LoadMusic("Maestro_Musical/BadApple3820096.chrp");
          break;
        case 15:
          orchestra.LoadMusic("Maestro_Musical/FlyMoon3600404.chrp");
          break;
        case 16:
          orchestra.LoadMusic("Maestro_Musical/DisassemblyRequired3252559.chrp");
          break;
        case 17:
          orchestra.LoadMusic("Maestro_Musical/GreenHill3698170.chrp");
          break;
        case 18:
          orchestra.LoadMusic("Maestro_Musical/KillerQueen3777783.chrp");
          break;
        case 19:
          orchestra.LoadMusic("Maestro_Musical/Macarena2880456.chrp");
          break;
        case 20:
          orchestra.LoadMusic("Maestro_Musical/MasterKohga3463326.chrp");
          break;
        case 21:
          orchestra.LoadMusic("Maestro_Musical/Megalotrousle2155665.chrp");
          break;
        case 22:
          orchestra.LoadMusic("Maestro_Musical/SlippingThroughMyFingersFREEMIDI.chrp");
          break;
        case 23:
          orchestra.LoadMusic("Maestro_Musical/SMB3AirshipVGMusic.chrp");
          break;
        case 24:
          orchestra.LoadMusic("Maestro_Musical/SongOfStorms3780744.chrp");
          break;
        case 25:
          orchestra.LoadMusic("Maestro_Musical/StillStanding3696063.chrp");
          break;
        case 26:
          orchestra.LoadMusic("Maestro_Musical/WaitOfWorld3312229.chrp");
          break;
        case 27:
          orchestra.LoadMusic("Maestro_Musical/Asgore1633865.chrp");
          break;
        case 28:
          orchestra.LoadMusic("Maestro_Musical/OMORI125_3448462.chrp");
          break;
        case 29:
          orchestra.LoadMusic("Maestro_Musical/BeatIt3563264.chrp");
          break;
        case 30:
          orchestra.LoadMusic("Maestro_Musical/BuddyHolly2294759.chrp");
          break;
        case 31:
          orchestra.LoadMusic("Maestro_Musical/UNDERTALE85_3359661.chrp");
          break;
        case 32:
          orchestra.LoadMusic("Maestro_Musical/LivinOnAPrayer2528149.chrp");
          break;
        case 33:
          orchestra.LoadMusic("Maestro_Musical/LostWoods782496.chrp");
          break;
        case 34:
          orchestra.LoadMusic("Maestro_Musical/Caribbean3586082.chrp");
          break;
        case 35:
          orchestra.LoadMusic("Maestro_Musical/PokemonTrainerBattle3557578.chrp");
          break;
        case 36:
          orchestra.LoadMusic("Maestro_Musical/Scary2774320.chrp");
          break;
        case 37:
          orchestra.LoadMusic("Maestro_Musical/SM64_3585783.chrp");
          break;
        case 38:
          orchestra.LoadMusic("Maestro_Musical/NeverGonna59740.chrp");
          break;
        case 39:
          orchestra.LoadMusic("Maestro_Musical/RushEFanchen.chrp");
          break;
        case 40:
          orchestra.LoadMusic("Maestro_Musical/StickBug.chrp");
          break;
        default:
          orchestra.LoadMusic("Maestro_Musical/cantina.chrp");
      }
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
  //     shooter.OffsetIndexer(-3.0);
  //   }, {}),
  //   IndexerSet(ShooterConstants::kIndexerHolding, &shooter)
  //   ).ToPtr()
  // };

  // frc2::CommandPtr indexResting{IndexerSet(ShooterConstants::kIndexerResting, &shooter).ToPtr()};
  // frc2::CommandPtr indexPrimed{IndexerSet(ShooterConstants::kIndexerPrimed, &shooter).ToPtr()};

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
  

  // LED control Commands
  frc2::InstantCommand SetBlinkinAButton{[this] {
    blinkinLeftChassis.Set(.41); /*End to End; Color 1 (Purple) and Color 2 (Yellow)*/ },
                                        {}};

  frc2::InstantCommand SetBlinkinLeftBumper{[this] {
    blinkinLeftChassis.Set(.07); /*Fast Heartbeat; Color 1 (Purple)*/ },
                                        {}};

  frc2::InstantCommand SetBlinkinRightBumper{[this] {
    blinkinLeftChassis.Set(.27); /*Fast Heartbeat; Color 2 (Yellow)*/; },
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

  // The chooser for the autonomous routines

  frc2::CommandPtr testAuto{pathplanner::PathPlannerAuto("ActualAuto")};

  frc::SendableChooser<frc2::Command*> autonChooser;

};