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

#include "GlobalConstants.h"
#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/LEDSubsystem/LEDSubsystem.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "iostream"
#include "frc/motorcontrol/Spark.h"
// #include <pathplanner/lib/commands/PathPlannerAuto.h>
// #include <ctre/Phoenix.h>

// #include <pathplanner/lib/commands/FollowPathHolonomic.h>


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
  
  JetsonSubsystem jetson{};

  DriveSubsystem m_drive{&jetson, &TrackingTarget};

  std::function<units::length::meter_t()> distToTarget{[this]() { 
      return m_drive.GetDistToTarget();
  }};

  LEDSubsystem led{};

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

  frc2::CommandPtr autonOdomSet{frc2::InstantCommand ([this]{
      m_drive.ResetOdometry(AutoConstants::kDefaultStartingPose);
    },{&m_drive}
  ).ToPtr()};

  frc2::InstantCommand odomReset{[this]{
      m_drive.ResetOdometry({7.5_m, 4.3_m, 180_deg});
    },{}};

  // Command to repetitively call odom update
  frc2::RepeatCommand repeatOdom{std::move(updateOdometry)};

  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { return validTag && !tagOverrideDisable; }};

  frc2::InstantCommand toggleFieldCentric{[this] {
      fieldCentric = !fieldCentric;
    }, {}
  };

  frc2::InstantCommand toggleOmegaOverride{[this] { 
      omegaOverride = !omegaOverride;
      m_drive.SetOmegaOverride(omegaOverride);
    }, {}
  };

  frc2::Trigger driverTurning{[this]() {
      return abs(controller.GetRightX()) > DriveConstants::kTurnDeadzone && !controller2.A().Get();
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