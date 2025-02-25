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
#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"
#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"
#include "subsystems/ClimbSubsystem/ClimbSubsystem.h"
#include "subsystems/IntakeSubsystem/IntakeSubsystem.h"
#include "subsystems/FunnelSubsystem/FunnelSubsystem.h"
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

  static struct KinematicsPoses {
    units::length::meter_t cascadePose;
    //Angle of wrist
    //More will be added
  } kinematicsInfo;
  /**
   * Return the command pointer that sets all subsystem kinematics.
   */
  frc2::CommandPtr SetAllKinematics(KinematicsPoses kinInfoRef);

  /**
   * Return the command pointer to the autonomous command. 
   */
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
  
  // Starting tracking target
  int TrackingTarget = GlobalConstants::kCoralMode;
  
  // The robot's subsystems
  AlgaeSubsystem algae{};

  JetsonSubsystem jetson{};

  DriveSubsystem m_drive{&jetson, &TrackingTarget};
  
  CascadeSubsystem cascade{};

  // ClimbSubsystem climb{};

  IntakeSubsystem intake{};

  // FunnelSubsystem funnel{};

  // FloorSubsystem floor{};

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

  int currentTarget = 0;

  int omegaTempDisabled = 0;

  // update odom based on Nvdia Jetson estimation
  frc2::CommandPtr updateOdometry {
    frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] {
        // if(!tagOverrideDisable) {
          m_drive.ResetFromJetson();
        // }
      }, {}),
      frc2::cmd::Wait(5.0_s)
    )};

  frc2::CommandPtr autonOdomSet{frc2::cmd::RunOnce([this]{
      m_drive.ResetOdometry(AutoConstants::kDefaultStartingPose);
    },{&m_drive}
  )};

  frc2::CommandPtr odomReset{frc2::cmd::RunOnce([this]{
      m_drive.ResetOdometry({7.5_m, 4.3_m, 180_deg});
  },{})};

  // Command to repetitively call odom update
  frc2::CommandPtr repeatOdom{std::move(updateOdometry).Repeatedly()};

  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { 
    return jetson.IsPoseAvailable(); }};

  frc2::CommandPtr toggleFieldCentric{frc2::cmd::RunOnce([this] {
      fieldCentric = !fieldCentric;
    }, {})
  };

  frc2::CommandPtr toggleOmegaOverride{frc2::cmd::RunOnce([this] { 
      omegaOverride = !omegaOverride;
      m_drive.SetOmegaOverride(omegaOverride);
    }, {})
  };

  frc2::Trigger mainBack{controller.Back()};
  frc2::Trigger mainDpadUp{controller.POV(0)};
  frc2::Trigger mainDpadDown{controller.POV(180)};
  frc2::Trigger mainDpadLeft{controller.POV(270)};
  frc2::Trigger mainDpadRight{controller.POV(90)};

  frc2::Trigger driverTurning{[this]() {
      return abs(controller.GetRightX()) > DriveConstants::kTurnDeadzone && !controller2.A().Get();
    }
  };

  frc2::CommandPtr targetArbitrary{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
    }, {})
  };
  
  frc2::CommandPtr targetCoral{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kCoralMode;
    }, {})
  };
  
  frc2::CommandPtr targetAlgae{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kAlgaeMode;
    }, {})
  };

  frc2::CommandPtr driveOff{frc2::cmd::RunOnce([this] { 
      m_drive.Drive({0_mps, 0_mps, 0_deg_per_s});
    }, {&m_drive})
  };

  frc2::CommandPtr autonTrackingDisable{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
      m_drive.SetOmegaOverride(false);
    }, {&m_drive})  
  };

  // funny rumble Commands
  frc2::CommandPtr rumblePrimaryOn{frc2::cmd::RunOnce([this] { controller.GetHID().SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {})};

  frc2::CommandPtr rumbleSecondaryOn{frc2::cmd::RunOnce([this] { controller2.GetHID().SetRumble(GenericHID::kBothRumble, 1.0); },
                                        {})};
  
  frc2::CommandPtr rumblePrimaryOff{frc2::cmd::RunOnce([this] { controller.GetHID().SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {})};

  frc2::CommandPtr rumbleSecondaryOff{frc2::cmd::RunOnce([this] { controller2.GetHID().SetRumble(GenericHID::kBothRumble, 0.0); },
                                        {})};
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

  frc2::CommandPtr SetMultijoint(units::length::meter_t cascadeHeight, units::angle::degree_t algaeAngle);

  // The chooser for the autonomous routines
  frc::SendableChooser<std::string> autonChooser;
  // frc::SendableChooser<frc2::Command*> autonChooser;
  frc2::Command* currentAuton;
};
