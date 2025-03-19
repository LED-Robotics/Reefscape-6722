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
#include <frc/Timer.h>
#include <math.h>

#include "GlobalConstants.h"
#include "subsystems/AlgaeSubsystem/AlgaeSubsystem.h"
#include "subsystems/DriveSubsystem/DriveSubsystem.h"
#include "subsystems/CascadeSubsystem/CascadeSubsystem.h"
#include "subsystems/ClimbSubsystem/ClimbSubsystem.h"
#include "subsystems/CoralSubsystem/CoralSubsystem.h"
#include "subsystems/LEDSubsystem/LEDSubsystem.h"
#include "subsystems/PivotSubsystem/PivotSubsystem.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "iostream"
#include "frc/motorcontrol/Spark.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
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

  enum MLLabels {
    Algae,
    Coral,
    Reef
  };

  enum ReefTargetStates {
    Unset,
    Left,
    Right
  };

  struct KinematicsPose {
    units::length::meter_t cascadePose;
    units::angle::degree_t pivotAngle;
  };

  frc::AprilTagFieldLayout field = frc::AprilTagFieldLayout::LoadField(AprilTagField::k2025ReefscapeWelded);

  /**
   * Return the command pointer to the autonomous command. 
   */
  frc2::Command* GetAutonomousCommand();

  void ChangeCoralCamID(int newId);
  void ChangeAprilTagCamID(int newId);

  /**
   * Set the brake mode of most robot motors.
   */  
  void SetDriveBrakes(bool state);

  void ZeroDriveAdjustments();

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
  int CodriverIntakeTarget = GlobalConstants::kAlgaeMode;

  int ReefTarget = 0;
  int ReefHeightLevel = 0;
  int LoadTarget = 0;
  bool rerunThetaSet = false;
  bool autonCoralLoad = false;
  bool autonReefLineup = false;
  
  // The robot's subsystems
  JetsonSubsystem jetson{};

  DriveSubsystem drive{&jetson, &TrackingTarget};
  
  CascadeSubsystem cascade{};

  units::length::meter_t minCoralSweepHeight{0.88_m};
  units::angle::degree_t coralSweepRange[2] = {-210.0_deg, -155.0_deg};

  units::length::meter_t minAlgaeSweepHeight{0.88_m};
  units::angle::degree_t algaeSweepRange[2] = {-60.0_deg, 30.0_deg};

  PivotSubsystem pivot{};

  // ClimbSubsystem climb{};

  CoralSubsystem coral{};

  AlgaeSubsystem algae{};

  LEDSubsystem led{};

  // Kinematics Poses //
  KinematicsPose startingPose{0.9_m, 90.00_deg};
  KinematicsPose loadPose{1.18_m, -129.27_deg};
  KinematicsPose floorIntakePose{0.69_m, -48.56_deg};

  KinematicsPose l1Coral{0.85_m, -145.0_deg};
  KinematicsPose l2Coral{0.86_m, 51.78_deg};
  KinematicsPose l3Coral{1.33_m, 53.16_deg};
  KinematicsPose l4Coral{1.95_m, 60.06_deg};

  KinematicsPose l1Algae{1.0_m, 90_deg};
  KinematicsPose l2Algae{1.0_m, -60.45_deg};
  KinematicsPose l3Algae{1.28_m, -80.2_deg};
  KinematicsPose l4Algae{2.05_m, -192.3_deg};

  // Kinematics Poses //

  // used for AprilTag odom updates
  units::degree_t startOffset{180.0};

  int reefTargetDirection = ReefTargetStates::Unset;

  frc::PIDController xTransAdjust{0.006, 0.0, 0.0003};

  frc::PIDController yTransAdjust{0.006, 0.0, 0.0003};
  frc::PIDController coralAdjust{0.0016137, 0.0, 0.0};
  /*frc::PIDController reefAdjust{0.0019137, 0.0, 0.0};*/
  frc::PIDController reefAdjust{0.0016137, 0.0, 0.0};

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
          drive.ResetFromJetson();
        // }
      }, {}),
      frc2::cmd::Wait(7.0_s)
    )};

  frc2::CommandPtr autonOdomSet{frc2::cmd::RunOnce([this]{
      drive.ResetOdometry(AutoConstants::kDefaultStartingPose);
    },{&drive}
  )};

  frc2::CommandPtr odomReset{frc2::cmd::RunOnce([this]{
      drive.ResetOdometry({7.5_m, 4.3_m, 180_deg});
  },{})};

  // Command to repetitively call odom update
  frc2::CommandPtr repeatOdom{std::move(updateOdometry).Repeatedly()};

  // Trigger odom update on flag
  frc2::Trigger odomTrigger{[this]() { 
    return !tagOverrideDisable && jetson.IsPoseAvailable(); }};

  frc2::CommandPtr toggleFieldCentric{frc2::cmd::RunOnce([this] {
      fieldCentric = !fieldCentric;
    }, {})
  };

  frc2::CommandPtr toggleOmegaOverride{frc2::cmd::RunOnce([this] { 
      omegaOverride = !omegaOverride;
      drive.SetOmegaOverride(omegaOverride);
    }, {})
  };

  frc2::Trigger mainBack{controller.Back()};
  frc2::Trigger mainDpadUp{controller.POV(0)};
  frc2::Trigger mainDpadDown{controller.POV(180)};
  frc2::Trigger mainDpadLeft{controller.POV(270)};
  frc2::Trigger mainDpadRight{controller.POV(90)};

  frc2::Trigger mainDpadUp2{controller2.POV(0)};
  frc2::Trigger mainDpadDown2{controller2.POV(180)};
  frc2::Trigger mainDpadLeft2{controller2.POV(270)};
  frc2::Trigger mainDpadRight2{controller2.POV(90)};

  frc2::Trigger dpad2InteractedWith{[this]() {
      return controller2.GetHID().GetPOV() != -1;
    } 
  }; 

  frc2::Trigger reefTargetChanged{[this]() {
      double magnitude = sqrt(pow(controller2.GetLeftX(), 2) + pow(controller2.GetLeftY(), 2)); // Length of vector for trigger
      return rerunThetaSet || magnitude > 0.50;
    }
  };

  frc2::Trigger driverTurning{[this]() {
      return abs(controller.GetRightX()) > DriveConstants::kTurnDeadzone;
    }
  };

  frc2::CommandPtr targetArbitrary{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
    }, {})
  };
  
   frc2::CommandPtr targetCoral{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kCoralMode;
      rerunThetaSet = true;
    }, {})
  };
  
  frc2::CommandPtr targetAlgae{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kAlgaeMode;
      rerunThetaSet = true;
    }, {})
  };

 frc2::CommandPtr coDriverTargetCoral{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kCoralMode;
      rerunThetaSet = true;
    }, {})
  };
  
  frc2::CommandPtr coDriverTargetAlgae{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kAlgaeMode;
      rerunThetaSet = true;
    }, {})
  };

  frc2::CommandPtr coDriverIntakeCoral{frc2::cmd::RunOnce([this] { 
      CodriverIntakeTarget = GlobalConstants::kCoralMode;
    }, {})
  };
  
  frc2::CommandPtr coDriverIntakeAlgae{frc2::cmd::RunOnce([this] { 
      CodriverIntakeTarget = GlobalConstants::kAlgaeMode;
    }, {})
  };

  frc2::CommandPtr driveOff{frc2::cmd::RunOnce([this] { 
      drive.Drive({0_mps, 0_mps, 0_deg_per_s});
    }, {&drive})
  };

  frc2::CommandPtr autonTrackingDisable{frc2::cmd::RunOnce([this] { 
      TrackingTarget = GlobalConstants::kArbitrary;
      drive.SetOmegaOverride(false);
    }, {&drive})  
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
  
  const frc::Pose2d blueReef[6] = {
    {6.5_m, 4.0_m, {180_deg}}, // Away from driver station, 0_deg
    {5.5_m, 2.25_m, {120_deg}}, // 45_deg
    {3.5_m, 2.25_m, {60_deg}}, // 135_deg
    {2.5_m, 4.0_m, {0_deg}}, // Facing driverstation, 180_deg
    {3.5_m, 5.75_m, {-60_deg}}, // 215_deg
    {5.5_m, 5.75_m, {-120_deg}} //305_deg
  };

    /*{5.85_m, 3.75_m, {-90_deg}}, // Away from driver station, 0_deg*/
  const frc::Pose2d autonReef[6] = {
    {5.85_m, 4.25_m, {-90_deg}}, // Away from driver station, 0_deg
    {5.026_m, 2.724_m, {-150.0_deg}}, // 45_deg
    {3.524_m, 2.913_m, {150.0_deg}}, // 135_deg
    {2.5_m, 4.0_m, {0_deg}}, // Facing driverstation, 180_deg
    {3.5_m, 5.75_m, {-60_deg}}, // 215_deg
    {5.5_m, 5.75_m, {-120_deg}} //305_deg
  };

  const frc::Pose2d redReef[6] = {
    {11.0_m, 3.85_m, {90_deg}}, 
    {12.0_m, 5.75_m, {30_deg}}, 
    {14.1_m, 5.75_m, {-30_deg}}, 
    {15.1_m, 4.0_m, {-90_deg}}, 
    {14.15_m, 2.25_m, {-150_deg}}, 
    {12.0_m, 2.25_m, {150_deg}}
  };

  const frc::Pose2d blueLoading[2] = {
    {11.0_m, 4.0_m, {-54_deg}}, 
    {12.0_m, 5.75_m, {54_deg}}
  };

  const frc::Pose2d autonLoading[2] = {
    {1.218_m, 1.045_m, {145.0_deg}}, 
    {1.218_m, 7.017_m, {36.0_deg}}
  };

  const frc::Pose2d redLoading[2] = {
    {11.0_m, 4.0_m, {-54_deg}}, 
    {12.0_m, 5.75_m, {54_deg}}
  };

  const frc::Pose2d processorLoading {11.0_m, 4.0_m, {-180_deg}};

  // AUTON ROUTINES
  /*field.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);*/
  const frc::Translation2d fieldMiddle{field.GetFieldLength() / 2.0, field.GetFieldWidth() / 2.0};

  frc::Pose2d HandleAlliancePose(frc::Pose2d pose);

  frc2::CommandPtr EmptyAuto{GetEmptyCommand()};

  frc::Pose2d onePointMiddleStart{7.1_m, 4.0_m, -90.0_deg};
  frc::Pose2d onePointMiddleEnd{6.386_m, 4.0_m, -90.0_deg};

  // One Piece Middle Auton
  frc2::CommandPtr OnePieceMiddle{frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this]() { // Reset to starting pose
      drive.ResetOdometry(HandleAlliancePose(onePointMiddleStart));
    }, {}),
    frc2::cmd::Parallel(
      drive.PathGenCommand(HandleAlliancePose(autonReef[0])), // Drive to reef
      frc2::cmd::Sequence(  // Wait 2 seconds then go to L4Coral pose
        frc2::cmd::Wait(0.5_s),
        SetAllKinematics(l4Coral),
        frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      )),
      frc2::cmd::RunOnce([this]() {
        SetMLTarget(MLLabels::Reef);
        ReefHeightLevel = 4;
        drive.SetTransAdjust(true);
      }),
      frc2::cmd::Race(
        frc2::cmd::WaitUntil([this]() {
          double dist = drive.GetWallDistance();
          double off =  dist - 0.22;
          if(off < 0.0) off = 0.0;
          if(off > 1.0) off = 1.0;
          drive.SetTransYAdjustSpeeds(-5.0_mps * off);
          drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);

          return mlDCenter < 40 && !noReefFound && dist < 0.33;
        }),
        GetMLFollowCommand()
      ),
      frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this]() {
          coral.SetPower(0.7);
        }, {&coral}),
        frc2::cmd::Wait(0.7_s)
      ),
      frc2::cmd::RunOnce([this]() {
        coral.SetPower(0.0);
        SetMLTarget(-1);
      }, {&coral}),
      frc2::cmd::Parallel(
        drive.PathGenCommand(HandleAlliancePose(onePointMiddleEnd)), // Drive to reef
        frc2::cmd::Sequence(  // Wait 2 seconds then go to L4Coral pose
          frc2::cmd::Wait(0.75_s),
          SetAllKinematics(startingPose),
          frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      ))
  )};

  // TWO PIECE LEFT //
  frc::Pose2d twoPieceLeftStart{7.1_m, 2.521_m, -90.0_deg};

  frc2::CommandPtr TwoPieceLeft{frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this]() { // Reset to starting pose
      drive.ResetOdometry(HandleAlliancePose(twoPieceLeftStart));
    }, {}),
    frc2::cmd::Parallel(
      drive.PathGenCommand(HandleAlliancePose(autonReef[1])), // Drive to reef
      frc2::cmd::Sequence(  // Wait 2 seconds then go to L4Coral pose
        frc2::cmd::Wait(0.5_s),
        SetAllKinematics(l4Coral),
        frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      )),
      frc2::cmd::RunOnce([this]() {
        SetMLTarget(MLLabels::Reef);
        ReefHeightLevel = 4;
        drive.SetTransAdjust(true);
      }),
      frc2::cmd::Race(
        frc2::cmd::WaitUntil([this]() {
          double dist = drive.GetWallDistance();
          double off =  dist - 0.3;
          if(off < 0.0) off = 0.0;
          if(off > 1.0) off = 1.0;
          drive.SetTransYAdjustSpeeds(-5.0_mps * off);
          drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);

          return mlDCenter < 40 && !noReefFound && dist < 0.33;
        }),
        GetMLFollowCommand()
      ),
      frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this]() {
          coral.SetPower(0.7);
        }, {&coral}),
        frc2::cmd::Wait(0.7_s)
      ),
      frc2::cmd::RunOnce([this]() {
        coral.SetPower(0.0);
        SetMLTarget(-1);
        drive.SetTransAdjust(false);
      }, {&coral}),
      frc2::cmd::Parallel(
        drive.PathGenCommand(HandleAlliancePose(autonLoading[0])), // Drive to reef
        frc2::cmd::Sequence(  // Wait 2 seconds then go to L4Coral pose
          frc2::cmd::Wait(0.20_s),
          SetAllKinematics(loadPose),
          frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      )),
      frc2::cmd::RunOnce([this]() {
        drive.SetOmegaOverride(true);
        drive.SetThetaToHold(HandleAlliancePose(autonLoading[0]).Rotation());
        SetMLTarget(MLLabels::Coral);
        drive.SetTransAdjust(true);
        coral.SetPower(-1.0);
      }),
      frc2::cmd::Race(
        frc2::cmd::WaitUntil([this]() {
          drive.SetTransYAdjustSpeeds(0.0_mps);
          drive.Drive({0.0_mps, 0.9_mps, 0.0_deg_per_s}, true, false);

          return coral.IsCoralIndexed();
        }),
        GetMLFollowCommand()
      ),
      frc2::cmd::RunOnce([this]() {
        coral.SetPower(0.0);
        SetMLTarget(-1);
        drive.SetTransAdjust(false);
      }, {&coral}),
      frc2::cmd::Parallel(
      drive.PathGenCommand(HandleAlliancePose(autonReef[2])), // Drive to reef
      frc2::cmd::Sequence(  // Wait 2 seconds then go to L4Coral pose
        frc2::cmd::Wait(1.5_s),
        SetAllKinematics(l4Coral),
        frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      )),
      frc2::cmd::RunOnce([this]() {
        SetMLTarget(MLLabels::Reef);
        ReefHeightLevel = 4;
        drive.SetTransAdjust(true);
      }),
      frc2::cmd::Race(
        frc2::cmd::WaitUntil([this]() {
          double dist = drive.GetWallDistance();
          double off =  dist - 0.3;
          if(off < 0.0) off = 0.0;
          if(off > 1.0) off = 1.0;
          drive.SetTransYAdjustSpeeds(-5.0_mps * off);
          drive.Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s}, true, false);

          return mlDCenter < 40 && !noReefFound && dist < 0.33;
        }),
        GetMLFollowCommand()
      ),
      frc2::cmd::Parallel(
        frc2::cmd::RunOnce([this]() {
          coral.SetPower(0.7);
        }, {&coral}),
        frc2::cmd::Wait(0.7_s)
      ),
      frc2::cmd::RunOnce([this]() {
        coral.SetPower(0.0);
        SetMLTarget(-1);
        drive.SetTransAdjust(false);
      }, {&coral}),
      frc2::cmd::Parallel(
        drive.PathGenCommand(HandleAlliancePose(autonLoading[0])), // Drive to left load
        frc2::cmd::Sequence(  // Wait 2 seconds then go to load pose
          frc2::cmd::Wait(0.20_s),
          SetAllKinematics(loadPose),
          frc2::cmd::WaitUntil([this]() { return ManipulatorIsAtTarget(); })
      ))
  )};

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
  void HandlePartnerCommands(frc2::CommandPtr solo, frc2::CommandPtr partner);

  /**
   * Return a pointer to an empty Command that will do nothing when run.
   *
   * @return A Command* to an empty Command
   */
  frc2::CommandPtr GetEmptyCommand();

  frc2::CommandPtr HandleModeScheduling(frc2::CommandPtr coral, frc2::CommandPtr algae);

  /**
   * Return the command pointer that sets all subsystem kinematics.
   */
  frc2::CommandPtr SetAllKinematics(KinematicsPose pose);

  /**
   * Return if the manipulator is at the target.
   */
  bool ManipulatorIsAtTarget();

  void ManuallySchedule(frc2::CommandPtr&& cmd);
  void SetMLTarget(int label);

  int camFrameHeight = 480;
  int camFrameWidth = 640;
  int camFrameCenterOffset = 50;

  int mlTrackingTarget = MLLabels::Coral;
  bool noCoralFound = true;
  bool noReefFound = true;
  double mlDCenter = 99999.0;
  double mlAutoScoreThreshold = 20.0;
  int mlReefCamId = 2;
  int aprilTagCamId = 0;
  // Persistance variables
  bool persistenceDataSet = false;
  int persistenceRetries = 10;
  int currentRetries = 0;
  double mlLastX = 0.0;
  double mlLastY = 0.0;
  double mlLastWidth = 0.0;
  double mlLastHeight = 0.0;
  double mlLastHeightRatio = 0.0;
  uint32_t mlLastCaptureTime = 0;
  units::second_t mlRioLastCaptureTime = 0_s;
  // Persistance variables
  // Reef filter parameters
  double reefL4HeightRatioThreshold = 1.7;
  double reefHeightRatioThreshold = 0.8;
  double reefYPosMax = 240;
  double reefL4AreaMin = 10000.0;
  double reefAreaMin = 2000.0;
  // Reef filter parameters
  //
  // Coral filter parameters
  double coralAreaMin = 1000.0;
  double coralAreaMax = 200000.0;
  // Coral filter parameters

  // Reef persistence parameters
  double reefMaxWidthDrift = 20.0;
  double reefMaxHeightDrift = 10.0;
  double reefMaxXDrift = 10.0;
  double reefMaxYDrift = 10.0;
  double reefTimeMultiplier = 0.0;
  // The X/Y comments are not typos
  double reefXSpeedMultiplier = 0.0; // Matched to robot Y speed
  double reefYSpeedMultiplier = 0.0; // Matched to robot X speed
  // Reef persistence parameters

  // Coral persistence parameters
  double coralMaxWidthDrift = 80.0;
  double coralMaxHeightDrift = 80.0;
  double coralMaxXDrift = 160.0;
  double coralMaxYDrift = 60.0;
  double coralTimeMultiplier = 0.0;
  // The X/Y comments are not typos
  double coralXSpeedMultiplier = 0.0; // Matched to robot Y speed
  double coralYSpeedMultiplier = 0.0; // Matched to robot X speed
  // Reef persistence parameters


  bool IsReefDisqualified(JetsonSubsystem::MLDetectionFrame &reef);
  bool IsViableReefPersistenceTarget(JetsonSubsystem::MLDetectionFrame &reef);
  JetsonSubsystem::MLDetectionFrame GetReefTrackingTarget(std::vector<JetsonSubsystem::MLDetectionFrame> &dets);

  bool IsCoralDisqualified(JetsonSubsystem::MLDetectionFrame &coral);
  bool IsViableCoralPersistenceTarget(JetsonSubsystem::MLDetectionFrame &coral);
  JetsonSubsystem::MLDetectionFrame GetCoralTrackingTarget(std::vector<JetsonSubsystem::MLDetectionFrame> &dets);

  frc2::CommandPtr GetMLFollowCommand();

  // The chooser for the autonomous routines
  /*frc::SendableChooser<frc2::CommandPtr&&> autonChooser;*/
  frc::SendableChooser<frc2::Command*> autonChooser;
};
