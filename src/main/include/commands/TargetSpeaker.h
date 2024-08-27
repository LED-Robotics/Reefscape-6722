#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
 */
class TargetSpeaker : public frc2::CommandHelper<frc2::Command, TargetSpeaker> {
 public:
  explicit TargetSpeaker(int *targetRef, DriveSubsystem *driveRef, ArmSubsystem* armRef, ShooterSubsystem* shooterRef, LimelightSubsystem *limelightRef);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    ArmSubsystem *arm;
    ShooterSubsystem *shooter;
    LimelightSubsystem *limelight;
    int *target;
};