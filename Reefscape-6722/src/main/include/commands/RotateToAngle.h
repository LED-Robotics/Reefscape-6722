#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/geometry/Rotation2d.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/LimelightSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
 */
class RotateToAngle : public frc2::CommandHelper<frc2::Command, RotateToAngle> {
 public:
  explicit RotateToAngle(int *targetRef, frc::Rotation2d angle, DriveSubsystem *driveRef);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    frc::Rotation2d targ;
    int *target;
};