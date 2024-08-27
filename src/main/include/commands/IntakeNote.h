#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
 */
class IntakeNote : public frc2::CommandHelper<frc2::Command, IntakeNote> {
 public:
  explicit IntakeNote(IntakeSubsystem* intakeRef, ShooterSubsystem *shooterRef);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    IntakeSubsystem *intake;
    ShooterSubsystem *shooter;
    int *target;
};