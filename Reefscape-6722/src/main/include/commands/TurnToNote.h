#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/JetsonSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
 */
class TurnToNote : public frc2::CommandHelper<frc2::Command, TurnToNote> {
 public:
  explicit TurnToNote(units::angular_velocity::degrees_per_second_t speed, DriveSubsystem* driveRef, JetsonSubsystem *jetsonRef);

  void Initialize() override;

  void Execute() override;
  
  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    JetsonSubsystem *jetson;
    units::angular_velocity::degrees_per_second_t turnSpeed;
};