#pragma once
#include "Constants.h"
#include <iostream>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/JetsonSubsystem.h"

/**
 * Command that moves the robot to a set Pose2d based on its odometry.
 */
class AlignNote : public frc2::CommandHelper<frc2::Command, AlignNote> {
 public:
  explicit AlignNote(DriveSubsystem* driveRef, JetsonSubsystem *jetsonRef);

  void Initialize() override;

  void Execute() override;
  
  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    JetsonSubsystem *jetson;
    bool targetFound = false;
    frc::PIDController thetaController{0.25, 0.0, 0.0};

    double tx;
    double lastTs;

    const double kThreshold = 40.0;
    const double kDistanceScale = 0.03;
};