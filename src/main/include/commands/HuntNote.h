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
class HuntNote : public frc2::CommandHelper<frc2::Command, HuntNote> {
 public:
  explicit HuntNote(DriveSubsystem* driveRef, IntakeSubsystem* intakeRef, JetsonSubsystem *jetsonRef);

  void Initialize() override;

  void Execute() override;
  
  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    DriveSubsystem *drive;
    IntakeSubsystem *intake;
    JetsonSubsystem *jetson;
    double tx;
    double ty;
    double lastTs;
    bool targetFound = false;
    frc::PIDController xController{0.1, 0.0, 0.0};
    frc::PIDController yController{0.025, 0.0, 0.0};
    frc::PIDController thetaController{0.5, 0.0, 0.0};

    const double kXDeadzone = 10.0;
    const double kDistanceScale = 0.03;
    const double kDiffThreshold = 100.0;
    const double kMaxX = DriveConstants::kDriveTranslationLimit.value() * 0.7;


    //Holy shit why does tristan torture me?!?!?!!??? among us

};