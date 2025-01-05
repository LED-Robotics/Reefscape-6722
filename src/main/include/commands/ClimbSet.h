#pragma once

#include "Constants.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>


/**
 * Command that sets the position of the elevator, intake, and arm subsystems. 
 * This Command controls the order in which the subsystems move based on the target position,
 * as opposed to the original method of moving all three simultaneously all the time.
 */
class ClimbSet : public frc2::CommandHelper<frc2::Command, ClimbSet> {
 public:
  explicit ClimbSet(double position, ClimbSubsystem* climbRef);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    double target = 0.0;
    ClimbSubsystem *climb;
};