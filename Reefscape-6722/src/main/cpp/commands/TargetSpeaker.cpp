#include "commands/TargetSpeaker.h"

TargetSpeaker::TargetSpeaker(int *targetRef, DriveSubsystem *driveRef, ArmSubsystem* armRef, ShooterSubsystem* shooterRef, LimelightSubsystem *limelightRef) 
: drive(driveRef),
arm(armRef),
shooter(shooterRef),
limelight(limelightRef),
target{targetRef} {
  AddRequirements({armRef, shooterRef});
  SetName("Target_Speaker");
}

void TargetSpeaker::Initialize() {
  *target = GlobalConstants::kSpeaker;
  shooter->SetState(ShooterConstants::kRpmMode);
  drive->SetOmegaOverride(true);
}

void TargetSpeaker::Execute() {
  drive->Drive({0.0_mps, 0.0_mps, 0_deg_per_s});
}

bool TargetSpeaker::IsFinished() {
  return drive->IsAtTarget() && arm->IsAtTarget() && shooter->IsAtTarget() && limelight->IsTarget();
}