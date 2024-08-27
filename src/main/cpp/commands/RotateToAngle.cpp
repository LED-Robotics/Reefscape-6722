#include "commands/RotateToAngle.h"

RotateToAngle::RotateToAngle(int *targetRef, frc::Rotation2d angle, DriveSubsystem *driveRef) 
: drive(driveRef),
target{targetRef} {
  AddRequirements({driveRef});
  SetName("RotateToAngle");
  targ = angle;
}

void RotateToAngle::Initialize() {
  drive->SetThetaToHold(frc::DriverStation::GetAlliance() == frc::DriverStation::kRed ? targ.RotateBy(180_deg) : targ);
  *target = GlobalConstants::kArbitrary;
  drive->SetOmegaOverride(true);
}

void RotateToAngle::Execute() {
  drive->Drive({0.0_mps, 0.0_mps, 0_deg_per_s});
}

void RotateToAngle::End(bool interrupted) {
  drive->SetOmegaOverride(false);
}

bool RotateToAngle::IsFinished() {
  drive->IsAtTarget();
  return drive->IsAtTarget();
}