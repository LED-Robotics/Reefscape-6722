#include "commands/TurnToNote.h"

TurnToNote::TurnToNote(units::angular_velocity::degrees_per_second_t speed, DriveSubsystem *driveRef, JetsonSubsystem *jetsonRef) 
: drive(driveRef),
jetson(jetsonRef) {
  AddRequirements({driveRef});
  SetName("TurnToNote");
  turnSpeed = speed;
}

void TurnToNote::Initialize() {
  drive->SetOmegaOverride(false);
}

void TurnToNote::Execute() {
  drive->Drive({0.0_mps, 0.0_mps, turnSpeed});
}

void TurnToNote::End(bool interrupted) {
  drive->Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s});
}

bool TurnToNote::IsFinished() {
  return jetson->IsTarget() && jetson->GetTargetSize() > 0.03;
}