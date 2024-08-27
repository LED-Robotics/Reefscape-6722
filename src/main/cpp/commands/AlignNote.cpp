#include "commands/AlignNote.h"

AlignNote::AlignNote(DriveSubsystem *driveRef, JetsonSubsystem *jetsonRef) 
: drive(driveRef),
jetson(jetsonRef) {
  AddRequirements({driveRef});
  SetName("Align_Note");
}

void AlignNote::Initialize() {
  drive->SetOmegaOverride(false);
  thetaController.SetSetpoint(0.0);
  lastTs = jetson->GetTargetSize();
  tx = jetson->GetXOffset();
}

void AlignNote::Execute() {
  tx = jetson->GetXOffset();
  double thetaSpeed = thetaController.Calculate(tx);
  drive->Drive({0.0_mps, 0.0_mps, thetaSpeed * 1.0_deg_per_s});
}

void AlignNote::End(bool interrupted) {
  drive->Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s});
}

bool AlignNote::IsFinished() {
  lastTs = tx < jetson->GetTargetSize();
  return !jetson->IsTarget() || fabs(tx) < kThreshold;
}