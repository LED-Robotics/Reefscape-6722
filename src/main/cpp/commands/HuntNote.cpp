#include "commands/HuntNote.h"

HuntNote::HuntNote(DriveSubsystem *driveRef, IntakeSubsystem* intakeRef, JetsonSubsystem *jetsonRef) 
: drive(driveRef),
intake(intakeRef),
jetson(jetsonRef) {
  AddRequirements({driveRef, intakeRef});
  SetName("Hunt_Note");
}

void HuntNote::Initialize() {
  drive->SetOmegaOverride(false);
  intake->SetState(IntakeConstants::kAutoMode);
  xController.SetSetpoint(-190.0);
  yController.SetSetpoint(0.0);
  thetaController.SetSetpoint(0.0);
  lastTs = jetson->GetTargetSize();
  // xController.SetP(SmartDashboard::GetNumber("noteXP", xController.GetP()));
  // yController.SetP(SmartDashboard::GetNumber("noteYP", yController.GetP()));
  // thetaController.SetP(SmartDashboard::GetNumber("noteThetaP", thetaController.GetP()));
}

void HuntNote::Execute() {

  tx = jetson->GetXOffset();
  ty = jetson->GetYOffset();
  double xSpeed = xController.Calculate(ty);
  if(xSpeed > kMaxX) xSpeed = kMaxX;
  double ySpeed = -yController.Calculate(tx);
  double ts = jetson->GetTargetSize();
  
  xSpeed *= (kDistanceScale / ts);
  ySpeed *= (kDistanceScale / ts);
  double thetaSpeed = thetaController.Calculate(tx);
  drive->Drive({xSpeed * 1.0_mps, ySpeed * 1.0_mps, thetaSpeed * 1.0_deg_per_s});
}

void HuntNote::End(bool interrupted) {
  drive->Drive({-2.4_mps, 0.0_mps, 0.0_deg_per_s});
  frc::Wait(0.25_s);
  drive->Drive({0.0_mps, 0.0_mps, 0.0_deg_per_s});
}

bool HuntNote::IsFinished() {
  if(!jetson->IsTarget()) return true;
  double currentTs = jetson->GetTargetSize();
  bool tsJumped =  lastTs / currentTs > 2.5;
  lastTs = currentTs;
  return tsJumped;
}