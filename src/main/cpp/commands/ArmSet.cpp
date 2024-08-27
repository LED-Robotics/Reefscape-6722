#include "commands/ArmSet.h"

ArmSet::ArmSet(double position, ArmSubsystem* armRef) 
: arm(armRef) {
  AddRequirements({armRef});
  target = position;
}

void ArmSet::Initialize() {
  arm->SetTargetAngle(target);
}

void ArmSet::End(bool interrupted) {
  SmartDashboard::PutNumber("Arm Position", target);
}

bool ArmSet::IsFinished() {
  // end when all three subsystems are at their target setpoint 
  return arm->IsAtTarget();
}