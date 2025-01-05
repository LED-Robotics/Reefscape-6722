#include "commands/TrapSet.h"

TrapSet::TrapSet(double position, TrapSubsystem* trapRef) 
: trap(trapRef) {
  AddRequirements({trapRef});
  target = position;
}

void TrapSet::Initialize() {
  trap->SetTargetAngle(target);
}

void TrapSet::End(bool interrupted) {
  SmartDashboard::PutNumber("Claw Position", target);
}

bool TrapSet::IsFinished() {
  // end when all three subsystems are at their target setpoint 
  return trap->IsAtTarget();
}