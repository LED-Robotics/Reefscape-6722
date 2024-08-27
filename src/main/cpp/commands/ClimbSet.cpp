#include "commands/ClimbSet.h"

ClimbSet::ClimbSet(double position, ClimbSubsystem* climbRef) 
: climb(climbRef) {
  AddRequirements({climbRef});
  target = position;
}

void ClimbSet::Initialize() {
  climb->SetTargetPosition(target);
}

void ClimbSet::End(bool interrupted) {
  SmartDashboard::PutNumber("Climb Position", target);
}

bool ClimbSet::IsFinished() {
  // end when all three subsystems are at their target setpoint 
  return climb->IsAtTarget();
}