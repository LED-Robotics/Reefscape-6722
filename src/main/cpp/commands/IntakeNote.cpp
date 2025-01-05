#include "commands/IntakeNote.h"

IntakeNote::IntakeNote(IntakeSubsystem* intakeRef, ShooterSubsystem *shooterRef) 
: intake(intakeRef),
shooter(shooterRef) {
  AddRequirements(intakeRef);
  SetName("Intake_Note");
}

void IntakeNote::Initialize() {
  intake->SetPower(-0.5);
}

void IntakeNote::End(bool interrupted) {
  intake->SetPower(0.0);
}

bool IntakeNote::IsFinished() {
  return shooter->IsNoteIndexed();
}