#include "commands/IndexerSet.h"

IndexerSet::IndexerSet(double position, ShooterSubsystem* shooterRef) 
: indexer(shooterRef) {
  AddRequirements({shooterRef});
  target = position;
}

void IndexerSet::Initialize() {
  origAutoIndexState = indexer->GetIndexerAuto();
  indexer->SetIndexerAuto(false);
  indexer->SetIndexerPosition(target);
}

void IndexerSet::End(bool interrupted) {
  indexer->SetIndexerAuto(origAutoIndexState);
}

bool IndexerSet::IsFinished() {
  // end when all three subsystems are at their target setpoint 
  return indexer->IndexerIsAtTarget();
}