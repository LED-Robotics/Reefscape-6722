#include "led_libraries/PositionalSubsystem.h"

using namespace frc;

PositionalSubsystem::PositionalSubsystem(std::vector<SmartMotor*> motorRefs)
  : motors{motorRefs} {
}

void PositionalSubsystem::RunMotors() {
  if(state == States::kOff) {
    for(auto &motor : motors) {
      motor->Stop();
    }
  } else if(state == States::kPowerMode) {
    for(auto &motor : motors) {
      motor->SetPower(power);
    }
  } else if(state == States::kPositionMode) {
    for(auto &motor : motors) {
      motor->SetPosition(position);
    }
  }
}

void PositionalSubsystem::Off() {
  state = States::kOff;
}

void PositionalSubsystem::On() {
  state = States::kPowerMode;
}

void PositionalSubsystem::SetPower(double newPower) {
  power = newPower;
}

void PositionalSubsystem::SetState(int newState) {
  state = newState;
}

int PositionalSubsystem::GetState() {
  return state;
}

units::angle::turn_t PositionalSubsystem::GetPosition() {
  units::angle::turn_t sum = 0_tr;
  for(auto &motor : motors) {
    sum += motor->GetPosition();
  }
  return sum / (double)motors.size();
}

void PositionalSubsystem::SetTargetPosition(units::angle::turn_t newPosition, double newFeedForward) {
  position = newPosition;
  feedForward = newFeedForward;
}

void PositionalSubsystem::SetNudge(units::angle::turn_t newNudge) {
  nudge = newNudge;
}
