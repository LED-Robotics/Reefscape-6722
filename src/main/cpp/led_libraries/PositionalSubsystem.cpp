#include "led_libraries/PositionalSubsystem.h"

using namespace frc;

PositionalSubsystem::PositionalSubsystem(std::vector<SmartMotor*> motorRefs)
  : motors{motorRefs} {
}

void PositionalSubsystem::Periodic() {
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

units::angle::turn_t PositionalSubsystem::GetLeftPosition() {
  // auto base = units::angle::turn_t{left.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  // return base + kStartPosition;
  return {};
}

units::angle::turn_t PositionalSubsystem::GetRightPosition() {
  // auto base = units::angle::turn_t{right.GetPosition().GetValueAsDouble() / kTurnsPerMeter};
  // return base + kStartPosition;
  return {};
}

units::angle::turn_t PositionalSubsystem::GetPosition() {
  auto left = GetLeftPosition();
  auto right = GetRightPosition();
  return (left + right) / 2;
}

void PositionalSubsystem::SetTargetPosition(units::angle::turn_t newPosition) {
  position = newPosition;
}

bool PositionalSubsystem::IsAtTarget() {
  // auto target = position + microAdjust;
  // auto leftPos = GetLeftPosition();
  // auto rightPos = GetRightPosition();
  //
  // bool leftAtTarget = leftPos > target - (kPositionDeadzone / 2) && leftPos < target + (kPositionDeadzone / 2);
  // bool rightAtTarget = rightPos > target - (kPositionDeadzone / 2) && rightPos < target + (kPositionDeadzone / 2);
  // return leftAtTarget && rightAtTarget;
  return {};
}

void PositionalSubsystem::SetBrakeMode(bool state) {
  // signals::NeutralModeValue mode;
  // if(state) mode = signals::NeutralModeValue::Brake;
  // else mode = signals::NeutralModeValue::Coast;
  // configs::MotorOutputConfigs updated;
  // updated.WithNeutralMode(mode);
  //
  // left.GetConfigurator().Apply(updated, 50_ms);
  // right.GetConfigurator().Apply(updated, 50_ms);
}

frc2::CommandPtr PositionalSubsystem::GetMoveCommand(units::angle::turn_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetPosition(target);
      }, {this}),
      frc2::cmd::WaitUntil([this](){
        return IsAtTarget();
      }));
}
