#include "led_libraries/TalonSmartMotor.h"

TalonSmartMotor::TalonSmartMotor(hardware::TalonFX *motorRef, bool foc) 
  : motor{motorRef} {
  focEnabled = foc;
}

units::angle::turn_t TalonSmartMotor::GetPosition() {
  return motor->GetPosition().GetValue();
}

units::angular_velocity::turns_per_second_t TalonSmartMotor::GetVelocity() {
  return motor->GetVelocity().GetValue();
}

double TalonSmartMotor::GetPower() {
  return motor->Get();
}

void TalonSmartMotor::SetPosition(units::angle::turn_t turns) {
  motor->SetControl(positionController
      .WithPosition(turns)
      .WithEnableFOC(focEnabled));
}

void TalonSmartMotor::SetVelocity(units::angular_velocity::turns_per_second_t tps) {
  motor->SetControl(velocityController
      .WithVelocity(tps)
      .WithEnableFOC(focEnabled));
}

void TalonSmartMotor::SetPower(double power) {
  motor->Set(power);
}

void TalonSmartMotor::Stop() {
  motor->StopMotor();
}

