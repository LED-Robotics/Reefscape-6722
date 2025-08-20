#include "led_libraries/SparkSmartMotor.h"
#include "units/angular_velocity.h"

SparkSmartMotor::SparkSmartMotor(SparkBase *motorRef) 
  : motor{motorRef} {
}

units::angle::turn_t SparkSmartMotor::GetPosition() {
  return units::angle::turn_t{motor->GetEncoder().GetPosition()};
}

units::angular_velocity::turns_per_second_t SparkSmartMotor::GetVelocity() {
  return units::angular_velocity::turns_per_second_t{motor->GetEncoder().GetVelocity()};
}

double SparkSmartMotor::GetPower() {
  return motor->Get();
}

void SparkSmartMotor::SetPosition(units::angle::turn_t turns) {
  motor->GetClosedLoopController().SetReference(turns.value(), SparkBase::ControlType::kPosition);
}

void SparkSmartMotor::SetVelocity(units::angular_velocity::turns_per_second_t tps) {
  motor->GetClosedLoopController().SetReference(tps.value(), SparkBase::ControlType::kVelocity);
}

void SparkSmartMotor::SetPower(double power) {
  motor->Set(power);
}

void SparkSmartMotor::Stop() {
  motor->StopMotor();
}

