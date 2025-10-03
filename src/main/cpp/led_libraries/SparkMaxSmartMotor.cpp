#include "led_libraries/SparkMaxSmartMotor.h"
#include "units/angular_velocity.h"

SparkMaxSmartMotor::SparkMaxSmartMotor(int port, SparkLowLevel::MotorType type) 
  : motor{port, type} {
}

units::angle::turn_t SparkMaxSmartMotor::GetPosition() {
  return units::angle::turn_t{motor.GetEncoder().GetPosition()};
}

units::angular_velocity::turns_per_second_t SparkMaxSmartMotor::GetVelocity() {
  return units::angular_velocity::turns_per_second_t{motor.GetEncoder().GetVelocity()};
}

double SparkMaxSmartMotor::GetPower() {
  return motor.Get();
}

void SparkMaxSmartMotor::SetPosition(units::angle::turn_t turns) {
  motor.GetClosedLoopController().SetReference(turns.value(), SparkBase::ControlType::kPosition);
}

void SparkMaxSmartMotor::SetVelocity(units::angular_velocity::turns_per_second_t tps) {
  motor.GetClosedLoopController().SetReference(tps.value(), SparkBase::ControlType::kVelocity);
}

void SparkMaxSmartMotor::SetPower(double power) {
  motor.Set(power);
}

void SparkMaxSmartMotor::Stop() {
  motor.StopMotor();
}

