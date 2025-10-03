#include "led_libraries/SparkFlexSmartMotor.h"
#include "units/angular_velocity.h"

SparkFlexSmartMotor::SparkFlexSmartMotor(int port, SparkLowLevel::MotorType type) 
  : motor{port, type} {
}

units::angle::turn_t SparkFlexSmartMotor::GetPosition() {
  return units::angle::turn_t{motor.GetEncoder().GetPosition()};
}

units::angular_velocity::turns_per_second_t SparkFlexSmartMotor::GetVelocity() {
  return units::angular_velocity::turns_per_second_t{motor.GetEncoder().GetVelocity()};
}

double SparkFlexSmartMotor::GetPower() {
  return motor.Get();
}

void SparkFlexSmartMotor::SetPosition(units::angle::turn_t turns) {
  motor.GetClosedLoopController().SetReference(turns.value(), SparkBase::ControlType::kPosition);
}

void SparkFlexSmartMotor::SetVelocity(units::angular_velocity::turns_per_second_t tps) {
  motor.GetClosedLoopController().SetReference(tps.value(), SparkBase::ControlType::kVelocity);
}

void SparkFlexSmartMotor::SetPower(double power) {
  motor.Set(power);
}

void SparkFlexSmartMotor::Stop() {
  motor.StopMotor();
}

