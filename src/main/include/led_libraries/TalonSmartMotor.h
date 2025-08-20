#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include "SmartMotor.h"

using namespace ctre::phoenix6;

class TalonSmartMotor : public SmartMotor {
  public:
    hardware::TalonFX *motor;
    controls::PositionVoltage positionController{0_tr};
    controls::VelocityVoltage velocityController{0_tps};
    bool focEnabled;
    
    TalonSmartMotor(hardware::TalonFX *motorRef, bool foc = true);

    units::angle::turn_t GetPosition() override;
    units::angular_velocity::turns_per_second_t GetVelocity() override;
    double GetPower() override;
    void SetPosition(units::angle::turn_t turns) override;
    void SetVelocity(units::angular_velocity::turns_per_second_t tps) override;
    void SetPower(double power) override;
    void Stop() override;
};
