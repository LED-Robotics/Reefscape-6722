#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

class SmartMotor {
  public:
    SmartMotor() {};
    virtual units::angle::turn_t GetPosition() = 0;
    virtual units::angular_velocity::turns_per_second_t GetVelocity() = 0;
    virtual double GetPower() = 0;
    virtual void SetPosition(units::angle::turn_t turns) = 0;
    virtual void SetVelocity(units::angular_velocity::turns_per_second_t tps) = 0;
    virtual void SetPower(double power) = 0;
    virtual void Stop() = 0;
};
