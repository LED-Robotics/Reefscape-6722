#include "rev/SparkMax.h"
#include "SmartMotor.h"

using namespace rev::spark;

class SparkMaxSmartMotor : public SmartMotor {
  public:
    SparkMax motor;
    
    SparkMaxSmartMotor(int port, SparkLowLevel::MotorType type);

    units::angle::turn_t GetPosition() override;
    units::angular_velocity::turns_per_second_t GetVelocity() override;
    double GetPower() override;
    void SetPosition(units::angle::turn_t turns) override;
    void SetVelocity(units::angular_velocity::turns_per_second_t tps) override;
    void SetPower(double power) override;
    void Stop() override;
};
