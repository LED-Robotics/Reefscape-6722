// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
<<<<<<< Updated upstream
#include <rev/CANSparkMax.h>
=======
#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>
>>>>>>> Stashed changes
#include "ColorSensor.h"
#include <frc/I2C.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

#include "Constants.h"

using namespace frc;
using namespace ctre::phoenix6;
<<<<<<< Updated upstream
using namespace rev;
=======
using namespace rev::spark;
>>>>>>> Stashed changes

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem(std::function<units::length::meter_t()> getDistFunc, int *targetRef, Orchestra *orcRef);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  /**
   * Turns the Shooter state to kOff.
   */
  void Off();
  
  /**
   * Turns the Shooter state to kPowerMode.
   */
  void On();

  /**
   * Sets the power for the Shooter to use when in kPowerMode.
   *
   * @param power the power for the Shooter to use
   */
  void SetPower(double newPower);

  /**
   * Returns the current state of the Shooter.
   *
   * @return The current state of the Shooter
   */
  int GetState();

  /**
   * Sets the current state of the Shooter.
   */
  void SetState(int newState);

  /**
   * Returns the current velocity of the Shooter's left TalonFX.
   */
  double GetLeftVelocity();

  /**
   * Returns the current velocity of the Shooter's right TalonFX.
   */
  double GetRightVelocity();

  /**
   * Returns the current velocity in turns per second of the Shooter Subsystem.
   */
  double GetVel();

  /**
   * Sets the target velocity of the Shooter.
   */
  void SetTargetVelocity(double newVel);

  /**
   * Returns whether the shooter is at its intended target velocity.
   */
  bool IsAtTarget();

  void RpmFromDistance(units::length::meter_t distance);

  /**
   * Offsets the current position of the indexer.
   */
  void OffsetIndexer(double offset);
  
  /**
   * Sets the target position of the indexer.
   */
  void SetIndexerPosition(double newPos);

  /**
   * Returns whether the indexer is at its intended target velocity.
   */
  bool IndexerIsAtTarget();

  bool GetIndexerAuto();
  
  void SetIndexerAuto(bool state);

  bool IsNoteIndexed();

  bool IsNoteIndexedNew();

  /**
   * Sets the state of the Shooter brakes.
   *
   * @param state the state of the brakes.
   */
  void SetBrakeMode(bool state);

  /**
   * Initially configure onboard TalonFX settings for motors.
   */
  void ConfigMotors();
    
 private:
//  while the state is kOn the Shooter will run at the current power setting
  int state = ShooterConstants::kRpmMode;
  int *target;
  double power = ShooterConstants::kDefaultPower;
  double velocity = 0.0;
  units::angular_velocity::turns_per_second_t autoVel = 0.0_tps;
  bool indexerAutoEnabled = true;

  double indexerPosition = ShooterConstants::kIndexerHolding;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
<<<<<<< Updated upstream
  // WPI_TalonSRX left;
  hardware::TalonFX left;
  // WPI_TalonSRX right;
  hardware::TalonFX right;

  // hardware::TalonFX indexer;
=======
  hardware::TalonFX left;
  hardware::TalonFX right;
  SparkMax indexer;
>>>>>>> Stashed changes

  std::function<units::length::meter_t()> getTargetDistance;

  int lastColorVal = 5000;
  int lastIndexedAt = 5000;
  bool isNoteIndexed = false;
<<<<<<< Updated upstream

  CANSparkMax indexer;
  SparkPIDController indexerController;
  SparkRelativeEncoder indexerEncoder;

  controls::VelocityVoltage ctreVelocity{0_tps};
  // controls::PositionVoltage ctrePosition{0_tr};
=======
  
  SparkMaxConfig indexerConfig{};
  // SparkClosedLoopController indexerController;
  // SparkRelativeEncoder indexerEncoder;

  controls::VelocityVoltage ctreVelocity{0_tps};
>>>>>>> Stashed changes

  ColorSensor color;
  DigitalInput shooterBeamBreakReceiver;
  Orchestra *orca;
};
