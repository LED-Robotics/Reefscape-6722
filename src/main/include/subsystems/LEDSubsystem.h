// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc/motorcontrol/Spark.h"
#include <frc/DriverStation.h>

#include "Constants.h"

using namespace frc;

class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetPower(double newPower);
    
 private:
  Spark blinkin1;
  Spark blinkin2;

  double power = LEDConstants::kIdlePreset;
};
