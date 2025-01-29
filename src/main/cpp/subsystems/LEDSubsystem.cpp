// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem/LEDSubsystem.h"

#include <iostream>

using namespace LEDConstants;
using namespace frc;

LEDSubsystem::LEDSubsystem() : 
blinkin1{kBlinkin1Port}, blinkin2{kBlinkin2Port} {
  // Implementation of subsystem constructor goes here
}

void LEDSubsystem::Periodic() {
  blinkin1.Set(power);
  blinkin2.Set(power);
}

void LEDSubsystem::SetPower(double newPower) {
  power = newPower;
}
