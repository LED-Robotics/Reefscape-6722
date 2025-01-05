// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

<<<<<<< Updated upstream
=======
#include <vector>
>>>>>>> Stashed changes
#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/DriverStation.h>

#include "Constants.h"

using namespace frc;

class JetsonSubsystem : public frc2::SubsystemBase {
 public:
  JetsonSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Used to check if the limelight has acquired a target
   *
   * @return Whether the limelight is tracking a target or not
   */
  bool IsTarget();

  /**
   * Returns the horizontal offset of the target from the limelight.
   *
   * @return The target's horizontal offset
   */
  double GetXOffset();

  /**
   * Returns the vertical offset of the target from the limelight.
   *
   * @return The target's vertical offset
   */
  double GetYOffset();

  /**
   * Returns the area of the target in percent of the camera image.
   *
   * @return The target's area
   */
  double GetTargetSize();

<<<<<<< Updated upstream
  void SetRecording(bool state);
    
=======
  std::vector<uint8_t> GetRawData();

  void SetRecording(bool state);



>>>>>>> Stashed changes
 private:
  double targetXOffset;
  double targetYOffset;
  double targetSize;
  bool targetFound;
  std::shared_ptr<nt::NetworkTable> table;
};
