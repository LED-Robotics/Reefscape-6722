// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/DriverStation.h>

#include "Constants.h"

using namespace frc;

class LimelightSubsystem : public frc2::SubsystemBase {
 public:
  LimelightSubsystem(std::string_view targetTable);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Returns the current vision processing pipeline of the limelight.
   *
   * @return The subsystem's current vision processing pipeline
   */
  double GetPipeline();

  /**
   * Sets the vision processing pipeline of the limelight.
   *
   * @param pipeline The new pipeline to set the limelight to
   */
  void SetPipeline(double pipeline);

  /**
   * Sets the LED state of the limelight.
   *
   * @param state The new LED state
   */
  void SetLED(int state);

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
  double GetTargetArea();

  /**
   * Returns the skew of the target from the limelight (-90 to 0 degrees).
   *
   * @return The target's skew
   */
  double GetTargetSkew();

  /**
   * Returns the robot's position as tracked using AprilTags.
   *
   * @return The bot's position array
   */
  std::vector<double> GetBotPos();

  /**
   * Returns the target's position relative to the center of the robot.
   *
   * @return The target's position array
   */
  std::vector<double> GetTargetPos();
    
 private:
  double targetXOffset;
  double targetYOffset;
  double targetArea;
  double targetSkew;
  bool targetFound;
  std::vector<double> botPose{6};
  std::vector<double> targetPose{6};
  std::string_view tableName;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::shared_ptr<nt::NetworkTable> table;
};
