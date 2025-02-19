// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <frc/geometry/Pose2d.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <ctre/phoenix6/Orchestra.hpp>


#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace GlobalConstants {
    // Subsystem Target States
    enum GlobalModes {
      kArbitrary,
      kCoralMode,
      kAlgaeMode
    };
}

namespace AutoConstants {
    // for PID/pathfinding
    constexpr auto kMaxSpeed = 4.5_mps;
    constexpr auto kMaxAcceleration = 2_mps_sq;
    constexpr auto kAngularSpeed = 180_deg_per_s;
    constexpr auto kMaxAngularAcceleration = 180_deg_per_s_sq;

    constexpr double kPXController = 0.4;
    constexpr double kPYController = 0.4;
    constexpr double kPThetaController = 0.0;

    constexpr frc::Pose2d kDefaultStartingPose{{1.39_m, 4.11_m}, {180_deg}};
} //namespace AutoConstants

namespace OIConstants {
  enum ControllerPorts {
    kDriverControllerPort,
    kCoDriverControllerPort
  };
}  // namespace OIConstants

