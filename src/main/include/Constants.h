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
    constexpr int kArbitrary = 0;
}

namespace DriveConstants {
    //Wheel motors
    constexpr int kBackLeftPort = 0;
    constexpr int kFrontLeftPort = 1;
    constexpr int kBackRightPort = 2;
    constexpr int kFrontRightPort = 3;
    //Degree of wheel motors
    constexpr int kBackLeftThetaPort = 4;
    constexpr int kFrontLeftThetaPort = 5;
    constexpr int kBackRightThetaPort = 6;
    constexpr int kFrontRightThetaPort = 7;

    constexpr int kBackLeftEncoderPort = 4;
    constexpr int kFrontLeftEncoderPort = 5;
    constexpr int kBackRightEncoderPort = 6;
    constexpr int kFrontRightEncoderPort = 7;
    
    constexpr units::angle::turn_t kBLeftMagPos = 0.31396484375_tr;
    constexpr units::angle::turn_t kFLeftMagPos = -0.46044921875_tr;
    constexpr units::angle::turn_t kBRightMagPos = 0.47021484375_tr;
    constexpr units::angle::turn_t kFRightMagPos = -0.37939453125_tr;
    
    // info for odometry
    constexpr int kEncoderResolution = 4096;
    constexpr auto kDriveBaseRadius = 0.319786_m;
    constexpr double kWheelRadius = .0508;
    constexpr double kDriveRatio = 1 / 4.125;
    constexpr double kTurnRatio = 1 / 10.29;
    constexpr double kDriveDistancePerRev = (2 * std::numbers::pi * kWheelRadius) * kDriveRatio;
    // constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / (double)kEncoderResolution) * kDriveRatio;
    // constexpr double kTurnEncoderDegreesPerPulse = (360.0 / (double)kEncoderResolution) * kTurnRatio;
    constexpr double kTurnEncoderDegreesPerPulse = 360.0;

    // states
    constexpr int kArbitrary = 0;

    // TalonFX config
    constexpr double kDriveP = 0.2402345985174179;
    constexpr double kDriveV = 0.1345313787460327;
    constexpr units::time::second_t kDriveRamp = 0.0_s;
    constexpr units::current::ampere_t kDriveCurrentLimit = 30.0_A;
    constexpr double kTurnP = 80.0;
    constexpr double kTurnPRatio = 10.279000282287598;

    constexpr auto turnKp = 0.275;

    // drivetrain controller constants
    constexpr double kDriveDeadzone = 0.1;
    constexpr double kTurnDeadzone = 0.1;
    constexpr double kDriveCurveExtent = 0.6;
    constexpr auto kDriveAccelerationLimit = 9.0_mps_sq;
    constexpr auto kDriveDecelerationLimit = 25.0_mps_sq;
    constexpr auto kDriveTranslationLimit = 7.7_mps;
    constexpr double kTxAdjust = 0.5;
    constexpr double kAlignP = 0.5;
    constexpr double kPVelTurnOffset = -2.0;
    constexpr double kPVelDistOffset = 0.0;
    constexpr double kDistMultiplier = 1.5;
    constexpr int kDistSamples = 5;
    constexpr double kPNote = 0.3;
    constexpr double kPYTrans = 0.1;
    constexpr double kThetaDeadzone = 2.0;

    constexpr units::kilogram_t kRobotWeight = 54.43_kg;
    constexpr auto kMOI = 60;

}  // namespace DriveConstants

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
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

namespace LEDConstants {
    constexpr int kBlinkin1Port = 0;
    constexpr int kBlinkin2Port = 1;
    constexpr double kIdlePreset = 0.41;
}