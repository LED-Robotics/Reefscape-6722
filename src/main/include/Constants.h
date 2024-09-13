// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <ctre/phoenix6/Orchestra.hpp>
#include <units/angle.h>
#include <pathplanner/lib/path/PathConstraints.h>


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
    constexpr int kSpeaker = 1;
    constexpr int kAmp = 2;
    constexpr int kSource = 3;

    constexpr frc::Translation2d kSpeakerPose{0.0_m, 5.5_m};
    constexpr frc::Translation2d kAmpPose{1.85_m, 8.2_m};
    constexpr frc::Translation2d kSourcePose{15.65_m, 0.55_m};
}

namespace AutonConstants{
    //Starting Pose for Autos and OdomReset
    constexpr frc::Pose2d kDefaultStartingPose{{1.3_m, 5.5_m}, {180_deg}};
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

    constexpr int kBackLeftEncoderPort = 0;
    constexpr int kFrontLeftEncoderPort = 1;
    constexpr int kBackRightEncoderPort = 2;
    constexpr int kFrontRightEncoderPort = 3;
    // PREVIOUS
    // constexpr double kBLeftMagPos = 0.839401220985031;
    // constexpr double kFLeftMagPos = 0.440911911022798;
    // constexpr double kBRightMagPos = 0.751331268783282;
    // constexpr double kFRightMagPos = 0.299500257487506;
    constexpr double kBLeftMagPos = 0.5639529465050737;
    constexpr double kFLeftMagPos = 0.787052346955813;
    constexpr double kBRightMagPos = 0.71711400363785;
    constexpr double kFRightMagPos = 0.869769920694248;
    
    // info for odometry
    constexpr int kEncoderResolution = 4096;
    constexpr double kWheelRadius = .0495;
    constexpr double kDriveReduction = 0.9075;
    constexpr double kDriveRatio = 1 / 4.125 * kDriveReduction;
    constexpr double kTurnRatio = 1 / 13.37141;
    constexpr double kDriveDistancePerRev = (2 * std::numbers::pi * kWheelRadius) * kDriveRatio;
    constexpr auto kDriveBaseRadius = 0.319786_m;
    // constexpr double kDriveEncoderDistancePerPulse = (2 * std::numbers::pi * kWheelRadius / (double)kEncoderResolution) * kDriveRatio;
    // constexpr double kTurnEncoderDegreesPerPulse = (360.0 / (double)kEncoderResolution) * kTurnRatio;
    constexpr double kTurnEncoderDegreesPerPulse = 360.0;

    // states
    constexpr int kArbitrary = 0;
    constexpr int kSpeaker = 1;
    constexpr int kAmp = 2;
    constexpr int kSource = 3;

    // TalonFX config
    constexpr auto driveKf = 0.05614;
    constexpr auto driveKp = 0.1;

    constexpr auto turnKp = 0.275;

    // drivetrain controller constants
    constexpr double kDriveDeadzone = 0.1;
    constexpr double kDriveCurveExtent = 0.6;
    constexpr auto kDriveAccelerationLimit = 4.5_mps_sq;
    constexpr auto kDriveDecelerationLimit = 15.0_mps_sq;
    constexpr auto kDriveTranslationLimit = 7.7_mps;
    constexpr double kTxAdjust = 3.0;
    constexpr double kPVelOffset = 15.8;
}  // namespace DriveConstants



namespace AutoConstants {
        // for PID/pathfinding
        constexpr auto kMaxSpeed = 5.457_mps;
        constexpr auto kMaxAcceleration = 11.191_mps_sq;
        constexpr auto kAngularSpeed = 688_deg_per_s;
        constexpr auto kMaxAngularAcceleration = 1198.51_deg_per_s_sq;

        constexpr double kPXController = 0.4;
        constexpr double kPYController = 0.4;
        constexpr double kPThetaController = 0.0;
	
} 

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants
