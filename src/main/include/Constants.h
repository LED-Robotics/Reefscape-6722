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
    constexpr double kBLeftMagPos = 0.5572029465050737;
    constexpr double kFLeftMagPos = 0.892232346955813;
    constexpr double kBRightMagPos = 0.71251400363785;
    constexpr double kFRightMagPos = 0.861769920694248;
    
    // info for odometry
    constexpr int kEncoderResolution = 4096;
    constexpr double kWheelRadius = .0508;
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

namespace ArmConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kAngleMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr double kArmDegreeMin = 3.0;   // encoder Turns at the arm's minimum usable position
    constexpr int kMinAngleTurns = 0;   // encoder Turns at the arm's minimum usable position
    constexpr int kMaxAngleTurns = 4000;   // encoder Turns at the arm's max usable position
    constexpr double kArmDegreeMax = 89.0; // degrees the arm is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 0.2777777777;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.06;
    // arm position constants
    constexpr double kStartOffset = 5.0;
    constexpr double kStartAngle = 20.0;
    // position deadzone
    constexpr double kPositionDeadzone = 13.0;
    // TalonFX config
    constexpr int kP = 0.012;
    // Speaker distance targets 
    // angle = f(m)
    // Each point is sampled {m, angle}
    constexpr double kSpeakerTarget[][2] = {
        {0.0, 65.0},
        {1.26, 65.0},
        {2.0, 52.0},
        {3.0, 40.94},
        {40.0, 30.0},
        {69.0, 69.0}
    };
    // Amp distance targets 
    constexpr double kAmpTarget[][2] = {
        {0.0, 69.7367941712204},
        {40.0, 69.7367941712204},
        {69.0, 69.0}
    };
    // Source distance targets 
    constexpr double kSourceTarget[][2] = {
        {0.0, 70.0},
        {40.0, 70.0},
        {69.0, 69.0}
    };
}

namespace ShooterConstants {
    // ports
    constexpr int kLeftMotorPort = 9;
    constexpr int kRightMotorPort = 10;
    constexpr int kIndexerPort = 19;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kRpmMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    constexpr double kVelDeadzone = 500.0;
    // Indexer Constants
    constexpr double kIndexerDeadzone = 0.5;
    constexpr double kIndexerOffset = -0.8;
    constexpr double kIndexerResting = -0.8;
    constexpr double kIndexerHolding = 0.0;
    constexpr double kIndexerPrimed = -2.95;
    constexpr double kIndexerKicking = -3.95;
    constexpr double kPIndexer = 0.3;
    constexpr int kRedThreshold = 15;
    // TalonFX config
    constexpr int kP = 0.012;
    // Speaker distance targets 
    // rpm = f(m)
    // Each point is sampled {m, rpm}
    constexpr double kSpeakerTarget[][2] = {
        {0.0, 2000.0},
        {1.26, 2000.0},
        {2.0, 2300.0},
        {3.0, 3000.0},
        {4.0, 3000.0},
        {5.0, 6000.0},
        {40.0, 6000.0},
        {69.0, 69.0}
    };
    // Amp distance targets
    constexpr double kAmpTarget[][2] = {
        {0.0, 800.0},
        {40.0, 800.0},
        {69.0, 69.0}
    };
    // Source distance targets
    constexpr double kSourceTarget[][2] = {
        {0.0, -1000.0},
        {40.0, -1000.0},
        {69.0, 69.0}
    };
}

namespace ClimbConstants {
    // ports
    constexpr int kLeftMotorPort = 14;
    constexpr int kRightMotorPort = 15;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kPositionMode = 2;
    constexpr int kAngleMode = 3;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr double kArmDegreeMin = 0.0;   // encoder Turns at the arm's minimum usable position
    constexpr int kMinAngleTurns = 0;   // encoder Turns at the arm's minimum usable position
    constexpr int kMaxAngleTurns = 4000;   // encoder Turns at the arm's max usable position
    constexpr double kArmDegreeMax = 300.0; // degrees the arm is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 0.2777777777;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.06;
    // arm position constants
    constexpr double kStartPosition = 0.0;
    constexpr double kStartAngle = 170.0;
    // position deadzone
    constexpr int kPositionDeadzone = 3.6;
    // TalonFX config
    constexpr int kP = 0.012;
    // Climb distance targets 
    // angle = f(m)
    // Each point is sampled {m, angle}
    constexpr double kSpeakerTarget[][2] = {
        {0.0, 170.0},
        {40.0, 170.0},
        {69.0, 69.0}
    };
    // Amp distance targets 
    constexpr double kAmpTarget[][2] = {
        {0.0, 15.08},
        {40.0, 15.08},
        {69.0, 69.0}
    };
    // Source distance targets 
    constexpr double kSourceTarget[][2] = {
        {0.0, 170.0},
        {40.0, 170.0},
        {69.0, 69.0}
    };
}

namespace IntakeConstants {
    // ports
    constexpr int kIntakePort = 8;
    // states
    constexpr int kOff = 0;
    constexpr int kFullMode = 1;
    constexpr int kPowerMode = 2;
    // power in full power mode
    constexpr double kFullPower = 1.0;
    // current limit
    constexpr double kCurrentLimit = 15.0;
    // power deadzone
    constexpr double kIntakeDeadzone = 0.1;
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
} //namespace AutoConstants

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

namespace BlinkinConstants {
    constexpr int kBlinkinLeftChassisPort = 4;
    constexpr int kBlinkinRightChassisPort = 5;
    constexpr int kBlinkinLeftShooterPort = 6;
    constexpr int kBlinkinRightShooterPort = 7;
}   //namespace BlinkinConstants