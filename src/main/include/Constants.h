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
    constexpr int kNote = 4;
    constexpr int kStage = 5;

    constexpr frc::Translation2d kBlueSpeakerPose{0.0_m, 5.5_m};
    constexpr frc::Translation2d kRedSpeakerPose{16.5_m, 5.5_m};
    constexpr frc::Translation2d kBlueAmpPose{1.85_m, 8.2_m};
    constexpr frc::Translation2d kRedAmpPose{14.7_m, 8.2_m};
    constexpr frc::Translation2d kBlueSourcePose{15.65_m, 0.55_m};
    constexpr frc::Translation2d kRedSourcePose{15.65_m, 0.55_m};

    constexpr frc::Translation2d kShootingBox[] = {frc::Translation2d(1.2_m, 2.0_m), frc::Translation2d(4.6_m, 7.7_m)};
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
    
    constexpr double kBLeftMagPos = 0.07177734375;
    constexpr double kFLeftMagPos = -0.31103515625;
    constexpr double kBRightMagPos = -0.228759765625;
    constexpr double kFRightMagPos = -0.439697265625;
    
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
    constexpr int kSpeaker = 1;
    constexpr int kAmp = 2;
    constexpr int kSource = 3;
    constexpr int kStage = 5;

    // TalonFX config
    constexpr double kDriveP = 0.2402345985174179;
    constexpr double kDriveV = 0.1345313787460327;
    constexpr double kDriveRamp = 0.0;
    constexpr double kDriveCurrentLimit = 30.0;
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
}  // namespace DriveConstants

namespace ArmConstants {
    // ports
    constexpr int kLeftMotorPort = 12;
    constexpr int kRightMotorPort = 13;
    constexpr int kEncoderPort = 12;

    constexpr double kEncoderOffset = -0.447509765625;

    constexpr double kRampSeconds = 0.5;

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
    constexpr double kArmDegreeMax = 85.0; // degrees the arm is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 0.00278;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.04;
    // arm position constants
    constexpr double kStartOffset = 0.0;
    constexpr double kStartAngle = 30.0;
    // position deadzone
    constexpr double kPositionDeadzone = 2.0;
    // TalonFX config
    constexpr double kP = 24;
    constexpr double kRotorToGearbox = 25.0;
    // Speaker distance targets 
    // angle = f(m)
    // Each point is sampled {m, angle}
    constexpr double kSpeakerTarget[][2] = {
        {0.00, 63.5},
        {0.25, 58.5},
        {0.50, 53.5},
        {0.75, 48.5},
        {1.00, 46.5},
        {1.23, 40.5},
        {1.50, 37.0},
        {1.75, 35.5},
        {2.00, 33.5},
        {2.23, 32.0},
        {2.5, 31.0},
        {3.8, 27.0},
        {40.0, 29.0},
        {69.0, 69.0}
    };
    // Amp distance targets 
    constexpr double kAmpTarget[][2] = {
        {0.0, 63.0},
        {40.0, 63.0},
        {69.0, 69.0}
    };
    // Source distance targets 
    constexpr double kSourceTarget[][2] = {
        {0.0, 70.0},
        {40.0, 70.0},
        {69.0, 69.0}
    };
    //Stage distance targets
    constexpr double kStageTarget[][2] = {
        {0.0, 83.0},
        {50.0, 83.0},
        {69.0, 69.0}
    };
}

namespace ShooterConstants {
    // ports
    constexpr int kLeftMotorPort = 9;
    constexpr int kRightMotorPort = 10;
    constexpr int kIndexerPort = 19;
    constexpr int kShooterBeamBreakReceiverPort = 0;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kRpmMode = 2;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    constexpr double kVelDeadzone = 200.0;
    // Indexer Constants
    constexpr double kIndexerReduction = 1.0 / 3.0;
    constexpr double kIndexerDeadzone = 0.45;
    constexpr double kIndexerOffset = -0.27;
    constexpr double kIndexerResting = -0.27;
    constexpr double kIndexerHolding = 0.0;
    constexpr double kIndexerPrimed = -0.98;
    constexpr double kIndexerKicking = -1.32;
    constexpr double kPIndexer = 0.3;
    constexpr double kRedThreshold = 2000.0;
    constexpr double kBrightnessThreshold = 2500.0;
    constexpr double kIndexThreshold = 1.3;
    constexpr double kDeindexThreshold = 0.8;
    // TalonFX config
    constexpr int kP = 0.012;
    // Speaker distance targets 
    // rpm = f(m)
    // Each point is sampled {m, rpm}
    constexpr double kSpeakerTarget[][2] = {
        {0.00, 2000.0},
        {0.25, 2750.0},
        {0.50, 3000.0},
        {0.75, 3250.0},
        {1.00, 3600.0},
        {1.23, 3900.0},
        {1.50, 4300.0},
        {1.75, 4500.0}, //1.75, 4100, .8
        {2.00, 4500.0}, //2.0, 4500,.85
        {2.23, 4700.0}, //2.0, 4500,.85
        {2.5, 5000.0}, //2.0, 4500,.85
        {5.0, 5000.0},
        {40.0, 5000.0},
        {69.0, 69.0}
    };
    // Amp distance targets
    constexpr double kAmpTarget[][2] = {
        {0.0, 800.0},//No bottom offset, Top offset at 0.4
        {40.0, 800.0},
        {69.0, 69.0}
    };
    // Source distance targets
    constexpr double kSourceTarget[][2] = {
        {0.0, -1000.0},
        {40.0, -1000.0},
        {69.0, 69.0}
    };
    // Stage distance target (stays 1000 RPM)
    constexpr double kStageTarget[][2] = {
        {0.0, 1000.0},
        {50.0, 1000.0},
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
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 0.2777777777;
    constexpr double kRotorToGearbox = 100.0;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.06;
    // arm position constants
    constexpr double kStartPosition = 0.0;
    constexpr double kPreparePos = -279.0;
    constexpr double kClimbedPos = 0.0;
    // position deadzone
    constexpr int kPositionDeadzone = 3.6;
    // TalonFX config
    constexpr double kP = 80;
    constexpr double kRampSeconds = 0.0;

    
}

namespace TrapConstants {
    // ports
    constexpr int kHatchPort = 16;
    // states
    constexpr int kOff = 0;
    constexpr int kPowerMode = 1;
    constexpr int kAngleMode = 2;
    constexpr int kFollowMode = 3;
    // default power in power mode
    constexpr double kDefaultPower = 1.0;
    // arm min/max positions
    constexpr double kArmDegreeMin = 0.0;   // encoder Turns at the arm's minimum usable position
    constexpr double kArmDegreeMax = 300.0; // degrees the arm is capable of moving 
    // for arm feed forward trig
    constexpr double kTurnsPerDegree = 0.025;
    // feed forward at max gravity i.e. 90 degrees off the floor
    constexpr double kMaxFeedForward = 0.06;
    // arm position constants
    constexpr double kStartAngle = 110.0;
    // position deadzone
    constexpr int kPositionDeadzone = 3.6;
    // TalonFX config
    constexpr int kP = 0.012;
}

namespace IntakeConstants {
    // ports
    constexpr int kIntakePort = 8;
    // states
    constexpr int kOff = 0;
    constexpr int kFullMode = 1;
    constexpr int kPowerMode = 2;
    constexpr int kAutoMode = 3;
    // power in full power mode
    constexpr double kFullPower = 1.0;
    constexpr double kIntakeSpeed = 0.45;
    // current limit
    constexpr double kCurrentLimit = 15.0;
    // power deadzone
    constexpr double kIntakeDeadzone = 0.1;
}

namespace LimelightConstants {
    // states
    constexpr int kOff = 0;
    constexpr int kOn = 1;
    constexpr int kBlink = 2;
    constexpr int kUsePipeline = 3;
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
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

namespace LEDConstants {
    constexpr int kBlinkin1Port = 0;
    constexpr int kBlinkin2Port = 1;
    constexpr double kIdlePreset = 0.41;
    constexpr double kClimbedPreset =-0.95;
    constexpr double kNoteAutoAlign = 0.57;
    constexpr double kShootReadyPreset = 0.87;
    constexpr double kShootNotReadyPreset = 0.61;
    constexpr double kNoteIndexed = -0.23;
}   //namespace BlinkinConstants