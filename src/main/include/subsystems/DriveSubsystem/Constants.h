#pragma once

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
    //Abs encoders
    constexpr int kBackLeftEncoderPort = 4;
    constexpr int kFrontLeftEncoderPort = 5;
    constexpr int kBackRightEncoderPort = 6;
    constexpr int kFrontRightEncoderPort = 7;
    
    constexpr units::angle::turn_t kBLeftMagPos = 0.31396484375_tr;
    constexpr units::angle::turn_t kFLeftMagPos = -0.46044921875_tr;
    constexpr units::angle::turn_t kBRightMagPos = 0.47021484375_tr;
    constexpr units::angle::turn_t kFRightMagPos = -0.37939453125_tr;
    
    // info for odometry
    constexpr auto kDriveBaseRadius = 0.319786_m;
    constexpr double kWheelRadius = .0512;
    constexpr double kDriveRatio = 1 / 4.41;
    constexpr double kTurnRatio = 1 / 11.3142;
    constexpr double kDriveDistancePerRev = (2 * std::numbers::pi * kWheelRadius) * kDriveRatio;
    constexpr double kTurnEncoderDegreesPerPulse = 360.0;
    constexpr auto kMOI = 60;
    constexpr units::kilogram_t kRobotWeight = 54.43_kg;

    // states
    constexpr int kArbitrary = 0;

    // TalonFX config
    constexpr double kDriveP = 0.2402345985174179;
    constexpr double kDriveV = 0.1345313787460327;
    constexpr units::time::second_t kDriveRamp = 0.0_s;
    constexpr units::current::ampere_t kDriveCurrentLimit = 30.0_A;
    constexpr double kTurnP = 80.0;
    constexpr double kTurnPRatio = 10.279000282287598;

    // drivetrain controller constants
    constexpr double kDriveDeadzone = 0.2;
    constexpr double kTurnDeadzone = 0.1;
    constexpr double kDriveCurveExtent = 0.6;
    constexpr auto kDriveAccelerationLimit = 6.0_mps_sq;
    constexpr auto kDriveDecelerationLimit = 6.0_mps_sq;
    constexpr auto kDriveTranslationLimit = 7.7_mps;
    constexpr double kThetaDeadzone = 1.0;
    constexpr double kThetaControllerDefaultP = 0.13;

}
