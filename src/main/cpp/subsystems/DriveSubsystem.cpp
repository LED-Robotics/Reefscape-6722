// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

using namespace frc;
using namespace rev;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
      //Wheel motors
    : backLeft{kBackLeftPort},
      frontLeft{kFrontLeftPort},
      backRight{kBackRightPort},
      frontRight{kFrontRightPort},
      trapOpener{21},

      //Degree of wheel motors
      backLeftTheta{kBackLeftThetaPort},
      frontLeftTheta{kFrontLeftThetaPort},
      backRightTheta{kBackRightThetaPort},
      frontRightTheta{kFrontRightThetaPort},

      //Mag encoder motor controllers
      backLeftEncoder{kBackLeftEncoderPort},
      frontLeftEncoder{kFrontLeftEncoderPort},
      backRightEncoder{kBackRightEncoderPort},
      frontRightEncoder{kFrontRightEncoderPort},

      //Swerve group motors
      s_backLeft{&backLeft, &backLeftTheta},
      s_frontLeft{&frontLeft, &frontLeftTheta},
      s_backRight{&backRight, &backRightTheta},
      s_frontRight{&frontRight, &frontRightTheta},


      //Gryo
      gyro{0},

      //Odometry
      odometry{kDriveKinematics, {GetRotation()}, {s_frontLeft.GetPosition(), s_frontRight.GetPosition(), s_backLeft.GetPosition(),
      s_backRight.GetPosition()}, frc::Pose2d{{4.964_m, 6.402_m}, {0_deg}}},
      
      xAccel{kDriveAccelerationLimit},
      yAccel{kDriveAccelerationLimit},
      xDecel{kDriveDecelerationLimit},
      yDecel{kDriveDecelerationLimit} {
        std::cout << "Drive Constructor\n";
        ResetEncoders();
        ZeroSwervePosition();
        SmartDashboard::PutBoolean("TrapThingy", trapRelease);


        // backLeft.SetInverted(true);
        // frontLeft.SetInverted(true);
        // backRight.SetInverted(true);
        // frontRight.SetInverted(true);

        // backLeftTheta.SetInverted(true);
        // frontLeftTheta.SetInverted(true);
        // backRightTheta.SetInverted(true);
        // frontRightTheta.SetInverted(true);
        
        SmartDashboard::PutNumber("Theta Target", 0.0);
        SmartDashboard::PutNumber("Angle Target", 0.0);

        SmartDashboard::PutNumber("offP", kPVelOffset);
        SmartDashboard::PutNumber("turnP", kTxAdjust);


        // ResetEncoders();
        ResetOdometry(frc::Pose2d{{2.93455_m, 6.99768_m}, {0_deg}});
        // ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {180_deg}});
        // ResetOdometry(frc::Pose2d{{0.0_m, 0.0_m}, {90_deg}});

      // Configure the AutoBuilder last
    AutoBuilder::configureHolonomic(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // THIS SHIT WILL CHANGE Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return kDriveKinematics.ToChassisSpeeds(GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ Drive(speeds);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            AutoConstants::kMaxSpeed, // Max module speed, in m/s
            DriveConstants::kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );


        std::cout << "Drive Constructor End\n";
      }

void DriveSubsystem::Periodic() {
  // Encoder Vals
  SmartDashboard::PutNumber("BL Abs", backLeftEncoder.GetAbsolutePosition());
  SmartDashboard::PutNumber("FL Abs", frontLeftEncoder.GetAbsolutePosition());
  SmartDashboard::PutNumber("BR Abs", backRightEncoder.GetAbsolutePosition());
  SmartDashboard::PutNumber("FR Abs", frontRightEncoder.GetAbsolutePosition());
  trapRelease = SmartDashboard::GetBoolean("TrapThingy", trapRelease);
  // trapOpener.Set(trapRelease);
  trapRelease ? trapOpener.Set(1.0) : trapOpener.Set(0.0);
  // SmartDashboard::PutNumber("BL Pos", (double)s_backLeft.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("FL Pos", (double)s_frontLeft.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("BR Pos", (double)s_backRight.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("FR Pos", (double)s_frontRight.GetTurnEncoderAngle());

  // SetThetaToHold({units::angle::degree_t{SmartDashboard::GetNumber("Theta Target", 0.0)}});
  // SmartDashboard::PutBoolean("Omega Override State", omegaOverride);

  odometry.Update(GetRotation(),
                  {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
                  s_backLeft.GetPosition(), s_backRight.GetPosition()});
  auto pose = odometry.GetPose();
  SmartDashboard::PutNumber("poseX", (double)pose.X());
  SmartDashboard::PutNumber("poseY", (double)pose.Y());
  SmartDashboard::PutNumber("poseAngle", (double)pose.Rotation().Degrees());
  // SmartDashboard::PutNumber("frontLeftVel", (double)s_frontLeft.GetState().speed);
  // SmartDashboard::PutNumber("frontRightVel", (double)s_frontRight.GetState().speed);
  // HandleTargeting();

}

void DriveSubsystem::HandleTargeting() {
  auto pose = odometry.GetPose();

  SmartDashboard::PutNumber("Targ Distance", distFromTarget.value());
  double distX = 0.0;
  double distY = 0.0;
  // handle coordinate-system targeting
  switch(*thetaTarget) {
    case GlobalConstants::kSpeaker:
      distFromTarget = pose.Translation().Distance(GlobalConstants::kSpeakerPose);
      distX = 0.0 - pose.Translation().X().value(); // targX - robotX
      distY = 5.5 - pose.Translation().Y().value(); // targY - robotY
      break;
    case GlobalConstants::kAmp:
      distFromTarget = pose.Translation().Distance(GlobalConstants::kAmpPose);

      distX = 1.9 - pose.Translation().X().value();
      distY = 8.15 - pose.Translation().Y().value();
      break;
    case GlobalConstants::kSource:
      distFromTarget = pose.Translation().Distance(GlobalConstants::kSourcePose);
      distX = 15.65 - pose.Translation().X().value();
      distY = 0.54 - pose.Translation().Y().value();
      break;
  }
  double theta = 0.0;
  if(*thetaTarget != GlobalConstants::kArbitrary) {
    theta = atan(distY/distX);
    theta *= (180.0/M_PI); // target is valid if vector is in quadrant 1
    if(distY > 0.0 && distX < 0.0) {
      theta = 180.0 + theta;  // transform for quadrant 2
    } else if(distY < 0.0 && distX < 0.0) {
      theta = 180.0 + theta;  // transform for quadrant 3
    } else if(distY < 0.0 && distX > 0.0) {
      theta = 360.0 + theta;  // transform for quadrant 4
    }
    SmartDashboard::PutNumber("TargetD", theta);
  }
  SmartDashboard::PutNumber("thetaTarget", theta);
  SetThetaToHold({units::degree_t{theta}});
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds,
  bool applyLimits, bool fieldRelative) {
  units::meters_per_second_t x = speeds.vx;
  units::meters_per_second_t y = speeds.vy;
  units::angular_velocity::radians_per_second_t rot = speeds.omega;
  if(omegaOverride) {
    double angle = GetPose().Rotation().Degrees().value();
    double target = SwerveModule::PlaceInAppropriate0To360Scope(thetaHoldController.GetSetpoint(), angle);
    double val = thetaHoldController.Calculate(target);
    rot = units::angular_velocity::radians_per_second_t{val};
  }
  // arbitrary speed component adjustments
  x *= 1.0;
  y *= 1.0;
  rot *= 1.0;

  if(enableLimiting && applyLimits) {
    auto xUp = xAccel.Calculate(x);
    auto yUp = yAccel.Calculate(y);
    auto xDown = xDecel.Calculate(x);
    auto yDown = yDecel.Calculate(y);
    x = fabs(x.value()) > lastX ? xUp : xDown;
    y = fabs(y.value()) > lastY ? yUp : yDown;
  } 
  lastX = fabs(x.value());
  lastY = fabs(y.value());

  SmartDashboard::PutNumber("targetXVel", x.value());
  SmartDashboard::PutNumber("targetYVel", y.value());
  SmartDashboard::PutNumber("targetOmega", rot.value());
  SmartDashboard::PutBoolean("fieldCentric", fieldRelative);
  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        x, y, rot, GetPose().Rotation())
      : frc::ChassisSpeeds{x, y, rot});

  if(!applyLimits) kDriveKinematics.DesaturateWheelSpeeds(&states, kDriveTranslationLimit);

  SetModuleStates(states, applyLimits);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, 4> desiredStates, bool desaturate) {
  if(desaturate) kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kDriveTranslationLimit);
  SmartDashboard::PutNumber("FL Target Angle", (double)desiredStates[0].angle.Degrees());
    s_frontLeft.SetDesiredState(desiredStates[0]);
  SmartDashboard::PutNumber("FR Target Angle", (double)desiredStates[1].angle.Degrees());
    s_frontRight.SetDesiredState(desiredStates[1]);
  SmartDashboard::PutNumber("BL Target Angle", (double)desiredStates[2].angle.Degrees());
    s_backLeft.SetDesiredState(desiredStates[2]);
  SmartDashboard::PutNumber("BR Target Angle", (double)desiredStates[2].angle.Degrees());
    s_backRight.SetDesiredState(desiredStates[3]);
}


wpi::array<SwerveModuleState, 4> DriveSubsystem::GetModuleStates() const {
  return {s_frontLeft.GetState(), s_frontRight.GetState(), s_backLeft.GetState(), s_backRight.GetState()};
}

frc2::CommandPtr  DriveSubsystem::FollowPathCommand(std::shared_ptr<pathplanner::PathPlannerPath> path){

    return FollowPathHolonomic(
        path,
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](){ return kDriveKinematics.ToChassisSpeeds(GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            AutoConstants::kMaxSpeed, // Max module speed, in m/s
            DriveConstants::kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        { this } // Reference to this subsystem to set requirements
    ).ToPtr();
}

void DriveSubsystem::SetDrivePower(double power) {
  // std::cout << "Power: " << power << '\n';
  // std::cout << "Velocity: " << backLeft.GetSelectedSensorVelocity() << '\n';
  s_frontLeft.SetDrivePower(power);
  s_frontRight.SetDrivePower(power);
  s_backLeft.SetDrivePower(power);
  s_backRight.SetDrivePower(power);
}

void DriveSubsystem::SetTurnPower(double power) {
  s_frontLeft.SetTurnPower(power);
  s_frontRight.SetTurnPower(power);
  s_backLeft.SetTurnPower(power);
  s_backRight.SetTurnPower(power);
}

void DriveSubsystem::ZeroSwervePosition() {
  DutyCycleEncoder *absEncoders[4] = {&backLeftEncoder, &frontLeftEncoder, &backRightEncoder, &frontRightEncoder}; // mag encoder TalonSRX ref arr
  hardware::TalonFX *turnMotors[4] = {&backLeftTheta, &frontLeftTheta, &backRightTheta, &frontRightTheta}; // motor ref arr
  double poses[4] = {kBLeftMagPos, kFLeftMagPos, kBRightMagPos, kFRightMagPos}; // arr of correct swerve mag poses
  // iterate through each swerve module and offset theta motors so that they match the absolute mag encoder positions
  for(int i = 0; i < 4; i++) {
    double pos = 0.0;
    for(int j = 0; j < 200000; j++) {
      pos = absEncoders[i]->GetAbsolutePosition();
      if(pos != 0.0) break;
    }
    turnMotors[i]->SetPosition(units::angle::turn_t{poses[i] - pos});
  }
}

void DriveSubsystem::ResetEncoders() {
  s_frontLeft.ResetEncoders();
  s_backLeft.ResetEncoders();
  s_frontRight.ResetEncoders();
  s_backRight.ResetEncoders();
}

void DriveSubsystem::SetInverted(bool inverted) {
  backLeft.SetInverted(inverted);
  frontLeft.SetInverted(inverted);
  backRight.SetInverted(inverted);
  frontRight.SetInverted(inverted);
}

units::degree_t DriveSubsystem::GetAngle() const {
  return units::degree_t{-gyro.GetAngle()};
}

frc::Rotation2d DriveSubsystem::GetRotation() {
  return GetAngle();
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  // s_backLeft.ResetEncoders();
  // s_frontLeft.ResetEncoders();
  // s_backRight.ResetEncoders();
  // s_frontRight.ResetEncoders();
  odometry.ResetPosition(
    GetPose().Rotation(),
    {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
    s_backLeft.GetPosition(), s_backRight.GetPosition()},
    pose);
}

void DriveSubsystem::SetLimiting(bool state) {
  enableLimiting = state;
}

void DriveSubsystem::SetBrakeMode(bool state) {
  ctre::phoenix6::signals::NeutralModeValue mode;
  
  if(state) {
    mode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  }
  else {
    mode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  }
  ctre::phoenix6::configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);

  backLeft.GetConfigurator().Apply(updated, 50_ms);
  frontLeft.GetConfigurator().Apply(updated, 50_ms);
  backRight.GetConfigurator().Apply(updated, 50_ms);
  frontRight.GetConfigurator().Apply(updated, 50_ms);
  backLeftTheta.GetConfigurator().Apply(updated, 50_ms);
  frontLeftTheta.GetConfigurator().Apply(updated, 50_ms);
  backRightTheta.GetConfigurator().Apply(updated, 50_ms);
  frontRightTheta.GetConfigurator().Apply(updated, 50_ms);
}

double DriveSubsystem::GetPitch() {
  // return gyro.GetRoll();
  return 0.0; // temp
}

void DriveSubsystem::SetPoseToHold(frc::Pose2d target) {
  poseToHold = target;
}

frc::Pose2d DriveSubsystem::GetPoseToHold() {
  return poseToHold;
}

void DriveSubsystem::StartHolding() {
  // reset PID controllers
  xHoldController.Reset();
  yHoldController.Reset();
  thetaHoldController.Reset();

  // set PID setpoints to target components
  xHoldController.SetSetpoint((double)poseToHold.X());
  yHoldController.SetSetpoint((double)poseToHold.Y());
  thetaHoldController.SetSetpoint((double)poseToHold.Rotation().Degrees());
}

frc::ChassisSpeeds DriveSubsystem::CalculateHolding() {
  auto current = odometry.GetPose();  // current robot pose
  double angle = (double)current.Rotation().Degrees();  // current robot angle
  double currentAngle = SwerveModule::PlaceInAppropriate0To360Scope(thetaHoldController.GetSetpoint(), angle);  // angle with corrected range
  // SmartDashboard::PutNumber("angleTarget", thetaHoldController.GetSetpoint());
  // SmartDashboard::PutNumber("currentHoldAngle", currentAngle);
  // return ChassisSpeeds needed to hold position correctly
  return frc::ChassisSpeeds{units::meters_per_second_t{xHoldController.Calculate((double)current.X())}, 
  units::meters_per_second_t{yHoldController.Calculate((double)current.Y())}, 
  units::radians_per_second_t{thetaHoldController.Calculate(currentAngle)}};
}

void DriveSubsystem::SetThetaToHold(frc::Rotation2d target) {
  targetTheta = target;
  thetaHoldController.SetSetpoint(targetTheta.Degrees().value());
}

bool DriveSubsystem::GetOmegaOverride(){
  return omegaOverride;
}

void DriveSubsystem::SetOmegaOverride(bool state){
  omegaOverride = state;
}

units::length::meter_t DriveSubsystem::GetDistToTarget() {
  return distFromTarget;
}