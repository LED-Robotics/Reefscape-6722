// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem/DriveSubsystem.h"

#include <iostream>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

using namespace frc;
using namespace rev;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem(JetsonSubsystem *jetRef, int *targetRef)
      //Wheel motors
    : backLeft{kBackLeftPort, "canCan"},
      frontLeft{kFrontLeftPort, "canCan"},
      backRight{kBackRightPort, "canCan"},
      frontRight{kFrontRightPort, "canCan"},

      //Degree of wheel motors
      backLeftTheta{kBackLeftThetaPort, "canCan"},
      frontLeftTheta{kFrontLeftThetaPort, "canCan"},
      backRightTheta{kBackRightThetaPort, "canCan"},
      frontRightTheta{kFrontRightThetaPort, "canCan"},

      //Mag encoder motor controllers
      blCANCoder{kBackLeftEncoderPort, "canCan"},
      flCANCoder{kFrontLeftEncoderPort, "canCan"},
      brCANCoder{kBackRightEncoderPort, "canCan"},
      frCANCoder{kFrontRightEncoderPort, "canCan"},

      //Swerve group motors
      s_backLeft{&backLeft, &backLeftTheta},
      s_frontLeft{&frontLeft, &frontLeftTheta},
      s_backRight{&backRight, &backRightTheta},
      s_frontRight{&frontRight, &frontRightTheta},

      //Gryo
      gyro{0, "canCan"},

      //Odometry
      odometry{kDriveKinematics, {GetRotation()}, {s_frontLeft.GetPosition(), s_frontRight.GetPosition(), s_backLeft.GetPosition(),
      s_backRight.GetPosition()}, frc::Pose2d{{0.0_m, 0.0_m}, {0_deg}}},
      
      xAccel{kDriveAccelerationLimit},
      yAccel{kDriveAccelerationLimit},
      xDecel{kDriveDecelerationLimit},
      yDecel{kDriveDecelerationLimit} {
        jetson = jetRef;
        thetaTarget = targetRef;

        ConfigDriveMotors();
        ConfigThetaMotors();

        SmartDashboard::PutBoolean("Limelight Targeting", targetUsingLimelight);
        SmartDashboard::PutNumber("offP", kPVelTurnOffset);
        SmartDashboard::PutNumber("offD", kPVelDistOffset);
        SmartDashboard::PutNumber("turnP", kTxAdjust);

        // Configure the AutoBuilder last
        ConfigAutonController();
      }

void DriveSubsystem::Periodic() {
  // Encoder Vals
  // SmartDashboard::PutNumber("BL Abs", backLeftEncoder.GetAbsolutePosition());
  // SmartDashboard::PutNumber("FL Abs", frontLeftEncoder.GetAbsolutePosition());
  // SmartDashboard::PutNumber("BR Abs", backRightEncoder.GetAbsolutePosition());
  // SmartDashboard::PutNumber("FR Abs", frontRightEncoder.GetAbsolutePosition());

  // SmartDashboard::PutNumber("BL Pos", (double)s_backLeft.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("FL Pos", (double)s_frontLeft.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("BR Pos", (double)s_backRight.GetTurnEncoderAngle());
  // SmartDashboard::PutNumber("FR Pos", (double)s_frontRight.GetTurnEncoderAngle());

  // SetThetaToHold({units::angle::degree_t{SmartDashboard::GetNumber("Theta Target", 0.0)}});
  SmartDashboard::PutBoolean("Omega Override State", omegaOverride);
  targetUsingLimelight = SmartDashboard::GetBoolean("Limelight Targeting", targetUsingLimelight);

  odometry.Update(GetRotation(),
              {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
              s_backLeft.GetPosition(), s_backRight.GetPosition()});
  
  fieldWidget.SetRobotPose(odometry.GetPose());          
  auto pose = odometry.GetPose();
  SmartDashboard::PutNumber("poseX", (double)pose.X());
  SmartDashboard::PutNumber("poseY", (double)pose.Y());
  SmartDashboard::PutNumber("poseAngle", (double)pose.Rotation().Degrees());
  SmartDashboard::PutBoolean("isAtTarget", isAtTarget);
  SmartDashboard::PutData("Field", &fieldWidget);
  HandleTargeting();
}

void DriveSubsystem::HandleTargeting() {
  
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
        x, y, rot, GetPose().Rotation()
        .RotateBy(DriverStation::GetAlliance() == DriverStation::Alliance::kRed ? 180_deg : 0_deg))
      : frc::ChassisSpeeds{x, y, rot});

  if(!applyLimits) kDriveKinematics.DesaturateWheelSpeeds(&states, kDriveTranslationLimit);

  SetModuleStates(states, applyLimits);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, 4> desiredStates, bool desaturate) {
  if(desaturate) kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kDriveTranslationLimit);
  // SmartDashboard::PutNumber("FL Target Angle", (double)desiredStates[0].angle.Degrees());
    s_frontLeft.SetDesiredState(desiredStates[0]);
  // SmartDashboard::PutNumber("FR Target Angle", (double)desiredStates[1].angle.Degrees());
    s_frontRight.SetDesiredState(desiredStates[1]);
  // SmartDashboard::PutNumber("BL Target Angle", (double)desiredStates[2].angle.Degrees());
    s_backLeft.SetDesiredState(desiredStates[2]);
  // SmartDashboard::PutNumber("BR Target Angle", (double)desiredStates[2].angle.Degrees());
    s_backRight.SetDesiredState(desiredStates[3]);
}

wpi::array<SwerveModuleState, 4> DriveSubsystem::GetModuleStates() const {
  return {s_frontLeft.GetState(), s_frontRight.GetState(), s_backLeft.GetState(), s_backRight.GetState()};
}

frc2::CommandPtr DriveSubsystem::FollowPathCommand(std::string path){
  auto useablePath = PathPlannerPath::fromPathFile(path);
  return AutoBuilder::followPath(useablePath);
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

void DriveSubsystem::ResetEncoders() {
  s_frontLeft.ResetEncoders();
  s_backLeft.ResetEncoders();
  s_frontRight.ResetEncoders();
  s_backRight.ResetEncoders();
}

void DriveSubsystem::SetInverted(bool inverted) {
  //Counter clockwise is inverted, clockwise is not inverted
  signals::InvertedValue configInvert;
  configs::MotorOutputConfigs updated;

  configInvert = inverted ? signals::InvertedValue::CounterClockwise_Positive : signals::InvertedValue::Clockwise_Positive;
  
  updated.WithInverted(configInvert);

  backLeft.GetConfigurator().Apply(updated, 50_ms);
  frontLeft.GetConfigurator().Apply(updated, 50_ms);
  backRight.GetConfigurator().Apply(updated, 50_ms);
  frontRight.GetConfigurator().Apply(updated, 50_ms);
}

units::degree_t DriveSubsystem::GetAngle() {
  return gyro.GetYaw().GetValue();
}

frc::Rotation2d DriveSubsystem::GetRotation() {
  return GetAngle();
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetAngularVelocityZWorld().GetValueAsDouble();
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
    GetRotation(),
    {s_frontLeft.GetPosition(), s_frontRight.GetPosition(),
    s_backLeft.GetPosition(), s_backRight.GetPosition()},
    pose);
}

void DriveSubsystem::SetLimiting(bool state) {
  enableLimiting = state;
}

void DriveSubsystem::SetBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  
  if(state) {
    mode = signals::NeutralModeValue::Brake;
  }
  else {
    mode = signals::NeutralModeValue::Coast;
  }
  configs::MotorOutputConfigs updated;
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

void DriveSubsystem::ResetFromJetson() {
  auto updatedPose = jetson->AverageRobotPose();
  if(!jetson->IsPoseAvailable()) return;
  auto rot = odometry.GetPose().Rotation();
  ResetOdometry({updatedPose.Translation(), rot});
}

void DriveSubsystem::SetThetaToHold(frc::Rotation2d target) {
  isAtTarget = false;
  targetTheta = target;
  thetaHoldController.SetSetpoint(targetTheta.Degrees().value());
}

bool DriveSubsystem::GetOmegaOverride(){
  return omegaOverride;
}

void DriveSubsystem::SetOmegaOverride(bool state){
  omegaOverride = state;
}

bool DriveSubsystem::GetYOverride(){
  return yOverride;
}

void DriveSubsystem::SetYOverride(bool state){
  yOverride = state;
}

bool DriveSubsystem::IsAtTarget() {
  // double angle = GetPose().Rotation().Degrees().value();
  // double target = SwerveModule::PlaceInAppropriate0To360Scope(thetaHoldController.GetSetpoint(), angle);
  // return angle > target - kThetaDeadzone / 2 && angle < target + kThetaDeadzone / 2;
  // if(lastTarget != *thetaTarget) isAtTarget = false;
  // lastTarget = *thetaTarget;
  return isAtTarget;
}

units::length::meter_t DriveSubsystem::GetDistToTarget() {
  return distFromTarget;
}

void DriveSubsystem::ConfigDriveMotors() {
  configs::TalonFXConfiguration driveConfig{};
  driveConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  driveConfig.Slot0.kP = kDriveP;
  driveConfig.Slot0.kV = kDriveV;
  driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kDriveRamp;
  driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveCurrentLimit;
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.Audio.AllowMusicDurDisable = true;
  
  backLeft.GetConfigurator().Apply(driveConfig);
  frontLeft.GetConfigurator().Apply(driveConfig);
  backRight.GetConfigurator().Apply(driveConfig);
  frontRight.GetConfigurator().Apply(driveConfig);
}

void DriveSubsystem::ConfigThetaMotors() {
  configs::TalonFXConfiguration turnConfig{};
  turnConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  turnConfig.Slot0.kP = kTurnP;
  turnConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
  turnConfig.Feedback.RotorToSensorRatio = kTurnPRatio;
  turnConfig.Feedback.SensorToMechanismRatio = 1.0;
  turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
  turnConfig.Audio.AllowMusicDurDisable = true;
  turnConfig.MotorOutput.Inverted = true;

  turnConfig.Feedback.FeedbackRemoteSensorID = kBackLeftEncoderPort;
  backLeftTheta.GetConfigurator().Apply(turnConfig);
  turnConfig.Feedback.FeedbackRemoteSensorID = kFrontLeftEncoderPort;
  frontLeftTheta.GetConfigurator().Apply(turnConfig);
  turnConfig.Feedback.FeedbackRemoteSensorID = kBackRightEncoderPort;
  backRightTheta.GetConfigurator().Apply(turnConfig);
  turnConfig.Feedback.FeedbackRemoteSensorID = kFrontRightEncoderPort;
  frontRightTheta.GetConfigurator().Apply(turnConfig);

  configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;
  
  encoderConfig.MagnetSensor.MagnetOffset = kBLeftMagPos;
  blCANCoder.GetConfigurator().Apply(encoderConfig);
  encoderConfig.MagnetSensor.MagnetOffset = kFLeftMagPos;
  flCANCoder.GetConfigurator().Apply(encoderConfig);
  encoderConfig.MagnetSensor.MagnetOffset = kBRightMagPos;
  brCANCoder.GetConfigurator().Apply(encoderConfig);
  encoderConfig.MagnetSensor.MagnetOffset = kFRightMagPos;
  frCANCoder.GetConfigurator().Apply(encoderConfig);
}

void DriveSubsystem::ConfigAutonController() {
  RobotConfig config = RobotConfig::fromGUISettings();
  AutoBuilder::configure(
      [this](){ return GetPose(); }, // Robot pose supplier
      [this](frc::Pose2d pose){ odometry.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
      [this](){ return kDriveKinematics.ToChassisSpeeds(GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](auto speeds, auto feedforwards){ Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
          PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
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
}