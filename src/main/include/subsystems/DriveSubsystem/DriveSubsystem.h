// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <frc/DriverStation.h>

#include "GlobalConstants.h"
#include "SwerveModule.h"
#include "subsystems/JetsonSubsystem/JetsonSubsystem.h"

using namespace frc;
using namespace ctre::phoenix6;
using namespace rev;
using namespace DriveConstants;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem(JetsonSubsystem *jetRef, int *targetRef);

    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  void HandleTargeting();

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(frc::ChassisSpeeds speeds,
    bool applyLimits = false, 
    bool fieldRelative = false);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets invert status of the drive motors.
   */
  void SetInverted(bool inverted);

  /**
   * Sets the swerve modules to a SwerveModuleState.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates, bool desaturate = true);

  /**
   * Gets an array of the swerve modules states.
   */
  wpi::array<SwerveModuleState, 4> GetModuleStates() const;

  /**
   * Generates a command to follow the path passed in.
   */
  frc2::CommandPtr FollowPathCommand(std::string path);
  frc2::CommandPtr Aimbot();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetDrivePower(double power);

  /**
   * Sets the theta MotorControllers to a power from -1 to 1.
   */
  void SetTurnPower(double power);

  /**
   * Returns the degrees of the robot.
   *
   * @return the robot's degrees, from -180 to 180
   */
  units::degree_t GetAngle();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  frc::Rotation2d GetRotation();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Resets the rate limiter.
   */
  void ResetRateLimiter();

  /**
   * Enable or disable acceleration limiting.
   *
   * @param state Whether or not limiting is enabled.
   */
  void SetLimiting(bool state);

   /**
   * Enable or disable braking.
   *
   * @param state Whether or not braking is enabled.
   */
  void SetBrakeMode(bool state);

   /**
   * Initially configure onboard TalonFX settings for the drive motors.
   */
  void ConfigDriveMotors();
   
   /**
   * Initially configure onboard TalonFX settings for the theta motors & absolute encoders.
   */
  void ConfigThetaMotors();

  /**
   * Configure the drivetrain's autonomous controller.
   */
  void ConfigAutonController();

   /**
   * Returns the pitch of the robot.
   *
   * @return the robot's pitch, from -180 to 180
   */
  double GetPitch();
  
  /**
   * Give DriveSubsystem a position to hold at.
   */
  void SetPoseToHold(frc::Pose2d target);

  /**
   * Returns the current Pose2d the robot is holding at.
   *
   * @return the Pose2d the robot is set to hold to
   */
  frc::Pose2d GetPoseToHold();

  /**
   * Reset the odometry based on confidence of April Tag detections.
   */ 
  void ResetFromJetson();

  /**
   * Give DriveSubsystem a theta to hold at.
   */
  void SetThetaToHold(frc::Rotation2d target);
  
  /**
   * Get whether omega override is enabled.
   */
  bool GetOmegaOverride();
  
  /**
   * Set whether omega override is enabled.
   */
  void SetOmegaOverride(bool state);

  /**
   * Get whether Y override is enabled.
   */
  bool GetYOverride();
  
  /**
   * Set whether Y override is enabled.
   */
  void SetYOverride(bool state);

  bool IsAtTarget();

  units::length::meter_t GetDistToTarget();

  // Positions of the SwerveModules relative to the center of the robot.
  // X+ is towards the front of the bot, Y+ is towards the left of the robot.
  // The coordinates are weird because of the field coordinate system, I don't make the rules.
  // Florida, France, Bland, Brazil
  frc::Translation2d frontLeftLocation{0.319786_m, 0.319786_m};
  frc::Translation2d frontRightLocation{0.319786_m, -0.319786_m};
  frc::Translation2d backLeftLocation{-0.319786_m, 0.319786_m};
  frc::Translation2d backRightLocation{-0.319786_m, -0.319786_m};

  // Kinematics to generate swerve module states. The order the locations goes in is the , bool desaturateorder they come out from other functions.
  frc::SwerveDriveKinematics<4> kDriveKinematics{frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

 private:
  bool enableLimiting = false;  // flag for SlewRateLimiters

  //shooter hack
  bool shooting = false;
  JetsonSubsystem *jetson;
  double targetAngle;
  double difference;
  double angle;
  std::vector<double> pos;

  int *thetaTarget;

  bool omegaOverride = false;
  bool yOverride = false;
  bool targetUsingLimelight = true;
  bool isAtTarget = false;
  frc::Rotation2d targetTheta{0.0_deg};
  std::vector<double> targetPos{6};
  double tx = 0.0;
 
  frc::Pose2d poseToHold{}; // var to contain target pose
  // PID controllers for turn holding
  frc::PIDController thetaHoldController{0.13, 0.0, 0.0};
  int lastTarget = GlobalConstants::kArbitrary;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  //Wheel motors
  hardware::TalonFX backLeft;
  hardware::TalonFX frontLeft;
  hardware::TalonFX backRight;
  hardware::TalonFX frontRight;

  //Degree of wheel motors
  hardware::TalonFX backLeftTheta;
  hardware::TalonFX frontLeftTheta;
  hardware::TalonFX backRightTheta;
  hardware::TalonFX frontRightTheta;

  //Degree of wheel motors
  hardware::CANcoder blCANCoder;
  hardware::CANcoder flCANCoder;
  hardware::CANcoder brCANCoder;
  hardware::CANcoder frCANCoder;

  //Swerve motor groups
  SwerveModule s_backLeft;
  SwerveModule s_frontLeft;
  SwerveModule s_backRight;
  SwerveModule s_frontRight;
  
  // The gyro sensor
  hardware::Pigeon2 gyro;

  // Odometry class for tracking robot pose
  frc::SwerveDriveOdometry<4> odometry;

  // SlewRateLimiters for driving. They limit the max accel/decel of the drivetrain.
  SlewRateLimiter<units::meters_per_second> xAccel;
  SlewRateLimiter<units::meters_per_second> yAccel;
  SlewRateLimiter<units::meters_per_second> xDecel;
  SlewRateLimiter<units::meters_per_second> yDecel;
  double lastX = 0.0;
  double lastY = 0.0;

  int distSample = 0;
  double distArray[kDistSamples];
  units::length::meter_t distFromTarget{0.0_m};
  Field2d fieldWidget;
};
