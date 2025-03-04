//# PINEAPPLES # LED Robotics 6722 Wendell Krinn Technical High School (Grades 09-12) partnered with Marchman Technical College, PHSC, FLVS, and eSchool <3

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  m_container.SetDriveBrakes(false);
  // m_container.ZeroSwerve();
  // SmartDashboard::PutNumber("silly pineapples at", 69);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.SetDriveBrakes(false);
}

void Robot::DisabledPeriodic() {
  
}

/**
 * This autonomous runs the autonomous command selected by your
 * RobotContainer class.
 */
void Robot::AutonomousInit() {
  // m_container.DisableTagTracking();   // auton uses odom relative to start, not based on AprilTags
  // m_autonomousCommand = m_container.GetAutonomousCommand();
  m_container.SetDriveBrakes(true);
  m_container.SetSlew(false);
  // m_container.SetAutoIndex(true);
  // if(DriverStation::IsFMSAttached()) {
  //   m_container.SetRecording(true);
  // }
  // if(m_autonomousCommand) {
  //   m_autonomousCommand->Schedule();
  // }
  // frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  // frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.get());
  // m_autonomousCommand->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_container.EnableTagTracking();  // station auto-align uses AprilTag tracking 
  m_container.SetDriveBrakes(true);
  m_container.SetSlew(true);

  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Cancel();
  // }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // std::cout << "Pulse" << '\n';
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
