// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TestSubsystem/TestSubsystem.h"
#include "led_libraries/PositionalSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <iostream>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace TestConstants;
using namespace frc;

TestSubsystem::TestSubsystem()
  : PositionalSubsystem{std::vector<SmartMotor*>{&testController}},
    test{kTestPort},
    testEncoder{kEncoderPort}
    {
      /*test.SetPosition(0.0_tr);*/
      SmartDashboard::PutNumber("Test Angle", 90.0);
      SmartDashboard::PutNumber("microAdjustTest", 0.0);  // print to Shuffleboard
      ConfigTest();

      SetTargetAngle(angle);

}

void TestSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  // Test Control
  SetTargetAngle(units::angle::degree_t{SmartDashboard::GetNumber("Test Angle", GetAngle().value())});
  SmartDashboard::PutNumber("Test Actual", GetAngle().value());
  // feed forwards should be a changing constant that increases as the test moves further. It should be a static amount of power to overcome gravity.
  microAdjust = units::angle::degree_t{SmartDashboard::GetNumber("microAdjustTest", 0.0)};  // print to Shuffleboard
  SmartDashboard::PutNumber("testTestTr", test.GetPosition().GetValue().value());  // print to Shuffleboard
  SmartDashboard::PutNumber("angle", GetAngle().value());  // print to Shuffleboard
  double feedForward = fabs(sin(angle.value())) * kMaxFeedForward;
  SmartDashboard::PutNumber("Angle Target", angle.value());
  units::angle::turn_t posTarget{(angle + microAdjust - kTestStartAngle).value() * kTurnsPerDegree};
  SmartDashboard::PutNumber("wrTurnTarget", posTarget.value());
  // Send control signals to motors
  RunMotors();
}

void TestSubsystem::TestOn() {
  state = TestStates::kTestAngleMode;
}

void TestSubsystem::TestOff() {
  state = TestStates::kTestOff;
}

void TestSubsystem::SetTestPower(double newPower) {
  power = newPower;
}

double TestSubsystem::GetTestPower() {
  return power;
}

void TestSubsystem::SetTargetAngle(units::angle::degree_t newAngle) {
  angle = newAngle;
  if(angle < kTestDegreeMin) angle = kTestDegreeMin;
  if(angle > kTestDegreeMax) angle = kTestDegreeMax;
  SmartDashboard::PutNumber("Test Angle", angle.value());
}

units::angle::degree_t TestSubsystem::GetAngle() {
  return units::angle::degree_t{(GetTestPosition() / kTurnsPerDegree)} + kTestStartAngle;
}

double TestSubsystem::GetTestPosition() {
  return test.GetPosition().GetValueAsDouble();
}

bool TestSubsystem::IsAtTarget() {
  auto target = angle + microAdjust;
  auto angle = GetAngle();
  bool atTarget = angle > target - (kTestAngleDeadzone / 2) && angle < target + (kTestAngleDeadzone / 2);
  return atTarget;
}

void TestSubsystem::SetTestState(int newState) {
  state = newState;
}

int TestSubsystem::GetTestState() {
  return state;
}

frc2::CommandPtr TestSubsystem::GetMoveCommand(units::angle::degree_t target) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, target]() {
        SetTargetAngle(target);
      }, {this}),
      frc2::cmd::WaitUntil([this, target](){
        return IsAtTarget();
      }));
  /*return frc2::cmd::RunOnce([this, target]() {*/
  /*      SetTargetAngle(target);*/
  /*    }, {this});*/
}
void TestSubsystem::SetTestBrakeMode(bool state) {
  signals::NeutralModeValue mode;
  if(state) mode = signals::NeutralModeValue::Brake;
  else mode = signals::NeutralModeValue::Coast;
  configs::MotorOutputConfigs updated;
  updated.WithNeutralMode(mode);
  test.GetConfigurator().Apply(updated, 50_ms);
}

void TestSubsystem::ConfigTest() {
  configs::TalonFXConfiguration testConfig{};

  testConfig.Slot0.kP = kPTest;
  testConfig.Slot0.kD = kDTest;
  testConfig.MotorOutput.Inverted = false;
  // testConfig.Slot0.kS = 0.28;
  // testConfig.Slot0.kV = 8.5;
  // testConfig.Slot0.kA = 3.0;
  // testConfig.Slot0.kP = 8.0;

  // testConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
  // testConfig.MotionMagic.MotionMagicAcceleration = 2.0;
  // testConfig.MotionMagic.MotionMagicJerk = 200.0;
  
  // testConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
  testConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::SyncCANcoder;
  testConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  testConfig.ClosedLoopGeneral.ContinuousWrap = false;
  testConfig.Feedback.RotorToSensorRatio = kTestRotorToGearbox;
  testConfig.Feedback.SensorToMechanismRatio = kTestGearboxToMechanism;
  testConfig.MotorOutput.PeakReverseDutyCycle = -1.0;
  testConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
  // testConfig.Feedback.SensorToMechanismRatio = 1.0;
  testConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  testConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
  testConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampSeconds;
  testConfig.Audio.AllowMusicDurDisable = true;

  testConfig.Feedback.FeedbackRemoteSensorID = kEncoderPort;
  
  test.GetConfigurator().Apply(testConfig);

  configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
  encoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  encoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  
  testEncoder.GetConfigurator().Apply(encoderConfig);
}
