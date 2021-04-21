// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PWM.h>
#include <frc/AnalogInput.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/Talon.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <Instrum.h>
#include <vector>
#include <memory>
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
	void SimulationPeriodic() override;
  double_t GetLinearActuatorTurnValue();
 private:
  int _smoothing;
  bool AutoPilot = false;
  enum ROBOT_STATE {RESET,DIG_EXTEND_FOURBAR,DIG_SLOW_EXTEND_FOURBAR,DIG_EXTEND_SCOOP,
  DIG_RETRACT_FOURBAR,DIG_RETRACT_SCOOP,DUMP,ERR,DONE,HOLD};
  ROBOT_STATE CURRENT_ROBOT_STATE;
  ROBOT_STATE NEXT_ROBOT_STATE;
  std::string GetStateAsString(ROBOT_STATE state);
  void DisplayRobotState();
  void ReadPDPChannel(uint64_t channel);
  double_t ReadAnalogIn(uint64_t channel);
  void ReadAnalogChannel0Callback();
  void ChangePWM0Callback();
  void DebugControllerButtons();
  void InitializeAnalogInput(uint64_t channel, uint64_t bits);
  void InitializeTalonLinearActuator();
  volatile double_t AN_0_IN;
  std::vector<frc::AnalogInput> VEC_ANALOG_IN;
  frc::PowerDistributionPanel PDP;
  frc::Joystick joystick{0};
  TalonSRX srx = {1};
  TalonSRX SRX_LINACT = {5};
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
