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

  /**
   * Auto pilot variables and functions
   */
  enum ROBOT_STATE {RESET,
  DIG_EXTEND_FOURBAR,//Move the Fourbar down to digging position  
  DIG_RETRACT_FOURBAR, //Move the Fourbar back to reset/dump position
  DIG_EXTEND_SCOOP,//Extend the linear actuator aka bring the scoop in to collect material  
  DUMP_SCOOP, //Retract the linear actuator aka THIS WILL DUMP MATERIAL 
  ERR,DONE,HOLD};
  ROBOT_STATE CURRENT_ROBOT_STATE;
  ROBOT_STATE NEXT_ROBOT_STATE;
  bool AutoPilot = false;
  bool AutoPilotStarted = false;
  size_t AutoCycleCount = 0;
  double_t GetPositionThresholdValue(size_t CycleCount, ROBOT_STATE CurrentState);
  double_t GetCurrentThresholdValue(ROBOT_STATE CurrentState);
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
