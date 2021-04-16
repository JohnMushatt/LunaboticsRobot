// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#define ANALOG_CHANNELS (4)
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  this->InitializeAnalogInput(0,4);
  wpi::outs() << "PDP Current: " << this->PDP.GetTotalCurrent() << "\n";
  
  
  srx.Set(ControlMode::PercentOutput,0);
  this->TALON_LINACT.SetPosition(1.0);
  wpi::outs() << "Talon: " << this->TALON_LINACT.GetPosition() << "\n";

  //std::function<void ()> cb = Robot::ReadAnalogChannel0Callback;
  //this->AddPeriodic(std::bind(&Robot::ReadAnalogChannel0Callback,this),2_s,5_s);
  //this->AddPeriodic(std::bind(&Robot::ChangePWM0Callback,this),5_s,1_s);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
      //double_t res = this->ReadAnalogIn(0);

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}
void Robot::InitializeAnalogInput(uint64_t channel, uint64_t bits) {
  if(channel > 3) {
    perror("in initializeAnalogInput");
  }
  if(this->VEC_ANALOG_IN.size()==0 || this->VEC_ANALOG_IN.size() < channel) {
    this->VEC_ANALOG_IN.push_back(frc::AnalogInput(channel));
  }
  //this->VEC_ANALOG_IN[channel] = frc::AnalogInput(channel);
  this->VEC_ANALOG_IN[channel].SetOversampleBits(bits);
  //wpi::outs() << "[+] Set analog input " << channel << " oversample rate to " << bits << " bits\n"; 
}
double_t Robot::ReadAnalogIn(uint64_t channel) {
  if(channel > 3) {
    perror("in initializeAnalogInput");
  }
  double_t AVG_Voltage = this->VEC_ANALOG_IN[channel].GetAverageVoltage(); 
//  wpi::outs() << "[+] Read analog input " << channel << " : " << AVG_Voltage << " V (average volts)\n"; 

  return AVG_Voltage;
    //uint64_t getValue = this->A0_IN.GetValue();
    //uint64_t getAvgValue = this->A0_IN.GetAverageValue();
    //wpi::outs() << AVG_Voltage << " " << getValue << " " << getAvgValue<< "\n"; 
}
void Robot::ReadPDPChannel(uint64_t channel) {
  if(channel > 15) {
    perror("In ReadPDPChannel");
  }
  double_t Reading =this->PDP.GetCurrent(channel); 
  frc::SmartDashboard::PutNumber(std::string("Current Channel " + channel),Reading);
}
void Robot::ReadAnalogChannel0Callback() {
  this->AN_0_IN = this->VEC_ANALOG_IN[0].GetAverageVoltage();;
  wpi::outs() << "Current AN_0_IN reading: " << this->AN_0_IN <<"\n";
}
void Robot::InitializeTalonLinearActuator(uint64_t channel) {
  this->TALON_LINACT.SetRaw(0xff);
}

void Robot::ChangePWM0Callback() {
  if(this->TALON_LINACT.GetRaw() ==  2000) {
    this->TALON_LINACT.SetRaw(0x00);
  }
  else {
    this->TALON_LINACT.SetRaw(2000);
  }
  wpi::outs() << "Updated PWM0 to " << this->TALON_LINACT.GetPosition() << "\n";
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
