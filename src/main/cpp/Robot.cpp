// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>

#include <iostream>
#include <PhysicsSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#define ANALOG_CHANNELS (4)
#define PDPChannel_LinearActuator (0)

#define FOURBAR_EXTENSION_LIMIT (4000.0)
#define FOURBAR_RETRACTION_LIMIT (1000.0)

#define SCOOP_EXTENSION_LIMIT (10.0)
#define SCOOP_RETRACTION_LIMIT (1.0)
//TODO Add negative (reverse direction) values
#define FOURBAR_OUTPUT_ZERO (0.0)
#define FOURBAR_OUTPUT_HALF (.5)
#define FOURBAR_OUTPUT_FULL (1.0)

#define LINACT_OUTPUT_ZERO (0.0)
#define LINACT_OUTPUT_HALF (.5)
#define LINACT_OUTPUT_FULL (1.0)


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  this->InitializeAnalogInput(0,4);
  wpi::outs() << "PDP Current: " << this->PDP.GetTotalCurrent() << "\n";
  
  this->InitializeTalonLinearActuator();
  this->AddPeriodic(std::bind(&Robot::ReadAnalogChannel0Callback,this),1_s,0_s);
  //this->AddPeriodic(std::bind(&Robot::DebugControllerButtons,this),1_s,1_s);
}
void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonSRX(srx,.75,3400,false);
  PhysicsSim::GetInstance().AddTalonSRX(SRX_LINACT,.75,3400,false);
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();
}
void Robot::DebugControllerButtons() {
  
  int TotalButtons = this->joystick.GetButtonCount();
  for(size_t ButtonIndex = 1; ButtonIndex < TotalButtons; ButtonIndex++) {
    if(this->joystick.GetRawButtonPressed(ButtonIndex)) {
      wpi::outs() << "Button " << ButtonIndex << " has been pressed\n";
    }
  }
  int TotalAxis = this->joystick.GetAxisCount();
  for(size_t AxisIndex = 0; AxisIndex < TotalAxis;AxisIndex++) {
    if(this->joystick.GetRawAxis(AxisIndex)) {
      wpi::outs() << "Axis " << AxisIndex << " w/ type " << this->joystick.GetAxisType(AxisIndex) << " @ " 
      << this->joystick.GetRawAxis(AxisIndex) << "\n";
    }
  }
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
  this->CURRENT_ROBOT_STATE = RESET;
  m_autoSelected = m_chooser.GetSelected();
  srx.SetSelectedSensorPosition(0,0,10);
  SRX_LINACT.SetSelectedSensorPosition(0,0,10);
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}
std::string Robot::GetStateAsString(ROBOT_STATE state) {

  switch (state) {
    case ROBOT_STATE::RESET:
      return std::string("RESET");
    case ROBOT_STATE::DIG_EXTEND_FOURBAR:
      return std::string("DIG_EXTEND_FOURBAR");    
    case ROBOT_STATE::DIG_EXTEND_SCOOP:
      return std::string("DIG_EXTEND_SCOOP");    
    case ROBOT_STATE::DIG_RETRACT_FOURBAR:
      return std::string("DIG_RETRACT_FOURBAR");    
    case ROBOT_STATE::DIG_RETRACT_SCOOP:
      return std::string("DIG_RETRACT_SCOOP");    
    case ROBOT_STATE::DUMP:
      return std::string("DONE");    
    default:
      return std::string("ERR");
  }
}
/**
 * Should be general autonomous function, but for now it will be auto digger
 */
void Robot::AutonomousPeriodic() {
  /**
   * Get important data about the state of the robot
   */
  //wpi::outs() << "Current State: " << ""
  double_t PositionActuator = this->GetLinearActuatorTurnValue();
  double_t CurrentDraw_LinearActuator = this->PDP.GetCurrent(PDPChannel_LinearActuator);
  double_t PositionFourbar = this->srx.GetSelectedSensorPosition();
  /**
   * State Machine
   */
  //Reset State
  if(this->CURRENT_ROBOT_STATE == RESET) {
    this->CURRENT_ROBOT_STATE = DIG_EXTEND_FOURBAR;
  }
  //Fourbar phase state
  else if(this->CURRENT_ROBOT_STATE==DIG_EXTEND_FOURBAR) {
    if(PositionFourbar > FOURBAR_EXTENSION_LIMIT) {
      this->CURRENT_ROBOT_STATE =  DIG_EXTEND_SCOOP;
    }
    else {
      this->CURRENT_ROBOT_STATE = DIG_EXTEND_FOURBAR;
    }
  }
  //Scoop phase state
  else if(this->CURRENT_ROBOT_STATE == DIG_EXTEND_SCOOP) {
    if(PositionActuator > SCOOP_EXTENSION_LIMIT) {
      this->CURRENT_ROBOT_STATE = DIG_RETRACT_SCOOP;
    }
    else {
      this->CURRENT_ROBOT_STATE = DIG_EXTEND_SCOOP;
    }
  }
  else if(this->CURRENT_ROBOT_STATE == DIG_RETRACT_SCOOP) {
    if(PositionActuator < SCOOP_RETRACTION_LIMIT) {
      this->CURRENT_ROBOT_STATE = DIG_RETRACT_FOURBAR;
    }
    else {
      this->CURRENT_ROBOT_STATE = DIG_RETRACT_SCOOP;
    }
  }
  else if(this->CURRENT_ROBOT_STATE == DIG_RETRACT_FOURBAR) {
    if(PositionFourbar < FOURBAR_RETRACTION_LIMIT) {
      this->CURRENT_ROBOT_STATE = DUMP;
    }
    else {
      this->CURRENT_ROBOT_STATE = DIG_RETRACT_FOURBAR;
    }
  }
  /**
   * State Behavior Machine
   */

  if(this->CURRENT_ROBOT_STATE == DIG_EXTEND_FOURBAR) {
    this->SRX_LINACT.Set(ControlMode::PercentOutput,LINACT_OUTPUT_ZERO);
    this->srx.Set(ControlMode::PercentOutput,FOURBAR_OUTPUT_HALF);
  }
  else if(this->CURRENT_ROBOT_STATE == DIG_EXTEND_SCOOP) {
    this->srx.Set(ControlMode::PercentOutput,FOURBAR_OUTPUT_ZERO);
    this->SRX_LINACT.Set(ControlMode::PercentOutput,LINACT_OUTPUT_HALF);
  }
  else if(this->CURRENT_ROBOT_STATE == DIG_RETRACT_SCOOP) {
    this->srx.Set(ControlMode::PercentOutput,FOUR)
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
double_t Robot::GetLinearActuatorTurnValue() {
  double_t CurrentAnologReading = this->AN_0_IN;

  double_t ScaledAnalogReading = (CurrentAnologReading- 0.0f)/ (5.0f - 0.0f)
   * (10000.0f - 0.0f) + 0.0f;
  return ScaledAnalogReading;
}
void Robot::InitializeTalonLinearActuator() {
  srx.ConfigFactoryDefault();
  SRX_LINACT.ConfigFactoryDefault();
  srx.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
  SRX_LINACT.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
  /**
     * Configure Talon SRX Output and Sesnor direction accordingly
     * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
     * Phase sensor to have positive increment when driving Talon Forward (Green LED)
     */
  srx.SetSensorPhase(false);
  srx.SetInverted(false);
  SRX_LINACT.SetSensorPhase(false);
  SRX_LINACT.SetInverted(false);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  srx.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  srx.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  SRX_LINACT.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0,10,10);
  SRX_LINACT.SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_,10,10);

  /* Set the peak and nominal outputs */
  srx.ConfigNominalOutputForward(0, 10);
  srx.ConfigNominalOutputReverse(0, 10);
  srx.ConfigPeakOutputForward(1, 10);
  srx.ConfigPeakOutputReverse(-1, 10);
  SRX_LINACT.ConfigNominalOutputForward(0, 10);
  SRX_LINACT.ConfigNominalOutputReverse(0, 10);
  SRX_LINACT.ConfigPeakOutputForward(1, 10);
  SRX_LINACT.ConfigPeakOutputReverse(-1, 10);
  /* Set Motion Magic gains in slot0 - see documentation */
  srx.SelectProfileSlot(0,0);
  srx.Config_kF(0,0.3,10);
  srx.Config_kP(0,0.1,10);
  srx.Config_kI(0.0,0.0,10);
  srx.Config_kD(0,0.0,10);

  SRX_LINACT.SelectProfileSlot(0,0);
  SRX_LINACT.Config_kF(0,0.3,10);
  SRX_LINACT.Config_kP(0,0.1,10);
  SRX_LINACT.Config_kI(0.0,0.0,10);
  SRX_LINACT.Config_kD(0,0.0,10);
  SRX_LINACT.ConfigMotionCruiseVelocity(1500,10);
  SRX_LINACT.ConfigMotionAcceleration(1500,10);

  srx.SetSelectedSensorPosition(0,0,10);
  SRX_LINACT.SetSelectedSensorPosition(0,0,10);

}




void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double Left_Y_Stick = -1.0 * joystick.GetY();
  double Right_Y_Stick = -1.0 * joystick.GetRawAxis(5);
  double_t LinActTurnValue = this->GetLinearActuatorTurnValue();
  if(fabs(Left_Y_Stick) < 0.10) {
    Left_Y_Stick = 0;
  }
  if(fabs(Right_Y_Stick) < 0.10) {
    Right_Y_Stick =0;
  }
  double MotorOuput = srx.GetMotorOutputPercent();

  std::stringstream sb;



  if(joystick.GetRawButton(2)) {
    srx.SetSelectedSensorPosition(0,0,10);
    SRX_LINACT.SetSelectedSensorPosition(0,0,10);
  }

  if(joystick.GetRawButton(1)) {
    wpi::outs() <<"Button 1 pressed, entering magic motion mode\n";
    double TargetPos = Right_Y_Stick + 4096 * 10.0;
    srx.Set(ControlMode::MotionMagic,TargetPos);
    SRX_LINACT.Set(ControlMode::MotionMagic,TargetPos);
  }


  
  else {

    srx.Set(ControlMode::PercentOutput,Left_Y_Stick);
    SRX_LINACT.Set(ControlMode::PercentOutput,Right_Y_Stick);
  }
  if(joystick.GetRawButtonPressed(6)) {
    ++_smoothing;
    if(_smoothing < 0) {
      _smoothing = 0;
    }
    wpi::outs() << "Smoothing is set to: " << _smoothing << "\n";
    srx.ConfigMotionSCurveStrength(_smoothing,0);
    SRX_LINACT.ConfigMotionSCurveStrength(_smoothing,0);
  }

  if(joystick.GetRawButtonPressed(5)) {
    --_smoothing;
    if(_smoothing<0) {
      _smoothing = 0;
    }
    wpi::outs() << "Smoothing is set to: " << _smoothing << "\n";
    srx.ConfigMotionSCurveStrength(_smoothing,0);
    SRX_LINACT.ConfigMotionSCurveStrength(_smoothing,0);

  }
    Instrum::Process(&srx,&sb);
    Instrum::Process(&SRX_LINACT,&sb);
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
