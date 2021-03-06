// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>

#include <iostream>
#include <PhysicsSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#define ANALOG_CHANNELS (4)
#define PDPChannel_LinearActuator (0)
//Fourbar dig motor positions

//Fourbar position for digging
#define FOURBAR_DIG_EXTENSION_LIMIT (3700.0)

//Fourbar position for reset
#define FOURBAR_DIG_RETRACTION_LIMIT (500.0)

//Scoop dig motor positions

//Scoop position for collecting material
#define SCOOP_EXTENSION_LIMIT (4700.0)
//Scoop position for dumping material
#define SCOOP_RETRACTION_LIMIT (520.0)
//Fourbar dump motor positions
#define FOURBAR_DUMP_POSITION (1200.0)
//Scoop dump motor positions
#define SCOOP_DUMP_POSITION (600)
//Fourbar motor output values

//Zero motor output
#define FOURBAR_ZERO_OUTPUT (0.0)
//Will move fourbar lower to the ground
#define FOURBAR_EXTEND_OUTPUT (0.50)
//Will move fourbar closes to starting/dumping position
#define FOURBAR_RETRACT_OUTPUT (-0.50)
//Scoop motor output values

//Zero output for scoop motor
#define SCOOP_ZERO_OUTPUT (0.0)
//Will move linear actuator out/positive direction
#define SCOOP_EXTEND_OUTPUT (-0.90)
//Will move linear actuator in/negative direction
#define SCOOP_RETRACT_OUTPUT (0.90)

void Robot::RobotInit() {
  this->InitializeAnalogInput(0,4);
  
  this->InitializeTalonLinearActuator();
}
void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonSRX(SRX_FOURBAR,.75,3400,false);
  PhysicsSim::GetInstance().AddTalonSRX(SRX_LINACT,.75,3400,false);
}

void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();
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
  
  /**
   * Positional Information
   */
  double_t PositionActuator = this->GetLinearActuatorTurnValue();
  double_t PositionFourbar = this->SRX_FOURBAR.GetSelectedSensorPosition();
  double_t PositionThresholdValue = this->GetPositionThresholdValue(this->AutoCycleCount,this->CURRENT_ROBOT_STATE);
  /**
   * Current Information
   */
  double_t LinearActuatorCurrent = this->SRX_LINACT.GetOutputCurrent();
  double_t CurrentThresholdValue = this->GetCurrentThresholdValue(this->CURRENT_ROBOT_STATE);
    //Update SmartDashbord

  frc::SmartDashboard::PutString("Current Robot State",this->GetStateAsString(this->CURRENT_ROBOT_STATE));
  frc::SmartDashboard::PutBoolean("Button 7 Pressed?",(bool)this->joystick.GetRawButton(7));

  frc::SmartDashboard::PutNumber("Linear Actuator Current",LinearActuatorCurrent);
  frc::SmartDashboard::PutNumber("Linear Actuactor Motor Output",this->SRX_LINACT.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("Linear Actuator Position",PositionActuator);
  frc::SmartDashboard::PutNumber("Linear Actuator Potentiometer Reading (V)",this->GetPotentiometerReading());

  frc::SmartDashboard::PutNumber("Fourbar Position",PositionFourbar);
  frc::SmartDashboard::PutNumber("Fourbar Motor Output",this->SRX_FOURBAR.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("Fourbar Current",this->GetAvgFourbarCurrent());


  frc::SmartDashboard::PutNumber("Position Threshold",PositionThresholdValue);
  frc::SmartDashboard::PutBoolean("Current Exceed Safe Value? ", LinearActuatorCurrent >= CurrentThresholdValue);

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
  this->RuntimeLog.RunStart = std::time(NULL);
  this->RuntimeLog.StartVoltage = this->PDP.GetVoltage();
  Robot::RunInformation::StateLog InitialStateLog;
  InitialStateLog.State = this->CURRENT_ROBOT_STATE;
  InitialStateLog.StateStart = this->RuntimeLog.RunStart;
  InitialStateLog.StartVoltage= this->RuntimeLog.StartVoltage;
  this->RuntimeLog.StateTimes.emplace(InitialStateLog);
  SRX_FOURBAR.SetSelectedSensorPosition(0,0,10);
  SRX_LINACT.SetSelectedSensorPosition(0,0,10);
}
size_t Robot::GetAvgCurrentBufferIndex() {
  return this->AvgCurrentBufferIndex;
}

double_t Robot::GetAvgFourbarCurrent() {
  double_t val = 0.0;
  for(size_t index =0; index < this->AvgFourbarCurrentBuffer.size();index++) {
    val+=this->AvgFourbarCurrentBuffer.at(index);
  }
  val = val / this->AvgFourbarCurrentBuffer.size();
  return val;
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
    case ROBOT_STATE::DUMP_SCOOP:
      return std::string("DUMP_SCOOP");
    case ROBOT_STATE::HOLD:
      return std::string("HOLD");
    default:
      return std::string("ERR");
  }
}
double_t Robot::GetPositionThresholdValue(size_t CycleCount, ROBOT_STATE CurrentState) {
  if(CycleCount == 0) {
      if (CurrentState == ROBOT_STATE::DIG_EXTEND_FOURBAR) {
        return FOURBAR_DIG_EXTENSION_LIMIT;
      }
      else if(CurrentState== ROBOT_STATE::RESET) {
        return 50.0;
      }
      else if(CurrentState == ROBOT_STATE::DIG_EXTEND_SCOOP)
      {
        return SCOOP_EXTENSION_LIMIT;
      }
      else if(CurrentState == ROBOT_STATE::DUMP_SCOOP) {
        return SCOOP_RETRACTION_LIMIT;
      }
      else if(CurrentState == ROBOT_STATE::DIG_RETRACT_FOURBAR) {
        return FOURBAR_DIG_RETRACTION_LIMIT;
      }    
  }
}
/**
 * Return the correct value for current threshold depending on the current digging state
 */
double_t Robot::GetCurrentThresholdValue(ROBOT_STATE CurrentState) {
  if(CurrentState == ROBOT_STATE::DIG_EXTEND_SCOOP) {
    return 0.7;
  }
  else if(CurrentState == ROBOT_STATE::DIG_EXTEND_FOURBAR) {
    return 0.25;
  }
}
/**
 * Returns a value from 0.0V to 5.0V after conversion from 12-bit ADC with 
 */
double_t Robot::GetPotentiometerReading() {
  return this->VEC_ANALOG_IN.at(0).GetAverageVoltage();
}
void Robot::DisplayRobotState() {
}
int64_t Robot::GetPotentiometerReadingInTurns() {
  int64_t val = (int64_t) (this->VEC_ANALOG_IN.at(0).GetAverageVoltage() * 2.0);
  return val;

}
  
void Robot::UpdateCurrentBuffer() {
  this->AvgFourbarCurrentBuffer.at(this->AvgCurrentBufferIndex) = this->SRX_FOURBAR.GetOutputCurrent();
  this->AvgCurrentBufferIndex++;
  if(this->AvgCurrentBufferIndex ==this->AvgFourbarCurrentBuffer.size()) {
    this->AvgCurrentBufferIndex= 0;
  }
}
void Robot::UpdateLog() {
  /**
    * Positional Information
    */
  double_t PositionActuator = this->GetLinearActuatorTurnValue();
  double_t PositionFourbar = this->SRX_FOURBAR.GetSelectedSensorPosition();
  double_t PositionThresholdValue = this->GetPositionThresholdValue(this->AutoCycleCount,this->CURRENT_ROBOT_STATE);
  /**
   * Current Information
   */
  double_t LinearActuatorCurrent = this->SRX_LINACT.GetOutputCurrent();
  double_t CurrentThresholdValue = this->GetCurrentThresholdValue(this->CURRENT_ROBOT_STATE);
  double_t FourbarCurrent = this->GetAvgFourbarCurrent();
  /**
   * Log info about the current state *transition*, not general state
   */
  if(this->CURRENT_ROBOT_STATE !=this->NEXT_ROBOT_STATE) {
    
  
    /**
     * Compute end time for state
     */
    std::time_t EndingTimestamp = std::time(NULL) - this->RuntimeLog.RunStart;
    this->RuntimeLog.StateTimes.top().StateEnd = EndingTimestamp;
    this->RuntimeLog.StateTimes.top().State = this->CURRENT_ROBOT_STATE;
    this->RuntimeLog.StateTimes.top().EndVoltage = this->PDP.GetVoltage();
    Robot::RunInformation::StateLog CurrentStateLog;
    CurrentStateLog.AverageCurrent = this->GetAvgFourbarCurrent();


    if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_FOURBAR || ROBOT_STATE::DIG_RETRACT_FOURBAR) {
      CurrentStateLog.FeedbackPosition = PositionFourbar;
      CurrentStateLog.AverageCurrent = FourbarCurrent;
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_SCOOP || ROBOT_STATE::DUMP_SCOOP) {
      CurrentStateLog.FeedbackPosition = PositionActuator;
      CurrentStateLog.AverageCurrent = LinearActuatorCurrent;
    }
    CurrentStateLog.TargetPosition = PositionThresholdValue;
    CurrentStateLog.StateStart = EndingTimestamp;
    this->RuntimeLog.StateTimes.emplace(CurrentStateLog);
  }
  else {
    if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_FOURBAR || ROBOT_STATE::DIG_RETRACT_FOURBAR) {
      this->RuntimeLog.StateTimes.top().AverageCurrent = FourbarCurrent;
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_SCOOP || ROBOT_STATE::DUMP_SCOOP) {
      if(this->RuntimeLog.StateTimes.top().MaxCurrent < LinearActuatorCurrent) {
        this->RuntimeLog.StateTimes.top().MaxCurrent = LinearActuatorCurrent;
      }
      if(this->RuntimeLog.StateTimes.top().MinCurrent > LinearActuatorCurrent) {
        this->RuntimeLog.StateTimes.top().MinCurrent = LinearActuatorCurrent;
      }
    }
  }
}
void Robot::RunInformation::PrintRunInfo() {
  while(!this->StateTimes.empty()) {
    wpi::outs() << this->StateTimes.top().Stringify();
    this->StateTimes.pop();

  }

}
std::string Robot::RunInformation::StateLog::Stringify() {

  std::ostringstream ostream;
  std::time_t TotalTime = (this->StateEnd - this->StateStart);
  double_t Error = (this->FeedbackPosition - this->TargetPosition) / this->TargetPosition * 100.0;

  ostream << "State[" << Robot::GetStateAsString(this->State) << "] ";
  ostream << "Time[" << TotalTime << "] "; 
  ostream << "% Error[" << Error << "%] ";
  ostream << "Avg Current[" <<this->AverageCurrent << " A]\n";
  std::string result = ostream.str();
  
  return result;
}
/**
 * Should be general autonomous function, but for now it will be auto digger
 * From starting position (0), fourbar motor needs to slowly (ControlMode::PercentOutput = 0.15)
 * move until anywhere from absolute position -200 <-> -400 then slow down to (ControlMode::PercentOutput = 0.05)
 * In order to hold current position while in extended mode, motor needs to be set to (ControlMode::PercentOutput = -0.15)
 * 
 * Estimated Full range until scoop actuation: 0 -> -7000
 */
void Robot::AutonomousPeriodic() {
  if(this->AutoCycleCount <3) {
    this->UpdateCurrentBuffer();
    /**
     * Positional Information
     */
    double_t PositionActuator = this->GetLinearActuatorTurnValue();
    double_t PositionFourbar = this->SRX_FOURBAR.GetSelectedSensorPosition();
    double_t PositionThresholdValue = this->GetPositionThresholdValue(this->AutoCycleCount,this->CURRENT_ROBOT_STATE);
    /**
     * Current Information
     */
    double_t LinearActuatorCurrent = this->SRX_LINACT.GetOutputCurrent();
    double_t CurrentThresholdValue = this->GetCurrentThresholdValue(this->CURRENT_ROBOT_STATE);
    double_t FourbarCurrent = this->GetAvgFourbarCurrent();


    /**
     * State Machine
     */
    //Reset state, go to DIG_EXTEND_FOURBAR
    if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::RESET) {
      if(PositionFourbar > 50.0) {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::RESET;
      }
      else {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_EXTEND_FOURBAR;
      }
    }
    //Extend fourbar state
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_FOURBAR) {

      if(PositionFourbar < PositionThresholdValue && FourbarCurrent <= CurrentThresholdValue) {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_EXTEND_FOURBAR;
      }
      else {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_EXTEND_SCOOP;
      }
    }
    //Retract scoop state
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_SCOOP) {
      if(PositionActuator < PositionThresholdValue) {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_EXTEND_SCOOP;
      }
      else {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_RETRACT_FOURBAR;
      }
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_RETRACT_FOURBAR) {
      if(PositionFourbar > PositionThresholdValue) {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DIG_RETRACT_FOURBAR;
      }
      else {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DUMP_SCOOP;
      }
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DUMP_SCOOP) {
      if(PositionActuator > PositionThresholdValue) {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::DUMP_SCOOP;
      }
      else {
        this->NEXT_ROBOT_STATE = ROBOT_STATE::RESET;
      }
    }
    else {
      this->NEXT_ROBOT_STATE = ROBOT_STATE::RESET;
    }
    this->UpdateLog();
    if(this->NEXT_ROBOT_STATE == ROBOT_STATE::RESET) {
      this->AutoCycleCount++;
      this->RuntimeLog.AutoDigCycles= this->AutoCycleCount;
    }
    //Update current robot state
    this->CURRENT_ROBOT_STATE = this->NEXT_ROBOT_STATE;
    
    /**
     * State Behavior Machine
     */
    //Scop motor output
    double_t MotorOutputScoop= 0.0;
    //Fourbar motor output
    double_t MotorOutputFourbar = 0.0;
      
    
    if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::RESET) {
      MotorOutputScoop = SCOOP_ZERO_OUTPUT;
      MotorOutputFourbar = FOURBAR_RETRACT_OUTPUT;
    }

    else if(this->CURRENT_ROBOT_STATE ==ROBOT_STATE::HOLD) {
      MotorOutputScoop = SCOOP_ZERO_OUTPUT;
      MotorOutputFourbar = FOURBAR_ZERO_OUTPUT;
    }

    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_FOURBAR) {
      MotorOutputScoop = SCOOP_ZERO_OUTPUT;
      MotorOutputFourbar = FOURBAR_EXTEND_OUTPUT;
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DIG_EXTEND_SCOOP) {
      MotorOutputScoop =  SCOOP_EXTEND_OUTPUT;
      MotorOutputFourbar = FOURBAR_ZERO_OUTPUT;

    }
    else if(this->CURRENT_ROBOT_STATE ==ROBOT_STATE::DIG_RETRACT_FOURBAR) {
      MotorOutputScoop =  SCOOP_ZERO_OUTPUT;
      MotorOutputFourbar = FOURBAR_RETRACT_OUTPUT;
    }
    else if(this->CURRENT_ROBOT_STATE == ROBOT_STATE::DUMP_SCOOP) {
      MotorOutputScoop = SCOOP_RETRACT_OUTPUT;
      MotorOutputFourbar = FOURBAR_ZERO_OUTPUT;
    }
    else {
      MotorOutputScoop =  SCOOP_ZERO_OUTPUT;
      MotorOutputFourbar = FOURBAR_ZERO_OUTPUT;
    }
    this->SRX_LINACT.Set(ControlMode::PercentOutput,MotorOutputScoop);
    this->SRX_FOURBAR.Set(ControlMode::PercentOutput,MotorOutputFourbar);
  }
  if(this->AutoCycleCount==1) {
    this->RuntimeLog.PrintRunInfo();
    this->AutoCycleCount++;
  }
 
}
void Robot::InitializeAnalogInput(uint64_t channel, uint64_t bits) {
  if(channel > 3) {
    perror("in initializeAnalogInput");
  }
  if(this->VEC_ANALOG_IN.size()==0 || this->VEC_ANALOG_IN.size() < channel) {
    this->VEC_ANALOG_IN.push_back(frc::AnalogInput(channel))  ;
  }
  this->VEC_ANALOG_IN[channel].SetOversampleBits(bits);
}
double_t Robot::ReadAnalogIn(uint64_t channel) {
  if(channel > 3) {
    perror("in initializeAnalogInput");
  }
  double_t AVG_Voltage = this->VEC_ANALOG_IN[channel].GetAverageVoltage(); 

  return AVG_Voltage;

}
void Robot::ReadPDPChannel(uint64_t channel) {
  if(channel > 15) {
    perror("In ReadPDPChannel");
  }
  double_t Reading =this->PDP.GetCurrent(channel); 
  frc::SmartDashboard::PutNumber(std::string("Current Channel " + channel),Reading);
}
void Robot::ReadAnalogChannel0Callback() {
  this->AN_0_IN = this->VEC_ANALOG_IN[0].GetAverageVoltage();
}

double_t Robot::GetLinearActuatorTurnValue() {
  double_t CurrentAnologReading = this->VEC_ANALOG_IN[0].GetAverageVoltage();

  double_t ScaledAnalogReading = (CurrentAnologReading- 0.0f)/ (5.0f - 0.0f)
   * (10000.0f - 0.0f) + 0.0f;
  return ScaledAnalogReading;
}

void Robot::InitializeTalonLinearActuator() {
  SRX_FOURBAR.ConfigFactoryDefault();
  SRX_LINACT.ConfigFactoryDefault();
  SRX_FOURBAR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
  SRX_LINACT.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,10);
  /**
     * Configure Talon SRX Output and Sesnor direction accordingly
     * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
     * Phase sensor to have positive increment when driving Talon Forward (Green LED)
     */
  SRX_FOURBAR.SetSensorPhase(false);
  SRX_FOURBAR.SetInverted(false);
  SRX_LINACT.SetSensorPhase(false);
  SRX_LINACT.SetInverted(false);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  SRX_FOURBAR.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  SRX_FOURBAR.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  SRX_LINACT.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0,10,10);
  SRX_LINACT.SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_,10,10);

  /* Set the peak and nominal outputs */
  SRX_FOURBAR.ConfigNominalOutputForward(0, 10);
  SRX_FOURBAR.ConfigNominalOutputReverse(0, 10);
  SRX_FOURBAR.ConfigPeakOutputForward(1, 10);
  SRX_FOURBAR.ConfigPeakOutputReverse(-1, 10);
  SRX_LINACT.ConfigNominalOutputForward(0, 10);
  SRX_LINACT.ConfigNominalOutputReverse(0, 10);
  SRX_LINACT.ConfigPeakOutputForward(1, 10);
  SRX_LINACT.ConfigPeakOutputReverse(-1, 10);
  /* Set Motion Magic gains in slot0 - see documentation */
  SRX_FOURBAR.SelectProfileSlot(0,0);
  SRX_FOURBAR.Config_kF(0,0.3,10);
  SRX_FOURBAR.Config_kP(0,0.1,10);
  SRX_FOURBAR.Config_kI(0.0,0.0,10);
  SRX_FOURBAR.Config_kD(0,0.0,10);

  SRX_LINACT.SelectProfileSlot(0,0);
  SRX_LINACT.Config_kF(0,0.3,10);
  SRX_LINACT.Config_kP(0,0.1,10);
  SRX_LINACT.Config_kI(0.0,0.0,10);
  SRX_LINACT.Config_kD(0,0.0,10);
  SRX_LINACT.ConfigMotionCruiseVelocity(1500,10);
  SRX_LINACT.ConfigMotionAcceleration(1500,10);

  SRX_FOURBAR.SetSelectedSensorPosition(0,0,10);
  SRX_LINACT.SetSelectedSensorPosition(0,0,10);

}




void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  if(AutoPilot) {
    AutoPilot = false;
  }
  double Left_Y_Stick = -1.0 * joystick.GetY();
  double Right_Y_Stick = -1.0 * joystick.GetRawAxis(3);
  double_t LinActTurnValue = this->GetLinearActuatorTurnValue();
  

  if(fabs(Left_Y_Stick) < 0.10) {
    Left_Y_Stick = 0;
  }
  if(fabs(Right_Y_Stick) < 0.10) {
    Right_Y_Stick =0;
  }
  double MotorOuput = SRX_FOURBAR.GetMotorOutputPercent();

  std::stringstream sb;
  if(joystick.GetRawButton(8)) {
    this->DisplayRobotState();
  }
  if(joystick.GetRawButton(2)) {
    SRX_FOURBAR.SetSelectedSensorPosition(0,0,10);
    SRX_LINACT.SetSelectedSensorPosition(0,0,10);
  }

  else if(joystick.GetRawButton(3)) {
    SRX_FOURBAR.Set(ControlMode::PercentOutput,1.0);
  }
  else if(joystick.GetRawButton(1)) {
    SRX_FOURBAR.Set(ControlMode::PercentOutput,-1.0);
  }
  
  else {
    double New_Left_Y = 0;
    if (joystick.GetRawButtonPressed(3)){
      New_Left_Y = Left_Y_Stick/3;
    }
    else if(joystick.GetRawButtonPressed(4)){
      New_Left_Y = Left_Y_Stick;
    }
    else {
      New_Left_Y = Left_Y_Stick * .9;
    }
    SRX_FOURBAR.Set(ControlMode::PercentOutput,New_Left_Y);
    if(Right_Y_Stick > 0) {
      SRX_LINACT.Set(ControlMode::PercentOutput,1);
    }
    SRX_LINACT.Set(ControlMode::PercentOutput,Right_Y_Stick);
  }
  if(joystick.GetRawButtonPressed(6)) {
    ++_smoothing;
    if(_smoothing < 0) {
      _smoothing = 0;
    }
    wpi::outs() << "Smoothing is set to: " << _smoothing << "\n";
    SRX_FOURBAR.ConfigMotionSCurveStrength(_smoothing,0);
    SRX_LINACT.ConfigMotionSCurveStrength(_smoothing,0);
  }

  if(joystick.GetRawButtonPressed(5)) {
    --_smoothing;
    if(_smoothing<0) {
      _smoothing = 0;
    }
    wpi::outs() << "Smoothing is set to: " << _smoothing << "\n";
    SRX_FOURBAR.ConfigMotionSCurveStrength(_smoothing,0);
    SRX_LINACT.ConfigMotionSCurveStrength(_smoothing,0);

  }
    Instrum::Process(&SRX_FOURBAR,&sb);
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
