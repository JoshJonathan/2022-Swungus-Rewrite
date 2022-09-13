// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  //Solenoid  //Motor  
    private WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR_ID);
    private DigitalInput limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_DIO);
    private double desiredSpeed = 0;
    private double encoderOffset = 0;

  /** Creates a new Elevator. */
  public ElevatorSub() {
    //Config Motor
     
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("ticks", elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("adjustedticks", elevatorMotor.getSelectedSensorPosition()-encoderOffset);
    elevatorMotor.set(ControlMode.PercentOutput, desiredSpeed*Constants.ELEVATOR_SPEED);
    if(getBottomLimit()){
      if(desiredSpeed<0)
        desiredSpeed = 0;
      elevatorMotor.setNeutralMode(NeutralMode.Coast);
      encoderOffset = elevatorMotor.getSelectedSensorPosition();
      return;
    }
    if(getTopLimit()){
      if(desiredSpeed>0)
        desiredSpeed = 0;
      elevatorMotor.setNeutralMode(NeutralMode.Brake);
      return;
    }
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    
  }

  public void upButton(){
    updateSpeed(1);
  }
  public void downButton(){
    updateSpeed(-1);
  }

  public void updateSpeed(double speed){
    desiredSpeed = speed;
  }


  public void brakeElevator(){
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    desiredSpeed = 0;
  }


  public boolean getBottomLimit(){
    //return false;
    return limitSwitch.get();
  }

  public boolean getTopLimit(){
    return elevatorMotor.getSelectedSensorPosition()-encoderOffset>Constants.ELEVATOR_ENCODER_MAX;
  }
}
