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
  
  /** Creates a new Elevator. */
  public ElevatorSub() {
    //Config Motor
     
  }

  private double desiredPosition;
  private double offset = elevatorMotor.getSelectedSensorPosition();
  private final double threshold = 100;
  private double powerDampened = 0;
  private double dampenPower = 0.05;

  @Override
  public void periodic(){
    double powerOut = 0;

    if(limitSwitch.get()){
      offset = elevatorMotor.getSelectedSensorPosition();
    }

    if(Math.abs(desiredPosition-elevatorPosition())>threshold){
      elevatorMotor.setNeutralMode(NeutralMode.Coast);
     powerOut = (desiredPosition-elevatorPosition())/Constants.ELEVATOR_ENCODER_MAX*Constants.ELEVATOR_SPEED;
     if(powerOut>0) powerOut+=Constants.ELEVATOR_MIN_SPEED;
     else powerOut-=Constants.ELEVATOR_MIN_SPEED;
    }
    else{
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

  //  if(Math.abs(powerOut)>Math.abs(powerDampened)){
      powerDampened += (powerOut- powerDampened)*dampenPower;
  //  else 
  //    powerDampened = powerOut;
    
    elevatorMotor.set(ControlMode.PercentOutput,powerDampened);

    SmartDashboard.putNumber("output",powerOut);
    SmartDashboard.putNumber("output_dampened",powerDampened);
    SmartDashboard.putNumber("rawEncoder",elevatorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("encoderAdjusted",elevatorPosition());
    SmartDashboard.putNumber("elevator %",Math.floor(elevatorPosition()/Constants.ELEVATOR_ENCODER_MAX*100));


  }

  public double elevatorPosition(){
    return elevatorMotor.getSelectedSensorPosition() - offset;
  }
public void setTargetPercent(double target){
desiredPosition = Constants.ELEVATOR_ENCODER_MAX*target;
}

  public void upButton(){
setTargetPercent(1);
  }
  public void downButton(){
    setTargetPercent(0);
  }

  public void halfButton(){
    setTargetPercent(0.5);
  }

  public void brakeElevator(){
    desiredPosition = elevatorPosition();
  }

}
