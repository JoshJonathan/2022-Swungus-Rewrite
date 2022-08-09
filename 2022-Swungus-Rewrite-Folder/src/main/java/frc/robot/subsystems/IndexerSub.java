// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSub extends SubsystemBase {
  //Motors
  WPI_TalonSRX indexerFront = new WPI_TalonSRX(Constants.INDEXER_FRONT_ID);
  WPI_TalonSRX indexerRear = new WPI_TalonSRX(Constants.INDEXER_REAR_ID); 

  /** Creates a new IndexerSub. */
  public IndexerSub() {
    //Configure Motors
    indexerFront.configFactoryDefault();
    indexerFront.configOpenloopRamp(Constants.INDEXER_RAMP_TIME);
    indexerFront.configVoltageCompSaturation(Constants.INDEXER_NOMINAL_VOLTAGE);
    indexerFront.configVoltageMeasurementFilter(Constants.INDEXER_VOLTAGE_FILTER_WINDOW_SAMPLES);
    indexerFront.enableVoltageCompensation(true);
    indexerFront.setInverted(true);
    indexerRear.setInverted(InvertType.FollowMaster);
    indexerFront.setNeutralMode(NeutralMode.Coast);
    indexerFront.setNeutralMode(NeutralMode.Coast);
    indexerRear.follow(indexerFront);
  }

  //Index
  public void index(double percentOutput) {
    indexerFront.set(ControlMode.PercentOutput, percentOutput);
  }

  //Shoot
  public void shoot() {
    //determine if we are at setpoints
    //SmartDashboard.putNumber("mainWheelSetpoint", ShooterSub.mainWheelSetpoint);
    //SmartDashboard.putNumber("mainWheelValue", ShooterSub.mainWheelValue);
    //SmartDashboard.putNumber("hoodWheelsSetpoint", ShooterSub.hoodWheelsSetpoint);
    //SmartDashboard.putNumber("hoodWheelsValue", ShooterSub.hoodWheelsValue);
    //SmartDashboard.putNumber("kickerWheelSetpoint", ShooterSub.kickerWheelSetpoint);
    //SmartDashboard.putNumber("kickerWheelValue", ShooterSub.kickerWheelValue);
    if(ShooterSub.mainWheelSetpoint != Constants.SHOOTER_MAIN_WHEEL_IDLE_VELOCITY &&
       readyToShoot(ShooterSub.mainWheelSetpoint, ShooterSub.mainWheelValue, Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR) //&&
       //readyToShoot(ShooterSub.hoodWheelsSetpoint, ShooterSub.hoodWheelsValue, Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR) &&
       //readyToShoot(ShooterSub.kickerWheelSetpoint, ShooterSub.kickerWheelValue, Constants.SHOOTER_KICKER_WHEEL_ALLOWABLE_ERROR)
    ) index(Constants.INDEXER_OUTPUT);
    else index(0);
  }

  //Check if shooter is ready
  public boolean readyToShoot(double setpoint, double value, double allowableError) {
    SmartDashboard.putNumber("value", value);
    SmartDashboard.putNumber("min", setpoint-(allowableError*setpoint));
    SmartDashboard.putNumber("max", (allowableError*setpoint)+setpoint);
    if (value > (setpoint-(allowableError*setpoint)) && value < ((allowableError*setpoint)+setpoint)) return true;
    else return false;
  }
}