// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
  
}
