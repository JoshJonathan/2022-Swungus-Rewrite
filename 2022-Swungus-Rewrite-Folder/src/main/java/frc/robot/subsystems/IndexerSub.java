// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSub extends SubsystemBase {
  //Motors
  WPI_TalonSRX indexerFront = new WPI_TalonSRX(Constants.INDEXER_FRONT_ID);
  WPI_TalonSRX indexerRear = new WPI_TalonSRX(Constants.INDEXER_REAR_ID);

  //Timer
  Timer timer = new Timer();

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

    timer.reset();
    timer.start();
  }

  //Index
  public void index(double percentOutput) {
    indexerFront.set(ControlMode.PercentOutput, percentOutput);
  }

  //Shoot
  public void shoot() {
    //determine if we are at setpoints
    if(ShooterSub.mainWheelSetpoint != Constants.SHOOTER_MAIN_WHEEL_IDLE_VELOCITY &&
       readyToShoot(ShooterSub.mainWheelSetpoint, ShooterSub.mainWheelValue, Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR) &&
       readyToShoot(ShooterSub.hoodWheelsSetpoint, ShooterSub.hoodWheelsValue, Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR) &&
       readyToShoot(ShooterSub.kickerWheelSetpoint, ShooterSub.kickerWheelValue, Constants.SHOOTER_KICKER_WHEEL_ALLOWABLE_ERROR) &&
       readyToShoot(ShooterSub.servoSetpoint, ShooterSub.servoValue, Constants.SHOOTER_SERVOS_ALLOWABLE_ERROR) &&
       (readyToShoot(0, LimelightSub.x, Constants.DRIVETRAIN_ALLOWABLE_ERROR) || ShooterSub.mainWheelSetpoint == Constants.SHOOTER_MAIN_WHEEL_FENDERSHOT_VELOCITY)	
       ) {
      index(Constants.INDEXER_OUTPUT);
      timer.reset();
      timer.start();
    }
    else if (timer.get() > Constants.INDEXER_TIMER_DELAY) index(0);
  }

  //Check if shooter is ready
  public boolean readyToShoot(double setpoint, double value, double allowableError) {
    if (value > (setpoint-(allowableError*setpoint)) &&
        value < (setpoint+(allowableError*setpoint))
        ) return true;
    else return false;
  }
}