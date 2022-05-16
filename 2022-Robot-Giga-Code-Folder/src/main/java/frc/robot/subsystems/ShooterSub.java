// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  //Main Wheel Motor Controllers
  WPI_TalonFX shooterMainWheelLeft = new WPI_TalonFX(Constants.SHOOTER_MAIN_WHEEL_LEFT_ID);
  WPI_TalonFX shooterMainWheelRight = new WPI_TalonFX(Constants.SHOOTER_MAIN_WHEEL_RIGHT_ID);

  //Hood Wheels Motor Controller
  WPI_TalonFX shooterHoodWheels = new WPI_TalonFX(Constants.SHOOTER_HOOD_WHEEL_ID);

  //Hood Servos


  //Static Variables
  boolean shooterReady = false;
  boolean mainWheelReady = false;
  boolean hoodWheelsReady = false;
  boolean hoodServosReady = false;


  
  /** Creates a new ExampleSubsystem. */
  public ShooterSub() {
    //config main wheel left controller
    shooterMainWheelLeft.configFactoryDefault();
    shooterMainWheelLeft.setNeutralMode(NeutralMode.Coast);
    shooterMainWheelLeft.configClosedloopRamp(Constants.SHOOTER_RAMP_TIME);
    shooterMainWheelLeft.configOpenloopRamp(Constants.SHOOTER_RAMP_TIME);

    //config main wheel right controller
    shooterMainWheelRight.configFactoryDefault();
    shooterMainWheelRight.setNeutralMode(NeutralMode.Coast);

    //config hood wheels controller
    shooterHoodWheels.configFactoryDefault();
    shooterHoodWheels.setNeutralMode(NeutralMode.Coast);
    shooterHoodWheels.configClosedloopRamp(Constants.SHOOTER_RAMP_TIME);
    shooterHoodWheels.configOpenloopRamp(Constants.SHOOTER_RAMP_TIME);

    //config Main Wheel PID
    shooterMainWheelLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    shooterMainWheelLeft.configVelocityMeasurementWindow(1);
    shooterMainWheelLeft.config_kF(0, Constants.SHOOTER_MAIN_WHEEL_KF);
    shooterMainWheelLeft.config_kP(0, Constants.SHOOTER_MAIN_WHEEL_KP);

    //config hood wheels PID
    shooterHoodWheels.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    shooterHoodWheels.configVelocityMeasurementWindow(1);
    shooterHoodWheels.config_kF(0, Constants.SHOOTER_HOOD_WHEEL_KF);
    shooterHoodWheels.config_kP(0, Constants.SHOOTER_HOOD_WHEEL_KP);
  }

  public void idle() {
    shooterMainWheelLeft.set(ControlMode.Velocity, Constants.SHOOTER_MAINWHEEL_IDLE_VELOCITY);
    shooterHoodWheels.set(ControlMode.Velocity, Constants.SHOOTER_HOODWHEEL_IDLE_VELOCITY);
  }

  public void Shoot(double mainWheelVelocity, double hoodWheelsVelocity) {
    shooterMainWheelLeft.set(ControlMode.Velocity, mainWheelVelocity);
    shooterHoodWheels.set(ControlMode.Velocity, hoodWheelsVelocity);
  }

  public void stop() {
    shooterMainWheelLeft.set(TalonFXControlMode.PercentOutput, 0);
    shooterHoodWheels.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //is main wheel ready?
    if (shooterMainWheelLeft.getSelectedSensorVelocity() < (shooterMainWheelLeft.getClosedLoopTarget()+(shooterMainWheelLeft.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
        &&
        shooterMainWheelLeft.getSelectedSensorVelocity() > (shooterMainWheelLeft.getClosedLoopTarget()-(shooterMainWheelLeft.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
    ) mainWheelReady = true;
    else mainWheelReady = false;

    //is hood wheel ready?
    if (shooterHoodWheels.getSelectedSensorVelocity() < (shooterHoodWheels.getClosedLoopTarget()+(shooterHoodWheels.getClosedLoopTarget()*Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR))
        &&
        shooterHoodWheels.getSelectedSensorVelocity() > (shooterHoodWheels.getClosedLoopTarget()-(shooterHoodWheels.getClosedLoopTarget()*Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR))
    ) hoodWheelsReady = true;
    else hoodWheelsReady = false;

    //are servos ready?

    //is shooter ready?
    if (mainWheelReady
      &&hoodServosReady
    //  &&hoodServosReady
    ) shooterReady = true;
    else shooterReady = false;

    SmartDashboard.putBoolean("ShooterReady", shooterReady);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
