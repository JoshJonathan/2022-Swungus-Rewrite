// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  //Motor Controllers
    //MainWheel
    WPI_TalonFX shooterMainWheelLeft = new WPI_TalonFX(Constants.SHOOTER_MAIN_WHEEL_LEFT_ID);
    WPI_TalonFX shooterMainWheelRight = new WPI_TalonFX(Constants.SHOOTER_MAIN_WHEEL_RIGHT_ID);
    //HoodWheels
    WPI_TalonFX shooterHoodWheels = new WPI_TalonFX(Constants.SHOOTER_HOOD_WHEELS_ID);
    //KickerWheel
    WPI_TalonFX shooterKickerWheel = new WPI_TalonFX(Constants.SHOOTER_KICKER_WHEEL_ID);
  //Servos
    //HoodServos

    //Characterization Table
      double[][] characterizationTable = {
        /*tY*/{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /*mW*/{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /*hW*/{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
        /*kW*/{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /*sP*/{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      };
  
  /** Creates a new ExampleSubsystem. */
  public ShooterSub() {
    //Controllers
      //mainWheelLeft
      shooterMainWheelLeft.configFactoryDefault();
      shooterMainWheelLeft.setNeutralMode(NeutralMode.Coast);
      shooterMainWheelLeft.setInverted(TalonFXInvertType.Clockwise);
      shooterMainWheelLeft.configClosedloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterMainWheelLeft.configOpenloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      //mainWheelRight
      shooterMainWheelRight.configFactoryDefault();
      shooterMainWheelRight.follow(shooterMainWheelLeft);
      shooterMainWheelLeft.setInverted(TalonFXInvertType.OpposeMaster);
      shooterMainWheelRight.setNeutralMode(NeutralMode.Coast);
      //hoodWheels
      shooterHoodWheels.configFactoryDefault();
      shooterHoodWheels.setNeutralMode(NeutralMode.Coast);
      shooterHoodWheels.setInverted(TalonFXInvertType.Clockwise);
      shooterHoodWheels.configClosedloopRamp(Constants.SHOOTER_HOOD_WHEELS_RAMP_TIME);
      shooterHoodWheels.configOpenloopRamp(Constants.SHOOTER_HOOD_WHEELS_RAMP_TIME);
      //kickerWheel
      shooterKickerWheel.configFactoryDefault();
      shooterKickerWheel.setNeutralMode(NeutralMode.Coast);
      shooterKickerWheel.setInverted(TalonFXInvertType.CounterClockwise);
      shooterKickerWheel.configClosedloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterKickerWheel.configOpenloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
    //PID
      //mainWheel PID
      shooterMainWheelLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
      shooterMainWheelLeft.configVelocityMeasurementWindow(Constants.SHOOTER_VELOCITY_MEASUREMENT_WINDOW);
      shooterMainWheelLeft.config_kF(0, Constants.SHOOTER_MAIN_WHEEL_KF);
      shooterMainWheelLeft.config_kP(0, Constants.SHOOTER_MAIN_WHEEL_KP);
      //hoodWheels PID
      shooterHoodWheels.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
      shooterHoodWheels.configVelocityMeasurementWindow(Constants.SHOOTER_VELOCITY_MEASUREMENT_WINDOW);
      shooterHoodWheels.config_kF(0, Constants.SHOOTER_HOOD_WHEELS_KF);
      shooterHoodWheels.config_kP(0, Constants.SHOOTER_HOOD_WHEELS_KP);
      //kickerWheel PID
      shooterKickerWheel.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
      shooterKickerWheel.configVelocityMeasurementWindow(Constants.SHOOTER_VELOCITY_MEASUREMENT_WINDOW);
      shooterKickerWheel.config_kF(0, Constants.SHOOTER_KICKER_WHEEL_KF);
      shooterKickerWheel.config_kP(0, Constants.SHOOTER_KICKER_WHEEL_KP);
    //DashBoards
  }
//State computers
    
  //Shooter ready to shoot
//  public void computeShooterReadyToShoot() {
    //compute subsubsystems
      //mainWheel
//      computeMainWheelReadyToShoot();
      //hoodWheels
        /*
      computeHoodWheelsReadyToShoot()
        */
      //kickerWheels
        /*
      computeKickerWheelReadyToShoot();
        */
      //servos
    //compute subsystem
//    if (mainWheelReadyToShoot
      /*
     && hoodWheelsReadyToShoot
      */
      /*
     && kickerWheelsReadyToShoot
     */
    /*
     &&hoodServosReady
     */
//    ) readyToShoot = true;
//    else readyToShoot = false;
//  }
  //compute mainWheel ready to shoot
    /*
  public void computeMainWheelReadyToShoot() {
    if (shooterMainWheelLeft.getSelectedSensorVelocity()
        < (shooterMainWheelLeft.getClosedLoopTarget()+(shooterMainWheelLeft.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
        &&
        shooterMainWheelLeft.getSelectedSensorVelocity()
         > (shooterMainWheelLeft.getClosedLoopTarget()-(shooterMainWheelLeft.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
    ) mainWheelReadyToShoot = true;
    else mainWheelReadyToShoot = false;
  }
    */
  //compute hoodWheels ready to shoot
    /*
  public void computeHoodWheelsReadyToShoot() {
    if (shooterHoodWheels.getSelectedSensorVelocity()
        < (shooterHoodWheels.getClosedLoopTarget()+(shooterHoodWheels.getClosedLoopTarget()*Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR))
        &&
        shooterHoodWheels.getSelectedSensorVelocity()
        > (shooterHoodWheels.getClosedLoopTarget()-(shooterHoodWheels.getClosedLoopTarget()*Constants.SHOOTER_HOOD_WHEELS_ALLOWABLE_ERROR))
    ) hoodWheelsReady = true;
    else hoodWheelsReadyToShoot = false;
  }
    */
  //compute kickerWheel ready to shoot
    /*
  public void computeKickerWheelReadyToShoot() {
    if (shooterKickerWheel.getSelectedSensorVelocity()
        < (shooterKickerWheel.getClosedLoopTarget()+(shooterKickerWheel.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
        &&
        shooterKickerWheel.getSelectedSensorVelocity()
         > (shooterKickerWheel.getClosedLoopTarget()-(shooterKickerWheel.getClosedLoopTarget()*Constants.SHOOTER_MAIN_WHEEL_ALLOWABLE_ERROR))
    ) kickerWheelReadyToShoot = true;
    else kickerWheelReadyToShoot = false;
  }
    &/
  //compute servos ready to shoot
    /*
  public void computeServosReadyToShoot() {
    
  }
    */
//Actions
  /*public void runShooter() {
    shooterMainWheelLeft.set(ControlMode.Velocity, getMainWheelSetpoint());
  }
  */
  public void idleShooter() {
    //idle motors
    shooterMainWheelLeft.set(ControlMode.Velocity, Constants.SHOOTER_MAIN_WHEEL_IDLE_VELOCITY);
    shooterHoodWheels.set(ControlMode.Velocity, Constants.SHOOTER_HOOD_WHEELS_IDLE_VELOCITY);
    shooterKickerWheel.set(ControlMode.Velocity, Constants.SHOOTER_KICKER_WHEEL_IDLE_VELOCITY);
    //set servo position
  }
//Periodics
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //evaluate if the robot is ready to shoot
      /*
    computeShooterReadyToShoot();
      */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
