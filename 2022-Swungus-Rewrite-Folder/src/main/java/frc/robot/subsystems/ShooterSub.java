// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  //static variables
    //setpoints
    public static double mainWheelSetpoint = 0;
    public static double hoodWheelsSetpoint = 0;
    public static double kickerWheelSetpoint = 0;
    public static double servoSetpoint = 0;
    //values
    public static double mainWheelValue = 0;
    public static double hoodWheelsValue = 0;
    public static double kickerWheelValue = 0;
    public static double servoValue = 0;

  /** Creates a new ExampleSubsystem. */
  public ShooterSub() {
    //Controllers
      //mainWheelLeft
      shooterMainWheelLeft.configFactoryDefault();
      shooterMainWheelLeft.setNeutralMode(NeutralMode.Coast);
      shooterMainWheelLeft.setInverted(TalonFXInvertType.Clockwise);
      shooterMainWheelLeft.configClosedloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterMainWheelLeft.configOpenloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterMainWheelLeft.configVoltageCompSaturation(12.0);
      shooterMainWheelLeft.enableVoltageCompensation(true);
      //mainWheelRight
      shooterMainWheelRight.configFactoryDefault();
      shooterMainWheelRight.follow(shooterMainWheelLeft);
      shooterMainWheelRight.setInverted(TalonFXInvertType.OpposeMaster);
      shooterMainWheelRight.setNeutralMode(NeutralMode.Coast);
      shooterMainWheelRight.configVoltageCompSaturation(12.0);
      shooterMainWheelRight.enableVoltageCompensation(true);
      //hoodWheels
      shooterHoodWheels.configFactoryDefault();
      shooterHoodWheels.setNeutralMode(NeutralMode.Coast);
      shooterHoodWheels.setInverted(TalonFXInvertType.Clockwise);
      shooterHoodWheels.configClosedloopRamp(Constants.SHOOTER_HOOD_WHEELS_RAMP_TIME);
      shooterHoodWheels.configOpenloopRamp(Constants.SHOOTER_HOOD_WHEELS_RAMP_TIME);
      shooterHoodWheels.configVoltageCompSaturation(12.0);
      shooterHoodWheels.enableVoltageCompensation(true);
      //kickerWheel
      shooterKickerWheel.configFactoryDefault();
      shooterKickerWheel.setNeutralMode(NeutralMode.Coast);
      shooterKickerWheel.setInverted(TalonFXInvertType.CounterClockwise);
      shooterKickerWheel.configClosedloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterKickerWheel.configOpenloopRamp(Constants.SHOOTER_MAIN_WHEEL_RAMP_TIME);
      shooterKickerWheel.configVoltageCompSaturation(12.0);
      shooterKickerWheel.enableVoltageCompensation(true);
    //PID
      //mainWheel PID
      shooterMainWheelLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
      shooterMainWheelLeft.configVelocityMeasurementWindow(32);
      shooterMainWheelLeft.config_kF(0, Constants.SHOOTER_MAIN_WHEEL_KF);
      shooterMainWheelLeft.config_kP(0, Constants.SHOOTER_MAIN_WHEEL_KP);
      shooterMainWheelLeft.config_kI(0, Constants.SHOOTER_MAIN_WHEEL_KI);
      shooterMainWheelLeft.config_IntegralZone(0, Constants.SHOOTER_MAIN_WHEEL_KI_ZONE);
      shooterMainWheelLeft.config_kD(0, Constants.SHOOTER_MAIN_WHEEL_KD);
      SmartDashboard.putNumber("shooterMainWheelkF", Constants.SHOOTER_MAIN_WHEEL_KF);
      SmartDashboard.putNumber("shooterMainWheelkP", Constants.SHOOTER_MAIN_WHEEL_KP);
      SmartDashboard.putNumber("shooterMainWheelkI", Constants.SHOOTER_MAIN_WHEEL_KI);
      SmartDashboard.putNumber("shooterMainWheelkIZone", Constants.SHOOTER_MAIN_WHEEL_KI_ZONE);
      SmartDashboard.putNumber("shooterMainWheelkD", Constants.SHOOTER_MAIN_WHEEL_KD);
      //hoodWheels PID
      shooterHoodWheels.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
      shooterHoodWheels.configVelocityMeasurementWindow(32);
      shooterHoodWheels.config_kF(0, Constants.SHOOTER_HOOD_WHEELS_KF);
      shooterHoodWheels.config_kP(0, Constants.SHOOTER_HOOD_WHEELS_KP);
      //kickerWheel PID
      shooterKickerWheel.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
      shooterKickerWheel.configVelocityMeasurementWindow(32);
      shooterKickerWheel.config_kF(0, Constants.SHOOTER_KICKER_WHEEL_KF);
      shooterKickerWheel.config_kP(0, Constants.SHOOTER_KICKER_WHEEL_KP);
    //DashBoards
  }

  //Output to shooter
  public void outputToShooter(double mainWheel, double hoodWheel, double kickerWheel, double servoPosition) {
    //modify setpoints
    mainWheelSetpoint = mainWheel;
    hoodWheelsSetpoint = hoodWheel;
    kickerWheelSetpoint = kickerWheel;
    servoSetpoint = servoPosition;
    //wheels
    shooterMainWheelLeft.set(ControlMode.Velocity, mainWheel);
    shooterHoodWheels.set(ControlMode.Velocity, kickerWheel);
    shooterKickerWheel.set(ControlMode.Velocity, kickerWheel);
    //servo

    //modify values
    mainWheelValue = shooterMainWheelLeft.getSelectedSensorVelocity();
    hoodWheelsValue = shooterHoodWheels.getSelectedSensorVelocity();
    kickerWheelValue = shooterKickerWheel.getSelectedSensorVelocity();

    //SmartDashboard.putNumber("mainWheelSetpoint", mainWheelSetpoint);
    SmartDashboard.putNumber("mainWheelValue", mainWheelValue);
    SmartDashboard.putNumber("robotVoltage", shooterMainWheelLeft.getBusVoltage());
    SmartDashboard.putNumber("mainWheelVoltage", shooterMainWheelLeft.getMotorOutputVoltage());
    //SmartDashboard.putNumber("hoodWheelsSetpoint", hoodWheelsSetpoint);
    //SmartDashboard.putNumber("hoodWheelsValue", hoodWheelsValue);
    //SmartDashboard.putNumber("kickerWheelSetpoint", kickerWheelSetpoint);
    //SmartDashboard.putNumber("kickerWheelValue", kickerWheelValue);
  }
  public void setConstants() {
    shooterMainWheelLeft.config_kF(0, SmartDashboard.getNumber("shooterMainWheelkF", Constants.SHOOTER_MAIN_WHEEL_KF));
    shooterMainWheelLeft.config_kP(0, SmartDashboard.getNumber("shooterMainWheelkP", Constants.SHOOTER_MAIN_WHEEL_KP));
    shooterMainWheelLeft.config_kI(0, SmartDashboard.getNumber("shooterMainWheelkI", Constants.SHOOTER_MAIN_WHEEL_KP));
    shooterMainWheelLeft.config_IntegralZone(0, SmartDashboard.getNumber("shooterMainWheelkIZone", Constants.SHOOTER_MAIN_WHEEL_KI_ZONE));
    shooterMainWheelLeft.config_kD(0, SmartDashboard.getNumber("shooterMainWheelkD", Constants.SHOOTER_MAIN_WHEEL_KD));
  }
}