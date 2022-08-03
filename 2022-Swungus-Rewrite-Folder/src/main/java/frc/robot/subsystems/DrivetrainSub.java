// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {
  //Motors
    WPI_TalonFX drivetrainLeftFront = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_FRONT_ID);
    WPI_TalonFX drivetrainLeftRear = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_REAR_ID);
    WPI_TalonFX drivetrainRightFront = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_FRONT_ID);
    WPI_TalonFX drivetrainRightRear = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_REAR_ID);

  //Drivetrain
    DifferentialDrive arcadeDrive = new DifferentialDrive(drivetrainLeftFront, drivetrainRightFront);

  //Input Filters
    SlewRateLimiter speedFilter = new SlewRateLimiter(Constants.DRIVETRAIN_SPEED_SLEW);
    SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.DRIVETRAIN_TURN_SLEW);

  //Values
    double lt;
    double rt;
    double lx;
    double speed;
    double turn;

  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    //Motor Configs
      drivetrainLeftFront.configFactoryDefault();
      drivetrainRightFront.configFactoryDefault();
      drivetrainLeftFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
      drivetrainRightFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
      drivetrainLeftFront.configVoltageCompSaturation(Constants.DRIVETRAIN_NOMINAL_VOLTAGE);
      drivetrainRightFront.configVoltageCompSaturation(Constants.DRIVETRAIN_NOMINAL_VOLTAGE);
      drivetrainLeftFront.configVoltageMeasurementFilter(Constants.DRIVETRAIN_VOLTAGE_FILTER_WINDOW_SAMPLES);
      drivetrainRightFront.configVoltageMeasurementFilter(Constants.DRIVETRAIN_VOLTAGE_FILTER_WINDOW_SAMPLES);
      drivetrainLeftFront.enableVoltageCompensation(true);
      drivetrainRightFront.enableVoltageCompensation(true);
      drivetrainLeftFront.setInverted(TalonFXInvertType.Clockwise);
      drivetrainLeftRear.setInverted(TalonFXInvertType.Clockwise);
      drivetrainRightFront.setInverted(TalonFXInvertType.CounterClockwise);
      drivetrainRightRear.setInverted(TalonFXInvertType.CounterClockwise);
      drivetrainLeftFront.setNeutralMode(NeutralMode.Coast);
      drivetrainLeftRear.setNeutralMode(NeutralMode.Coast);
      drivetrainRightFront.setNeutralMode(NeutralMode.Coast);
      drivetrainRightRear.setNeutralMode(NeutralMode.Coast);
      drivetrainLeftRear.follow(drivetrainLeftFront);
      drivetrainRightRear.follow(drivetrainRightFront);
    //Drivetrain Configs
      arcadeDrive.setDeadband(0);
  }

  public void drive(double irt, double ilt, double ilx) {
    if(irt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      rt=0;
    }
    else rt=irt;
    if(ilt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      lt=0;
    }
    else lt=ilt;
    if(Math.abs(ilx) < Constants.DRIVETRAIN_TURN_DEADZONE) {
      lx=0;
    }
    else lx=ilx;
    speed = rt-lt;
    turn = lx;
    filterValues();
    scaleValues();
    arcadeDrive();
  }

  public void filterValues() {
    speed = speedFilter.calculate(speed);
    turn = turnFilter.calculate(turn);
  }

  public void scaleValues() {
    if (speed > 0) {
      speed = (Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)+(speed)-((Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(speed));
    }
    if (speed < 0) {
      speed = (-Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)+(speed)-((-Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(speed));
    }
    if (turn > 0) {
      turn = (Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(turn)-((Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(turn));
    }
    if (turn < 0) {
      turn = (-Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(turn)-((-Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(turn));
    }
    turn = turn*Constants.DRIVETRAIN_MAX_TURN_PERCENTAGE;
    }

  public void arcadeDrive() {
    arcadeDrive.arcadeDrive(speed, turn);
  }
}
