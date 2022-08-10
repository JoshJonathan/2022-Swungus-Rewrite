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
  //Motor Controllers
    //left
    WPI_TalonFX drivetrainLeftFront = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_FRONT_ID);
    WPI_TalonFX drivetrainLeftRear = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_REAR_ID);
    //right
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
    //Motor Controller Configs
      //Left
        //Front
        drivetrainLeftFront.configFactoryDefault();
        drivetrainLeftFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
        drivetrainLeftFront.configVoltageCompSaturation(Constants.DRIVETRAIN_NOMINAL_VOLTAGE);
        drivetrainLeftFront.configVoltageMeasurementFilter(Constants.DRIVETRAIN_VOLTAGE_FILTER_WINDOW_SAMPLES);
        drivetrainLeftFront.enableVoltageCompensation(true);
        drivetrainLeftFront.setInverted(TalonFXInvertType.Clockwise);
        drivetrainLeftFront.setNeutralMode(NeutralMode.Coast);
        //Rear
        drivetrainLeftRear.follow(drivetrainLeftFront);
        drivetrainLeftRear.setInverted(TalonFXInvertType.FollowMaster);
        drivetrainLeftRear.setNeutralMode(NeutralMode.Coast);
      //Right
        //Front
        drivetrainRightFront.configFactoryDefault();
        drivetrainRightFront.configNeutralDeadband(Constants.DRIVETRAIN_NEUTRAL_DEADBAND);
        drivetrainRightFront.configVoltageCompSaturation(Constants.DRIVETRAIN_NOMINAL_VOLTAGE);
        drivetrainRightFront.configVoltageMeasurementFilter(Constants.DRIVETRAIN_VOLTAGE_FILTER_WINDOW_SAMPLES);
        drivetrainRightFront.enableVoltageCompensation(true);
        drivetrainRightFront.setInverted(TalonFXInvertType.CounterClockwise);
        drivetrainRightFront.setNeutralMode(NeutralMode.Coast);
        //Rear
        drivetrainRightRear.follow(drivetrainRightFront);
        drivetrainRightRear.setInverted(TalonFXInvertType.FollowMaster);
        drivetrainRightRear.setNeutralMode(NeutralMode.Coast);
    //Drivetrain Configs
      arcadeDrive.setDeadband(0);
  }

  //Drive
  public void drive(double irt, double ilt, double ilx) {
    deadzoneInputs(irt, ilt, ilx);
    simplifyInputs();
    filterValues();
    scaleValues();
    arcadeDrive();
  }

  //Deadzone Inputs
  public void deadzoneInputs(double irt, double ilt, double ilx) {
    if(irt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      rt=0;
    } else rt=irt;

    if(ilt < Constants.DRIVETRAIN_SPEED_DEADZONE) {
      lt=0;
    } else lt=ilt;

    if(Math.abs(ilx) < Constants.DRIVETRAIN_TURN_DEADZONE) {
      lx=0;
    } else lx=ilx;
  }

  //Simplify Inputs
  public void simplifyInputs() {
    speed = rt-lt;
    turn = lx;
  }

  //filter Inputs
  public void filterValues() {
    speed = speedFilter.calculate(speed);
    turn = turnFilter.calculate(turn);
  }

  
  //Scale Values
    /*
      These equations modify the speed and turn inputs by adding a constant which serves to decrease looseness in the sticks,
      and by linearly scaling the new inputs such that the max output on the controller still cooresponds to max output on the motor,
      rather than max output plus the constant that was added before.
    */
  public void scaleValues() {
    //Speed
    if (speed > 0) {
      speed = (Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)+(speed)-((Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(speed));
    }
    if (speed < 0) {
      speed = (-Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)+(speed)-((-Constants.DRIVETRAIN_SPEED_MINIMUM_OUTPUT)*(speed));
    }
    //Turn
    if (turn > 0) {
      turn = (Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(turn)-((Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(turn));
    }
    if (turn < 0) {
      turn = (-Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)+(turn)-((-Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT)*(turn));
    }
    turn = turn*Constants.DRIVETRAIN_MAX_TURN_PERCENTAGE;
  }

  //Arcade Drive
  public void arcadeDrive() {
    arcadeDrive.arcadeDrive(speed, turn);
  }
}
