// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX shooterMainWheelLeft = new TalonFX(Constants.SHOOTER_MAIN_WHEEL_LEFT_ID);
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shooterMainWheelLeft.configFactoryDefault();
    shooterMainWheelLeft.configClosedloopRamp(.25);
    shooterMainWheelLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    shooterMainWheelLeft.configVelocityMeasurementWindow(1);
    shooterMainWheelLeft.config_kF(0, Constants.SHOOTER_MAIN_WHEEL_KF);
    shooterMainWheelLeft.config_kP(0, Constants.SHOOTER_MAIN_WHEEL_KP);
    shooterMainWheelLeft.config_kD(0, 0);
    shooterMainWheelLeft.config_kI(0, 0);
  }

  public void spinWheels() {
    shooterMainWheelLeft.set(TalonFXControlMode.Velocity, Constants.SHOOTER_VELOCITY);
  }

  public void stop() {
    shooterMainWheelLeft.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    System.out.print("SHOOTER_MAIN_WHEEL_VELOCITY" + shooterMainWheelLeft.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
