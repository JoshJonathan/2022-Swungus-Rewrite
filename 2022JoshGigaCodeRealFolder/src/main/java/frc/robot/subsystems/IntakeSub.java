// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  TalonSRX motor = new TalonSRX(Constants.DEVICE_NUMBER);
  Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.SOLENOID_CHANNEL);

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    //initialization ruitine
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(Constants.RAMP_TIME);
    motor.configVoltageCompSaturation(Constants.INTAKE_MOTOR_SATURATION_VOLTAGE);
  }

  public void deployIntake() {
    motor.set(ControlMode.PercentOutput, Constants.SPEED);
    solenoid.set(true);
  }

  public void retractIntake() {
    motor.set(ControlMode.PercentOutput, 0);
    solenoid.set(false);
  }
}
