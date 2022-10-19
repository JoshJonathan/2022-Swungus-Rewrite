// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  //Solenoid
    Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_CHANNEL);
  //Motor Controller 
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_TALON_ID);

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    //Motor Controller Configs
      intakeMotor.configFactoryDefault();
      intakeMotor.configOpenloopRamp(Constants.INTAKE_RAMP_TIME);
      intakeMotor.configVoltageCompSaturation(Constants.INTAKE_NOMINAL_ROBOT_VOLTAGE);
      intakeMotor.configVoltageMeasurementFilter(Constants.INTAKE_VOLTAGE_FILTER_WINDOW_SAMPLES);
      intakeMotor.enableVoltageCompensation(true);
      intakeMotor.setInverted(true);
      intakeMotor.setNeutralMode(NeutralMode.Coast);
  }
  
  //Deploy Intake
  public void deployIntake() {
    intakeSolenoid.set(true);
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_MOTOR_OUTPUT);
  }

  //Retract Intake
  public void retractIntake(){
    intakeSolenoid.set(false);
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}