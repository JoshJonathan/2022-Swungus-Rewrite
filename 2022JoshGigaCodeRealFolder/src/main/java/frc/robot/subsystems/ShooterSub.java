// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  TalonFX mainWheelLeft = new TalonFX(Constants.MAIN_WHEEL_LEFT_DEVICE_NUMBER);
  TalonFX mainWheelRight = new TalonFX(Constants.MAIN_WHEEL_RIGHT_DEVICE_NUMBER);

  TalonFXSimCollection talonSim = new TalonFXSimCollection(mainWheelLeft);
  
  private final FlywheelSim mainWheelSim = new FlywheelSim(DCMotor.getFalcon500(2), Constants.SHOOTER_GEARING, Constants.SHOOTER_MOI);

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    //initialization
    mainWheelLeft.configFactoryDefault();
    mainWheelRight.configFactoryDefault();
    mainWheelLeft.setNeutralMode(NeutralMode.Coast);
    mainWheelRight.setNeutralMode(NeutralMode.Coast);
    mainWheelLeft.configClosedloopRamp(Constants.SHOOTER_RAMP_TIME);
    mainWheelLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mainWheelLeft.configVelocityMeasurementWindow(1);
    mainWheelLeft.configOpenloopRamp(Constants.SHOOTER_RAMP_TIME);
    mainWheelLeft.config_kF(0, Constants.SHOOTER_KF);
    mainWheelLeft.config_kP(0, Constants.SHOOTER_KP);
    mainWheelLeft.config_kD(0, 0);
    mainWheelLeft.config_kI(0, 0);
    mainWheelLeft.setInverted(TalonFXInvertType.Clockwise);
    mainWheelRight.setInverted(TalonFXInvertType.OpposeMaster);
    mainWheelRight.follow(mainWheelLeft);
  }

  public void idle() {
    mainWheelLeft.set(ControlMode.Velocity, Constants.MAIN_WHEEL_IDLE_VELOCITY);
  }

  public void shoot(double IO) {
    double maxOutput = 12/Constants.SHOOTER_KF;
    double IORatio = IO/1.0;
    double speed = IORatio*(maxOutput);
    mainWheelLeft.set(ControlMode.Velocity, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    mainWheelSim.setInput(mainWheelLeft.getMotorOutputVoltage());
    mainWheelSim.update(Constants.SHOOTER_SIM_LOOP_TIME);
    SmartDashboard.putNumber("ShooterSim Velocity", mainWheelSim.getAngularVelocityRPM());
  }
}
