// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  Solenoid climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_SOLENOID_CHANNEL);
  /** Creates a new ClimbSub. */
  public ClimbSub() {
  }

  public void extendClimb() {
    climbSolenoid.set(true);
  }
  public void retractClimb() {
    climbSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
