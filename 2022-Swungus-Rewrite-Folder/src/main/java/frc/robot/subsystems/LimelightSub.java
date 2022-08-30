// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSub extends SubsystemBase {
  /** Creates a new LimelightSub. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry tv;
  public static double x;
  public static double y;
  public static double v;

  public static double turn = 0;
  
  public LimelightSub() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
  }

  public void setTurn() {
    if (v == 1 && x < 0) {
      turn = -Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT+Constants.DRIVETRAIN_TURN_kP*x;
    }
    else if (v == 1 && x > 0) {
      turn = Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT+Constants.DRIVETRAIN_TURN_kP*x;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
    setTurn();
  }
}