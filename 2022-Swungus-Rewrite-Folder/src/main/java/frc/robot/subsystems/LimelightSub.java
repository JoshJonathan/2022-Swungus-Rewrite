// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSub extends SubsystemBase {
  //Timer
    Timer timer = new Timer();
  //Entries  
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    NetworkTableEntry tpipeline;
  //Static Variables
    public static double x;
    public static double y;
    public static double v;

  public static double turn;
  
  public LimelightSub() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    tpipeline = table.getEntry("pipeline");
    tpipeline.setNumber(Constants.LIMELIGHT_STANDARD_PIPELINE);

    timer.start();
    
    turn = 0;
  }
  
  public void setTurn() {
    if (v == 1) {
      timer.reset();
      timer.start();
      if (x < -1) {
        turn = -Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT+Constants.DRIVETRAIN_TURN_kP*x;
      }
      else if (x > 1) {
        turn = Constants.DRIVETRAIN_TURN_MINIMUM_OUTPUT+Constants.DRIVETRAIN_TURN_kP*x;
      }
      else turn = 0;
    }
    else if (timer.get() > 0.06) {
      if (DrivetrainSub.lastTurnRight == false) {
        turn = -Constants.LIMELIGHT_SEARCH_SPEED;
      }
      else if (DrivetrainSub.lastTurnRight == true) {
        turn = Constants.LIMELIGHT_SEARCH_SPEED;
      }  
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