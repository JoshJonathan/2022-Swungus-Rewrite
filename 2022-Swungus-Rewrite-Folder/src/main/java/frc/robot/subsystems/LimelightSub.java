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
  static double x;
  static double y;
  static double area;
  public LimelightSub() {

  }

  public double getX(){return x;}
  public double getY(){return y;}
  public double getArea(){return area;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimeLight desired rotation", desiredRotation());
    SmartDashboard.putNumber("LimeLight desired power", desiredSpeed());
  }

  private static boolean lastRememberedSide;
  public static double desiredRotation(){
    if(area<Constants.SWUNGUS_MIN_AREA){
      if(lastRememberedSide)return Constants.SWUNGUS_CHASE_ROTATION_SPEED;
      return 0-Constants.SWUNGUS_CHASE_ROTATION_SPEED;
    }

      if(area>Constants.SWUNGUS_MIN_AREA)
    lastRememberedSide = x>0;

    if(x>0)
    return (x*Constants.SWUNGUS_TURN_MULTIPLIER)+Constants.SWUNGUS_MIN_TURN_SPEED;
    return (x*Constants.SWUNGUS_TURN_MULTIPLIER)-Constants.SWUNGUS_MIN_TURN_SPEED;
  }

  public static double desiredSpeed(){
    if(area<Constants.SWUNGUS_MIN_AREA) return 0;

   return powerLimit(0-(area-Constants.SWUNGUS_DESIRED_AREA)*Constants.SWUNGUS_SPEED_MULTIPLIER,Constants.SWUNGUS_CHASE_MAX_SPEED);

  }

  private static double powerLimit(double in, double limit){
    limit = Math.abs(limit);
    if(Math.abs(in)>limit){
      if(in<0) return 0-limit;
      return limit;
    }
    return in;
  }



}
