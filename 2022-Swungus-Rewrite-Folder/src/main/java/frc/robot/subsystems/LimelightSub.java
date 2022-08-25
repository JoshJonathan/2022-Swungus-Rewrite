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
    if(area<0.1){
      if(lastRememberedSide)return 0.35;
      return -0.35;
    }
      

      if(area>0.1)
    lastRememberedSide = x>0;

    if(x>0)
    return (x/150)+0.19;
    return (x/150)-0.19;
  }

  public static double desiredSpeed(){
    if(area<0.1) return 0;

   return powerLimit(0-(area-0.5)*1.5,0.5);

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
