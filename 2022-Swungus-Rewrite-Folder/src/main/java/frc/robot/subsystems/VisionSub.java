// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSub extends SubsystemBase {
  UsbCamera camera4509back;
  UsbCamera camera4509front;
  /** Creates a new VisionSub. */
  public VisionSub() {
    camera4509back = CameraServer.startAutomaticCapture();
    camera4509back.setResolution(320, 240);
    camera4509back.setFPS(20);

    camera4509front = CameraServer.startAutomaticCapture();
    camera4509front.setResolution(320, 240);
    camera4509front.setFPS(20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}